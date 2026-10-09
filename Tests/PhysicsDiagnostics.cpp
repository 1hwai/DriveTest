#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../Core/Debug/Logger.h"
#include "../Core/Math/Mat3.h"
#include "../Core/Math/Vec3.h"
#include "../Physics/Collider.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Physics/Terrain.h"
#include "../Vehicle/Car.h"
#include "../Vehicle/VehicleConfig.h"

namespace {
    constexpr float FixedDeltaTime = 1.0f / 120.0f;
    constexpr int SimulationSteps = 120 * 40;

    struct WheelEnergySnapshot {
        float compression = 0.0f;
        float compressionVelocity = 0.0f;
        float springForce = 0.0f;
        float damperForce = 0.0f;
        float suspensionPower = 0.0f;
        float suspensionResidual = 0.0f;
        float suspensionLength = 0.0f;
        bool grounded = false;
        Vec3 mountA;
        Vec3 mountB;
        Vec3 mountAVelocity;
        Vec3 mountBVelocity;
        Vec3 relativeVelocity;
        Vec3 springAxis;
        Vec3 forceOnChassis;
        float powerAtA = 0.0f;
        float powerAtB = 0.0f;
        float springPower = 0.0f;
        float damperPower = 0.0f;
        float powerBalanceResidual = 0.0f;
    };

    struct EnergySnapshot {
        bool valid = false;
        int step = 0;
        float time = 0.0f;
        float total = 0.0f;
        float potential = 0.0f;
        float linear = 0.0f;
        float angular = 0.0f;
        float suspension = 0.0f;
        Vec3 position;
        Vec3 velocity;
        Vec3 angularVelocity;
        std::array<WheelEnergySnapshot, WheelCount> wheels;
    };

    bool IsFinite(const Vec3& value) {
        return std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    float CalculateMechanicalEnergy(
        const RigidBody& body,
        float gravityMagnitude
    ) {
        const float potential =
            body.GetMass() * gravityMagnitude * body.GetPosition().y;
        const float kinetic =
            0.5f * body.GetMass() *
            body.GetLinearVelocity().LengthSquared();
        return potential + kinetic;
    }

    float MeasureFreeFallEnergyError(float deltaTime) {
        constexpr float GravityMagnitude = 9.81f;
        constexpr float Duration = 2.0f;
        constexpr int InitialHeight = 100;

        RigidBody body;
        body.SetMass(1.0f);
        body.SetPosition(Vec3(0.0f, static_cast<float>(InitialHeight), 0.0f));

        const float initialEnergy =
            CalculateMechanicalEnergy(body, GravityMagnitude);
        const Vec3 gravity(0.0f, -GravityMagnitude, 0.0f);
        const int steps = static_cast<int>(std::round(Duration / deltaTime));

        for (int step = 0; step < steps; ++step)
            body.Integrate(deltaTime, gravity);

        return std::abs(
            CalculateMechanicalEnergy(body, GravityMagnitude) -
            initialEnergy
        );
    }
}

int main() {
    Logger::Initialize("Logs/physics_diagnostics.log");

    PhysicsWorld physicsWorld;

    // Suspension stability is tested on a flat surface.
    // A sine surface introduces a real gravity component along the slope,
    // which would make the chassis move in Z even with tire forces disabled.
    Terrain terrain(
        65,
        100.0f,
        0.0f,
        0.0f,
        1,
        1337
    );

    RigidBody* terrainBody =
        physicsWorld.CreateRigidBody();

    terrainBody->SetMass(0.0f);

    Collider* terrainCollider =
        physicsWorld.CreateCollider();

    terrainCollider->SetShape(
        ColliderShape::Terrain
    );
    terrainCollider->SetTerrain(&terrain);
    terrainCollider->SetRigidBody(terrainBody);

    RigidBody* chassis =
        physicsWorld.CreateRigidBody();

    chassis->SetMass(1200.0f);
    chassis->SetBoxInertia(
        Vec3(2.0f, 1.0f, 3.0f)
    );
    chassis->SetPosition(
        Vec3(0.0f, 0.42f, 0.0f)
    );

    // The gravity-well test isolates suspension forces from chassis-terrain
    // contact resolution. Chassis collision is tested separately by the
    // rigid-body terrain diagnostics.
    constexpr bool EnableChassisTerrainCollision = false;

    if (EnableChassisTerrainCollision) {
        Collider* chassisCollider =
            physicsWorld.CreateCollider();

        chassisCollider->SetShape(
            ColliderShape::Box
        );
        chassisCollider->SetHalfExtents(
            Vec3(1.0f, 0.5f, 1.5f)
        );
        chassisCollider->SetRigidBody(chassis);
    }

    Car car;
    car.SetChassis(chassis);

    VehicleConfig vehicleConfig;
    car.ApplyConfig(vehicleConfig);

    // This diagnostic is for suspension stability only. Tire forces are
    // tested by VehicleDiagnostics and would otherwise turn the gravity-well
    // test into a combined suspension+tire test.
    for (size_t i = 0; i < WheelCount; ++i) {
        Tire& tire = car.GetTire(static_cast<WheelIndex>(i));
        tire.SetStaticFriction(0.0f);
        tire.SetDynamicFriction(0.0f);
        tire.SetLongitudinalStiffness(0.0f);
        tire.SetLateralStiffness(0.0f);
        tire.SetRollingResistance(0.0f);
    }

    constexpr float MaxDriftZ = 0.05f;
    constexpr float EnergyAbsoluteTolerance = 2.0f;
    constexpr float EnergyRelativeTolerance = 0.001f;

    // Establish a contact-consistent suspension state before measuring E0.
    // ApplyConfig initializes mounts at zero travel; the first normal update
    // can choose a different travel from ground contact. Counting that
    // kinematic initialization jump as simulated energy gain gives a false
    // conservation failure. A zero-dt update sets geometry/contact/compression
    // without integrating the chassis; clear the forces it queued afterwards.
    car.UpdatePhysics(physicsWorld, 0.0f);
    chassis->ClearForces();

    float maxAbsZ = 0.0f;
    float minY = chassis->GetPosition().y;
    float initialEnergy =
        chassis->GetMass() *
        9.81f *
        chassis->GetPosition().y;
    for (size_t i = 0; i < WheelCount; ++i) {
        const Suspension& suspension =
            car.GetSuspension(static_cast<WheelIndex>(i));
        const float compression = suspension.GetCompression();
        initialEnergy += 0.5f *
            suspension.GetSpringRate() *
            compression * compression;
    }
    const float energyTolerance = std::max(
        EnergyAbsoluteTolerance,
        std::abs(initialEnergy) * EnergyRelativeTolerance
    );
    float maxEnergy = initialEnergy;

    const float coarseFreeFallError = MeasureFreeFallEnergyError(1.0f / 60.0f);
    const float fineFreeFallError = MeasureFreeFallEnergyError(1.0f / 120.0f);
    {
        std::ostringstream log;
        log << std::fixed << std::setprecision(6)
            << "[EnergyConvergence] dtCoarse=" << (1.0f / 60.0f)
            << " errorCoarse=" << coarseFreeFallError
            << " dtFine=" << (1.0f / 120.0f)
            << " errorFine=" << fineFreeFallError;
        Logger::Info(log.str());
    }
    if (!std::isfinite(coarseFreeFallError) ||
        !std::isfinite(fineFreeFallError) ||
        !(fineFreeFallError < coarseFreeFallError * 0.75f)) {
        Logger::Error(
            "[FAIL] Free-fall energy error did not decrease with timestep refinement"
        );
        Logger::Shutdown();
        return 1;
    }
    Logger::Info("[PASS] Free-fall energy error decreases with timestep refinement");

    {
        std::ostringstream log;
        log << std::fixed << std::setprecision(6)
            << "[EnergyBaseline] total=" << initialEnergy
            << " tolerance=" << energyTolerance
            << " absTolerance=" << EnergyAbsoluteTolerance
            << " relativeTolerance=" << EnergyRelativeTolerance;
        Logger::Info(log.str());
    }

    static const char* wheelNames[WheelCount] = {"FL", "FR", "RL", "RR"};
    EnergySnapshot peakSnapshot;
    EnergySnapshot firstExceedSnapshot;

    std::array<Vec3, WheelCount> previousMountA{};
    std::array<Vec3, WheelCount> previousMountB{};
    std::array<Vec3, WheelCount> currentMountAVelocity{};
    std::array<Vec3, WheelCount> currentMountBVelocity{};
    std::array<float, WheelCount> currentPowerAtA{};
    std::array<float, WheelCount> currentPowerAtB{};
    std::array<float, WheelCount> currentSpringPower{};
    std::array<float, WheelCount> currentDamperPower{};
    std::array<float, WheelCount> currentPowerBalanceResidual{};
    float integratedPowerAtA = 0.0f;
    float integratedPowerAtB = 0.0f;
    float integratedSpringPower = 0.0f;
    float integratedDamperPower = 0.0f;
    float previousTotalEnergy = initialEnergy;
    float maxAbsEnergyStepDelta = 0.0f;

    for (size_t i = 0; i < WheelCount; ++i) {
        const DoubleWishbone& geometry =
            car.GetSuspensionGeometry(static_cast<WheelIndex>(i));
        previousMountA[i] = geometry.GetSpringMountA();
        previousMountB[i] = geometry.GetSpringMountB();
    }

    Logger::Info("[PhysicsDiagnostics] flat terrain suspension stability test");

    for (int step = 0;
        step < SimulationSteps;
        ++step) {

        car.UpdatePhysics(
            physicsWorld,
            FixedDeltaTime
        );

        // The spring endpoints are not both dynamic rigid bodies: mount A is
        // attached to the chassis and mount B is produced by kinematic
        // suspension geometry. Estimate endpoint velocities from consecutive
        // solved mount positions so their work can be accounted for separately.
        for (size_t i = 0; i < WheelCount; ++i) {
            const WheelIndex index = static_cast<WheelIndex>(i);
            const DoubleWishbone& geometry = car.GetSuspensionGeometry(index);
            const Wheel& wheel = car.GetWheel(index);
            const Suspension& suspension = car.GetSuspension(index);
            const Vec3 mountA = geometry.GetSpringMountA();
            const Vec3 mountB = geometry.GetSpringMountB();
            currentMountAVelocity[i] =
                (mountA - previousMountA[i]) / FixedDeltaTime;
            currentMountBVelocity[i] =
                (mountB - previousMountB[i]) / FixedDeltaTime;

            const Vec3 forceOnChassis =
                suspension.CalculateForceVector(mountA, mountB) * -1.0f;
            const Vec3 forceAtB = forceOnChassis * -1.0f;
            currentPowerAtA[i] =
                forceOnChassis.Dot(currentMountAVelocity[i]);
            currentPowerAtB[i] =
                forceAtB.Dot(currentMountBVelocity[i]);
            currentSpringPower[i] =
                wheel.GetSpringForce() * wheel.GetCompressionVelocity();
            currentDamperPower[i] =
                wheel.GetDamperForce() * wheel.GetCompressionVelocity();
            currentPowerBalanceResidual[i] =
                currentPowerAtA[i] + currentPowerAtB[i] +
                currentSpringPower[i] + currentDamperPower[i];

            previousMountA[i] = mountA;
            previousMountB[i] = mountB;
        }

        // Measure energy before stepping so chassis state and suspension
        // geometry/compression describe the same simulation time.
        const Vec3 position = chassis->GetPosition();
        const Vec3 velocity = chassis->GetLinearVelocity();

        minY =
            std::min(minY, position.y);

        maxAbsZ =
            std::max(maxAbsZ, std::abs(position.z));

        const float potentialEnergy =
            chassis->GetMass() *
            9.81f *
            position.y;

        const float linearEnergy =
            0.5f *
            chassis->GetMass() *
            velocity.LengthSquared();

        const Mat3 rotation =
            chassis->GetOrientation().ToMat3();

        const Mat3 inertiaWorld =
            rotation *
            chassis->GetInertiaTensor() *
            rotation.Transposed();

        const Vec3 angularVelocity =
            chassis->GetAngularVelocity();

        const float angularEnergy =
            0.5f *
            angularVelocity.Dot(
                inertiaWorld *
                angularVelocity
            );

        float suspensionEnergy = 0.0f;

        for (size_t i = 0;
            i < WheelCount;
            ++i) {

            const float compression =
                car.GetSuspension(
                    static_cast<WheelIndex>(i)
                ).GetCompression();

            suspensionEnergy +=
                0.5f *
                car.GetSuspension(
                    static_cast<WheelIndex>(i)
                ).GetSpringRate() *
                compression *
                compression;
        }

        const float totalEnergy =
            potentialEnergy +
            linearEnergy +
            angularEnergy +
            suspensionEnergy;

        maxEnergy =
            std::max(maxEnergy, totalEnergy);
        const float energyStepDelta = totalEnergy - previousTotalEnergy;
        maxAbsEnergyStepDelta =
            std::max(maxAbsEnergyStepDelta, std::abs(energyStepDelta));
        previousTotalEnergy = totalEnergy;

        // The endpoint velocities and spring/damper powers above describe
        // the interval ending at this sampled state. Accumulate before logging
        // so cumulative work and energy use the same time boundary.
        for (size_t i = 0; i < WheelCount; ++i) {
            integratedPowerAtA += currentPowerAtA[i] * FixedDeltaTime;
            integratedPowerAtB += currentPowerAtB[i] * FixedDeltaTime;
            integratedSpringPower += currentSpringPower[i] * FixedDeltaTime;
            integratedDamperPower += currentDamperPower[i] * FixedDeltaTime;
        }

        if (step % 60 == 0) {
            std::ostringstream energyDeltaLog;
            energyDeltaLog << std::fixed << std::setprecision(6)
                << "[EnergyDelta] step=" << step
                << " t=" << step * FixedDeltaTime
                << " deltaFromBaseline=" << (totalEnergy - initialEnergy)
                << " stepDelta=" << energyStepDelta
                << " integratedPowerA=" << integratedPowerAtA
                << " integratedPowerB=" << integratedPowerAtB
                << " integratedSpringPower=" << integratedSpringPower
                << " integratedDamperPower=" << integratedDamperPower
                << " integratedPowerBalanceResidual="
                << (integratedPowerAtA + integratedPowerAtB +
                    integratedSpringPower + integratedDamperPower);
            Logger::Info(energyDeltaLog.str());
        }

        auto captureEnergySnapshot = [&](EnergySnapshot& snapshot) {
            snapshot.valid = true;
            snapshot.step = step;
            snapshot.time = step * FixedDeltaTime;
            snapshot.total = totalEnergy;
            snapshot.potential = potentialEnergy;
            snapshot.linear = linearEnergy;
            snapshot.angular = angularEnergy;
            snapshot.suspension = suspensionEnergy;
            snapshot.position = position;
            snapshot.velocity = velocity;
            snapshot.angularVelocity = angularVelocity;

            for (size_t i = 0; i < WheelCount; ++i) {
                const WheelIndex index = static_cast<WheelIndex>(i);
                const Wheel& wheel = car.GetWheel(index);
                const Suspension& suspension = car.GetSuspension(index);
                const DoubleWishbone& geometry = car.GetSuspensionGeometry(index);
                WheelEnergySnapshot& item = snapshot.wheels[i];
                item.compression = suspension.GetCompression();
                item.compressionVelocity = suspension.GetCompressionVelocity();
                item.springForce = wheel.GetSpringForce();
                item.damperForce = wheel.GetDamperForce();
                item.suspensionPower = wheel.GetSuspensionPower();
                item.suspensionResidual = wheel.GetSuspensionResidual();
                item.suspensionLength = suspension.GetLength();
                item.grounded = wheel.IsGrounded();
                item.mountA = geometry.GetSpringMountA();
                item.mountB = geometry.GetSpringMountB();
                item.mountAVelocity = currentMountAVelocity[i];
                item.mountBVelocity = currentMountBVelocity[i];
                item.relativeVelocity =
                    item.mountBVelocity - item.mountAVelocity;
                item.springAxis =
                    (item.mountB - item.mountA).Normalized();
                item.forceOnChassis =
                    suspension.CalculateForceVector(item.mountA, item.mountB) * -1.0f;
                item.powerAtA = currentPowerAtA[i];
                item.powerAtB = currentPowerAtB[i];
                item.springPower = currentSpringPower[i];
                item.damperPower = currentDamperPower[i];
                item.powerBalanceResidual = currentPowerBalanceResidual[i];
            }
        };

        if (!peakSnapshot.valid || totalEnergy > peakSnapshot.total)
            captureEnergySnapshot(peakSnapshot);
        if (!firstExceedSnapshot.valid &&
            totalEnergy > initialEnergy + energyTolerance)
            captureEnergySnapshot(firstExceedSnapshot);

        if (step % 60 == 0) {
            std::ostringstream log;
            log << std::fixed << std::setprecision(6)
                << "[State] t="
                << step * FixedDeltaTime
                << " y="
                << position.y
                << " z="
                << position.z
                << " vy="
                << velocity.y
                << " vz="
                << velocity.z
                << " energy="
                << totalEnergy
                << " compression=";

            for (size_t i = 0;
                i < WheelCount;
                ++i) {

                log
                    << car.GetWheel(
                        static_cast<WheelIndex>(i)
                    ).GetCompression();

                if (i + 1 < WheelCount)
                    log << ',';
            }

            Logger::Info(log.str());
        }

        // Advance only after recording the synchronized pre-step state.
        physicsWorld.Step(FixedDeltaTime);

        const Vec3 steppedPosition = chassis->GetPosition();
        const Vec3 steppedVelocity = chassis->GetLinearVelocity();
        if (!IsFinite(steppedPosition) ||
            !IsFinite(steppedVelocity) ||
            !std::isfinite(chassis->GetAngularVelocity().LengthSquared())) {
            std::cerr
                << "[FAIL] Non-finite chassis state at step "
                << step << '\n';
            Logger::Shutdown();
            return 1;
        }

    }

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(6)
        << "[Summary] minY="
        << minY
        << " maxAbsZ="
        << maxAbsZ
        << " maxEnergy="
        << maxEnergy
        << " maxAbsEnergyStepDelta=" << maxAbsEnergyStepDelta
        << " finalMeasuredEnergy=" << previousTotalEnergy
        << " measuredEnergyDelta=" << (previousTotalEnergy - initialEnergy)
        << " integratedPowerA=" << integratedPowerAtA
        << " integratedPowerB=" << integratedPowerAtB
        << " integratedSpringPower=" << integratedSpringPower
        << " integratedDamperPower=" << integratedDamperPower
        << " integratedPowerBalanceResidual="
        << (integratedPowerAtA + integratedPowerAtB +
            integratedSpringPower + integratedDamperPower)
        << " finalY="
        << chassis->GetPosition().y
        << " finalZ="
        << chassis->GetPosition().z;

    Logger::Info(summary.str());

    auto logEnergySnapshot = [&](const char* label, const EnergySnapshot& snapshot) {
        if (!snapshot.valid)
            return;

        std::ostringstream log;
        log << std::fixed << std::setprecision(6)
            << "[" << label << "] step=" << snapshot.step
            << " t=" << snapshot.time
            << " total=" << snapshot.total
            << " potential=" << snapshot.potential
            << " linear=" << snapshot.linear
            << " angular=" << snapshot.angular
            << " spring=" << snapshot.suspension
            << " y=" << snapshot.position.y
            << " vy=" << snapshot.velocity.y
            << " vz=" << snapshot.velocity.z
            << " wx=" << snapshot.angularVelocity.x
            << " wy=" << snapshot.angularVelocity.y
            << " wz=" << snapshot.angularVelocity.z;
        Logger::Info(log.str());

        for (size_t i = 0; i < WheelCount; ++i) {
            const WheelEnergySnapshot& wheel = snapshot.wheels[i];
            std::ostringstream wheelLog;
            wheelLog << std::fixed << std::setprecision(6)
                << "[" << label << "Wheel] name=" << wheelNames[i]
                << " grounded=" << wheel.grounded
                << " length=" << wheel.suspensionLength
                << " compression=" << wheel.compression
                << " compressionVelocity=" << wheel.compressionVelocity
                << " springForce=" << wheel.springForce
                << " damperForce=" << wheel.damperForce
                << " suspensionPower=" << wheel.suspensionPower
                << " residual=" << wheel.suspensionResidual
                << " mountA=(" << wheel.mountA.x << "," << wheel.mountA.y << "," << wheel.mountA.z << ")"
                << " mountB=(" << wheel.mountB.x << "," << wheel.mountB.y << "," << wheel.mountB.z << ")"
                << " mountAVel=(" << wheel.mountAVelocity.x << "," << wheel.mountAVelocity.y << "," << wheel.mountAVelocity.z << ")"
                << " mountBVel=(" << wheel.mountBVelocity.x << "," << wheel.mountBVelocity.y << "," << wheel.mountBVelocity.z << ")"
                << " relativeVel=(" << wheel.relativeVelocity.x << "," << wheel.relativeVelocity.y << "," << wheel.relativeVelocity.z << ")"
                << " springAxis=(" << wheel.springAxis.x << "," << wheel.springAxis.y << "," << wheel.springAxis.z << ")"
                << " powerA=" << wheel.powerAtA
                << " powerB=" << wheel.powerAtB
                << " springPower=" << wheel.springPower
                << " damperPower=" << wheel.damperPower
                << " powerBalanceResidual=" << wheel.powerBalanceResidual
                << " force=(" << wheel.forceOnChassis.x << "," << wheel.forceOnChassis.y << "," << wheel.forceOnChassis.z << ")";
            Logger::Info(wheelLog.str());
        }
    };

    logEnergySnapshot("EnergyPeak", peakSnapshot);
    logEnergySnapshot("EnergyFirstExceed", firstExceedSnapshot);

    if (maxAbsZ > MaxDriftZ) {
        Logger::Error(
            "[FAIL] Unexpected chassis drift on flat terrain: maxAbsZ=" +
            std::to_string(maxAbsZ)
        );

        Logger::Shutdown();
        return 1;
    }

    if (maxEnergy > initialEnergy + energyTolerance) {
        Logger::Error(
            "[FAIL] Mechanical energy increased above initial energy"
        );

        Logger::Shutdown();
        return 1;
    }

    Logger::Info(
        "[PASS] Suspension remained stable and energy did not increase"
    );

    Logger::Shutdown();

    return 0;
}
