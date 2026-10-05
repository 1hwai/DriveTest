#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "../Core/Debug/Logger.h"
#include "../Physics/Collider.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Physics/Road.h"
#include "../Physics/Terrain.h"
#include "../Vehicle/Car.h"
#include "../Vehicle/VehicleCoordinates.h"
#include "../Vehicle/VehicleConfig.h"

namespace {
    constexpr float FixedDeltaTime = 1.0f / 120.0f;
    constexpr float TestDuration = 10.0f;
    constexpr float Pi = 3.14159265358979323846f;

    struct VehicleTestInput {
        float throttle;
        float brake;
        float steering;
        float clutch;
    };

    struct VehicleTestRig {
        PhysicsWorld physicsWorld;
        RigidBody* chassis;
        Car car;
        VehicleConfig config;
        bool configLoaded;
        std::string configError;

        VehicleTestRig()
            : chassis(nullptr), configLoaded(false) {
            Collider* ground = physicsWorld.CreateCollider();
            ground->SetShape(ColliderShape::Plane);
            ground->SetPlaneHeight(0.0f);

            configLoaded = config.Load(
                std::string(DRIVETEST_PROJECT_ROOT) + "/Assets/Vehicles/TestCar/vehicle.ini",
                configError
            );
            chassis = physicsWorld.CreateRigidBody();
            chassis->SetMass(config.mass);
            chassis->SetBoxInertia(config.colliderHalfExtents * 2.0f);
            chassis->SetPosition(Vec3(0.0f, config.spawnClearance, 0.0f));
            car.SetChassis(chassis);
            car.ApplyConfig(config);
        }
    };

    float LaunchClutch(float time) {
        if (time < 0.75f)
            return 1.0f;

        if (time < 2.75f) {
            return std::clamp(
                1.0f - (time - 0.75f) / 2.0f,
                0.0f,
                1.0f
            );
        }

        if (std::abs(time - 3.5f) < 0.20f ||
            std::abs(time - 6.0f) < 0.20f)
            return 1.0f;

        return 0.0f;
    }

    VehicleTestInput NeutralInput(float) {
        return { 0.0f, 0.0f, 0.0f, 1.0f };
    }

    VehicleTestInput LaunchInput(float time) {
        return {
            std::clamp(time * 0.35f, 0.0f, 1.0f),
            0.0f,
            0.0f,
            LaunchClutch(time)
        };
    }

    VehicleTestInput BrakeInput(float time) {
        if (time < 5.0f)
            return LaunchInput(time);

        return {
            0.0f,
            std::clamp((time - 5.0f) * 0.8f, 0.0f, 1.0f),
            0.0f,
            0.0f
        };
    }

    VehicleTestInput GentleSlalomInput(float time) {
        return {
            std::clamp(time * 0.25f, 0.0f, 0.55f),
            0.0f,
            0.12f * std::sin(time * 0.65f * 2.0f * Pi),
            LaunchClutch(time)
        };
    }

    int LaunchGear(float time) {
        if (time >= 6.0f)
            return 3;
        if (time >= 3.5f)
            return 2;
        return 1;
    }

    int BrakeGear(float time) {
        return time >= 3.5f ? 2 : 1;
    }

    int SlalomGear(float time) {
        return time >= 4.5f ? 2 : 1;
    }

    bool IsFinite(const Vec3& value) {
        return std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    float GetUpDot(const RigidBody& body) {
        return (
            body.GetOrientation() *
            Vec3(0.0f, 1.0f, 0.0f)
        ).y;
    }

    void ShiftToGear(Car& car, int targetGear) {
        while (car.GetTransmission().GetGear() < targetGear)
            car.GetTransmission().ShiftUp();

        while (car.GetTransmission().GetGear() > targetGear)
            car.GetTransmission().ShiftDown();
    }

    bool RunRaycastRollGeometryDiagnostics() {
        PhysicsWorld planeWorld;
        Collider* plane = planeWorld.CreateCollider();
        plane->SetShape(ColliderShape::Plane);
        plane->SetPlaneHeight(0.0f);

        RigidBody body;
        body.SetPosition(Vec3(0.0f, 1.5f, 0.0f));

        const Vec3 wheelLocalPositions[] = {
            VehicleCoordinates::LeftWheelPosition(0.76f, -0.10f, 1.25f),
            VehicleCoordinates::RightWheelPosition(0.76f, -0.10f, 1.25f),
            VehicleCoordinates::LeftWheelPosition(0.76f, -0.10f, -1.25f),
            VehicleCoordinates::RightWheelPosition(0.76f, -0.10f, -1.25f)
        };
        const char* wheelNames[] = { "FL", "FR", "RL", "RR" };

        auto TraceWheel = [&](int index, const Quaternion& orientation, float& distance) {
            body.SetOrientation(orientation);
            const Vec3 origin =
                body.GetPosition() +
                body.GetOrientation() * wheelLocalPositions[index];
            const Vec3 direction =
                body.GetOrientation() * Vec3(0.0f, -1.0f, 0.0f);
            Ray ray{ origin, direction };
            RaycastResult result;
            const bool hit = planeWorld.Raycast(ray, result, 5.0f, &body);

            std::ostringstream log;
            log << std::fixed << std::setprecision(5)
                << "[RaycastRollTest] wheel=" << wheelNames[index]
                << " hit=" << hit
                << " origin=(" << origin.x << "," << origin.y << "," << origin.z << ")"
                << " direction=(" << direction.x << "," << direction.y << "," << direction.z << ")";
            if (hit) {
                const Vec3 reconstructed = origin + direction * result.distance;
                log << " distance=" << result.distance
                    << " expectedPlaneDistance=" << origin.y / -direction.y
                    << " point=(" << result.point.x << "," << result.point.y << "," << result.point.z << ")"
                    << " normal=(" << result.normal.x << "," << result.normal.y << "," << result.normal.z << ")"
                    << " pointError=" << (reconstructed - result.point).Length();
                distance = result.distance;
            }
            Logger::Info(log.str());

            if (!hit || direction.y >= -0.1f)
                return false;

            const float expectedDistance = origin.y / -direction.y;
            return std::abs(result.distance - expectedDistance) < 0.0001f &&
                std::abs(result.point.y) < 0.0001f &&
                (origin + direction * result.distance - result.point).Length() < 0.0001f;
        };

        bool passed = true;
        for (float rollAngle : { 0.10f, -0.10f }) {
            const Quaternion orientation =
                Quaternion::FromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), rollAngle);
            float distances[4] = {};
            for (int i = 0; i < 4; ++i)
                passed = TraceWheel(i, orientation, distances[i]) && passed;

            const bool positiveRollExpected =
                rollAngle > 0.0f
                ? distances[0] < distances[1] && distances[2] < distances[3]
                : distances[0] > distances[1] && distances[2] > distances[3];

            std::ostringstream summary;
            summary << std::fixed << std::setprecision(5)
                << "[RaycastRollSummary] rollAngle=" << rollAngle
                << " FL=" << distances[0]
                << " FR=" << distances[1]
                << " RL=" << distances[2]
                << " RR=" << distances[3]
                << " expectedSideOrdering=" << positiveRollExpected;
            Logger::Info(summary.str());

            if (!positiveRollExpected) {
                Logger::Error("[FAIL] Raycast roll geometry produced incorrect left/right distance ordering");
                passed = false;
            }
        }

        if (passed)
            Logger::Info("[PASS] Raycast roll geometry diagnostics");
        else
            Logger::Error("[FAIL] Raycast roll geometry diagnostics");
        return passed;
    }

    bool RunRaycastSurfaceDiagnostics() {
        bool passed = true;

        {
            Terrain flatTerrain(33, 100.0f, 0.0f, 0.035f, 1, 1337);
            PhysicsWorld world;
            Collider* collider = world.CreateCollider();
            collider->SetShape(ColliderShape::Terrain);
            collider->SetTerrain(&flatTerrain);

            RaycastResult result;
            const Ray ray{ Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f) };
            const bool hit = world.Raycast(ray, result, 3.0f);
            const float error = hit ? std::abs(result.distance - 1.0f) : INFINITY;
            Logger::Info("[RaycastSurfaceTest] shape=Terrain hit=" +
                std::to_string(hit) + " distance=" +
                (hit ? std::to_string(result.distance) : std::string("none")) +
                " expected=1.00000 error=" + std::to_string(error));
            if (!hit || error > 0.02f || std::abs(result.point.y) > 0.02f) {
                Logger::Error("[FAIL] Terrain raycast distance/point mismatch");
                passed = false;
            }
        }

        {
            Terrain flatTerrain(33, 100.0f, 0.0f, 0.035f, 1, 1337);
            Road road;
            const std::vector<Vec3> controlPoints = {
                Vec3(0.0f, 0.0f, -20.0f),
                Vec3(0.0f, 0.0f, 20.0f)
            };
            if (!road.GenerateCourse(flatTerrain, controlPoints, 10.0f, 0.01f)) {
                Logger::Error("[FAIL] Road raycast diagnostic could not generate test road");
                return false;
            }

            PhysicsWorld world;
            Collider* collider = world.CreateCollider();
            collider->SetShape(ColliderShape::Road);
            collider->SetRoad(&road);

            RaycastResult result;
            const Ray ray{ Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f) };
            const bool hit = world.Raycast(ray, result, 3.0f);
            const float error = hit ? std::abs(result.distance - 0.99f) : INFINITY;
            Logger::Info("[RaycastSurfaceTest] shape=Road hit=" +
                std::to_string(hit) + " distance=" +
                (hit ? std::to_string(result.distance) : std::string("none")) +
                " expected=0.99000 error=" + std::to_string(error));
            if (!hit || error > 0.02f || std::abs(result.point.y - 0.01f) > 0.02f) {
                Logger::Error("[FAIL] Road raycast distance/point mismatch");
                passed = false;
            }
        }

        if (passed)
            Logger::Info("[PASS] Raycast surface diagnostics");
        else
            Logger::Error("[FAIL] Raycast surface diagnostics");
        return passed;
    }

    bool RunTireModelDiagnostics() {
        Tire tire;
        const float normalLoad = 4000.0f;
        const float maxForce = tire.GetDynamicFriction() * normalLoad;

        TireState state{
            true,
            normalLoad,
            10.0f,
            2.0f,
            12.0f,
            -2.0f,
            0.2f,
            0.15f,
            Vec3(0.0f, 0.0f, 1.0f),
            Vec3(1.0f, 0.0f, 0.0f),
            Vec3(0.0f, 0.0f, 0.0f)
        };

        const Vec3 force =
            tire.CalculateForce(state);

        const float forceMagnitude =
            force.Length();

        if (!std::isfinite(forceMagnitude) ||
            forceMagnitude > maxForce + 0.001f) {
            Logger::Error(
                "[FAIL] TireModel friction limit exceeded"
            );
            return false;
        }

        if (tire.GetLongitudinalForce() <= 0.0f ||
            tire.GetLateralForce() >= 0.0f) {
            Logger::Error(
                "[FAIL] TireModel force directions are incorrect"
            );
            return false;
        }

        state.slipRatio = 0.0f;
        state.slipAngle = 0.0f;
        state.longitudinalVelocity = 10.0f;
        state.lateralVelocity = 0.0f;
        state.wheelSurfaceSpeed = 10.0f;

        const Vec3 rollingForce =
            tire.CalculateForce(state);

        const float expectedRollingForce =
            -tire.GetRollingResistance() *
            normalLoad;

        if (std::abs(
                rollingForce.Dot(state.forward) -
                expectedRollingForce
            ) > 0.001f) {
            Logger::Error(
                "[FAIL] TireModel rolling resistance is incorrect"
            );
            return false;
        }

        Logger::Info("[PASS] Tire model diagnostics");
        return true;
    }

    bool RunTireGripCurveDiagnostics() {
        Tire tire;
        const float normalLoad = 4000.0f;
        const float rollingResistance =
            tire.GetRollingResistance() *
            normalLoad;

        float peakLongitudinalForce = 0.0f;
        float peakLongitudinalSlip = 0.0f;
        float previousLongitudinalForce = 0.0f;
        bool longitudinalFellAfterPeak = false;

        for (int i = 0; i <= 100; ++i) {
            const float slip = i * 0.01f;

            TireState state{
                true,
                normalLoad,
                20.0f,
                0.0f,
                20.0f * (1.0f + slip),
                20.0f - 20.0f * (1.0f + slip),
                slip,
                0.0f,
                Vec3(0.0f, 0.0f, 1.0f),
                Vec3(1.0f, 0.0f, 0.0f),
                Vec3(0.0f, 0.0f, 0.0f)
            };

            const Vec3 tireForce =
                tire.CalculateForce(state);

            const float longitudinalForce =
                tireForce.Dot(state.forward) +
                rollingResistance;

            if (!std::isfinite(longitudinalForce)) {
                Logger::Error(
                    "[FAIL] TireGripCurve longitudinal force is not finite"
                );
                return false;
            }

            if (longitudinalForce > peakLongitudinalForce) {
                peakLongitudinalForce = longitudinalForce;
                peakLongitudinalSlip = slip;
            }

            if (slip > 0.0f &&
                slip <= 0.12f &&
                longitudinalForce + 0.5f < previousLongitudinalForce) {
                Logger::Error(
                    "[FAIL] TireGripCurve longitudinal rise is not monotonic"
                );
                return false;
            }

            if (slip >= 0.20f &&
                longitudinalForce < peakLongitudinalForce * 0.98f)
                longitudinalFellAfterPeak = true;

            previousLongitudinalForce = longitudinalForce;
        }

        if (peakLongitudinalSlip < 0.08f ||
            peakLongitudinalSlip > 0.16f ||
            !longitudinalFellAfterPeak) {
            Logger::Error(
                "[FAIL] TireGripCurve longitudinal peak/falloff is incorrect"
            );
            return false;
        }

        float peakLateralForce = 0.0f;
        float peakLateralAngle = 0.0f;
        float previousLateralForce = 0.0f;
        bool lateralFellAfterPeak = false;

        for (int i = 0; i <= 80; ++i) {
            const float angle =
                i * (0.70f / 80.0f);

            TireState state{
                true,
                normalLoad,
                20.0f,
                std::tan(angle) * 20.0f,
                20.0f,
                20.0f,
                0.0f,
                angle,
                Vec3(0.0f, 0.0f, 1.0f),
                Vec3(1.0f, 0.0f, 0.0f),
                Vec3(0.0f, 0.0f, 0.0f)
            };

            const Vec3 tireForce =
                tire.CalculateForce(state);

            const float lateralForce =
                -tireForce.Dot(state.lateral);

            if (!std::isfinite(lateralForce)) {
                Logger::Error(
                    "[FAIL] TireGripCurve lateral force is not finite"
                );
                return false;
            }

            if (lateralForce > peakLateralForce) {
                peakLateralForce = lateralForce;
                peakLateralAngle = angle;
            }

            if (angle > 0.0f &&
                angle <= 0.105f &&
                lateralForce + 0.5f < previousLateralForce) {
                Logger::Error(
                    "[FAIL] TireGripCurve lateral rise is not monotonic"
                );
                return false;
            }

            if (angle >= 0.20f &&
                lateralForce < peakLateralForce * 0.98f)
                lateralFellAfterPeak = true;

            previousLateralForce = lateralForce;
        }

        if (peakLateralAngle < 0.08f ||
            peakLateralAngle > 0.14f ||
            !lateralFellAfterPeak) {
            Logger::Error(
                "[FAIL] TireGripCurve lateral peak/falloff is incorrect"
            );
            return false;
        }

        std::ostringstream log;
        log << std::fixed << std::setprecision(3)
            << "[TireGripCurve] longitudinalPeakSlip="
            << peakLongitudinalSlip
            << " lateralPeakAngle="
            << peakLateralAngle
            << " longitudinalPeakForce="
            << peakLongitudinalForce
            << " lateralPeakForce="
            << peakLateralForce;

        Logger::Info(log.str());
        Logger::Info("[PASS] Tire grip curve diagnostics");

        return true;
    }

    bool RunScenario(
        const char* name,
        VehicleTestInput (*inputFunction)(float),
        int (*gearFunction)(float)
    ) {
        VehicleTestRig rig;
        if (!rig.configLoaded) {
            Logger::Error("[FAIL] Vehicle config: " + rig.configError);
            return false;
        }
        ShiftToGear(rig.car, gearFunction(0.0f));

        float maxSpeed = 0.0f;
        float maxAngularSpeed = 0.0f;
        float minUpDot = 1.0f;
        float maxSlipRatio = 0.0f;
        float maxSlipAngle = 0.0f;
        int previousGear = rig.car.GetTransmission().GetGear();

        Logger::Info(
            std::string("[VehicleDiagnostics] begin ") + name
        );

        const int steps =
            static_cast<int>(TestDuration / FixedDeltaTime);

        for (int step = 0; step < steps; ++step) {
            const float time = step * FixedDeltaTime;
            const int targetGear = gearFunction(time);

            if (targetGear != previousGear) {
                ShiftToGear(rig.car, targetGear);
                previousGear = targetGear;
            }

            const VehicleTestInput input = inputFunction(time);

            rig.car.SetInput(
                input.throttle,
                input.brake,
                input.steering,
                input.clutch
            );
            rig.car.UpdatePhysics(
                rig.physicsWorld,
                FixedDeltaTime
            );
            rig.physicsWorld.Step(FixedDeltaTime);

            if (!IsFinite(rig.chassis->GetPosition()) ||
                !IsFinite(rig.chassis->GetLinearVelocity()) ||
                !IsFinite(rig.chassis->GetAngularVelocity()) ||
                !std::isfinite(rig.car.GetEngine().GetRPM())) {
                Logger::Error(
                    std::string("[FAIL] ") + name +
                    " non-finite state at t=" +
                    std::to_string(time)
                );
                return false;
            }

            const Vec3 velocity =
                rig.chassis->GetLinearVelocity();

            const float speed =
                std::sqrt(
                    velocity.x * velocity.x +
                    velocity.z * velocity.z
                );

            const float angularSpeed =
                rig.chassis->GetAngularVelocity().Length();

            maxSpeed = std::max(maxSpeed, speed);
            maxAngularSpeed =
                std::max(maxAngularSpeed, angularSpeed);
            minUpDot =
                std::min(minUpDot, GetUpDot(*rig.chassis));

            for (size_t i = 0; i < WheelCount; ++i) {
                const Tire& tire =
                    rig.car.GetTire(
                        static_cast<WheelIndex>(i)
                    );

                maxSlipRatio =
                    std::max(
                        maxSlipRatio,
                        std::abs(tire.GetSlipRatio())
                    );
                maxSlipAngle =
                    std::max(
                        maxSlipAngle,
                        std::abs(tire.GetSlipAngle())
                    );
            }

            if (step % 60 == 0) {
                std::ostringstream log;
                log << std::fixed << std::setprecision(3)
                    << "[VehicleState] test=" << name
                    << " t=" << time
                    << " gear=" << rig.car.GetTransmission().GetGear()
                    << " throttle=" << input.throttle
                    << " brake=" << input.brake
                    << " steer=" << input.steering
                    << " clutch=" << input.clutch
                    << " speed=" << speed
                    << " rpm=" << rig.car.GetEngine().GetRPM()
                    << " upY=" << GetUpDot(*rig.chassis)
                    << " yawRate="
                    << rig.chassis->GetAngularVelocity().y;

                Logger::Info(log.str());
            }
        }

        std::ostringstream summary;
        summary << std::fixed << std::setprecision(3)
            << "[VehicleSummary] test=" << name
            << " finalSpeedKmh=" << rig.car.GetSpeedKmh()
            << " maxSpeed=" << maxSpeed
            << " maxAngularSpeed=" << maxAngularSpeed
            << " minUpY=" << minUpDot
            << " maxSlipRatio=" << maxSlipRatio
            << " maxSlipAngle=" << maxSlipAngle;

        Logger::Info(summary.str());

        if (maxSlipRatio > 2.001f) {
            Logger::Error(
                std::string("[FAIL] ") + name +
                " low-speed slip ratio exceeded bound"
            );
            return false;
        }

        if (minUpDot < 0.25f) {
            Logger::Error(
                std::string("[FAIL] ") + name +
                " vehicle approached rollover"
            );
            return false;
        }

        Logger::Info(std::string("[PASS] ") + name);
        return true;
    }

    struct CorneringMetrics {
        float meanLateralAcceleration = 0.0f;
        float meanLoadDifference = 0.0f;
        float meanLoadAccelerationProduct = 0.0f;
        float meanAbsoluteLateralAcceleration = 0.0f;
        float meanLeftMinusRightRayDistance = 0.0f;
        int raySamples = 0;
        int samples = 0;
    };

    bool RunVehicleCoordinateConventionDiagnostics() {
        const Vec3 forward = VehicleCoordinates::Forward();
        const Vec3 right = VehicleCoordinates::Right();
        const Vec3 up = VehicleCoordinates::Up();

        const bool handedness =
            (forward.Cross(up) - right).Length() < 0.000001f;

        VehicleConfig fallbackConfig;
        const bool fallbackPositions =
            fallbackConfig.wheelPositions[0].x > 0.0f &&
            fallbackConfig.wheelPositions[1].x < 0.0f &&
            fallbackConfig.wheelPositions[2].x > 0.0f &&
            fallbackConfig.wheelPositions[3].x < 0.0f;

        VehicleTestRig rig;
        if (!rig.configLoaded) {
            Logger::Error("[FAIL] Vehicle coordinate convention: " + rig.configError);
            return false;
        }

        bool loadedPositions = true;
        for (size_t i = 0; i < WheelCount; ++i) {
            const Vec3 position =
                rig.car.GetWheel(static_cast<WheelIndex>(i)).GetLocalPosition();
            const float rightProjection =
                position.Dot(right);

            const bool expectedRightWheel =
                i == static_cast<size_t>(WheelIndex::FrontRight) ||
                i == static_cast<size_t>(WheelIndex::RearRight);

            loadedPositions =
                (expectedRightWheel
                    ? rightProjection > 0.0f
                    : rightProjection < 0.0f) &&
                loadedPositions;
        }

        std::ostringstream log;
        log << std::fixed << std::setprecision(3)
            << "[VehicleCoordinates] forward=("
            << forward.x << "," << forward.y << "," << forward.z
            << ") right=("
            << right.x << "," << right.y << "," << right.z
            << ") up=("
            << up.x << "," << up.y << "," << up.z
            << ") handedness=" << handedness
            << " fallbackPositions=" << fallbackPositions
            << " loadedPositions=" << loadedPositions;
        Logger::Info(log.str());

        const bool passed =
            handedness &&
            fallbackPositions &&
            loadedPositions;

        if (passed)
            Logger::Info("[PASS] Vehicle coordinate convention diagnostics");
        else
            Logger::Error("[FAIL] Vehicle coordinate convention diagnostics");

        return passed;
    }

    bool RunSteeringForceDirectionTest(const char* name, float steering, float expectedYawSign) {
        VehicleTestRig rig;
        if (!rig.configLoaded) {
            Logger::Error("[FAIL] Vehicle config: " + rig.configError);
            return false;
        }

        for (int step = 0; step < 240; ++step) {
            rig.car.SetInput(0.0f, 0.0f, 0.0f, 1.0f);
            rig.car.UpdatePhysics(rig.physicsWorld, FixedDeltaTime);
            rig.physicsWorld.Step(FixedDeltaTime);
        }

        rig.chassis->SetLinearVelocity(Vec3(0.0f, 0.0f, 8.0f));
        rig.car.SetInput(0.0f, 0.0f, steering, 1.0f);
        rig.car.UpdatePhysics(rig.physicsWorld, FixedDeltaTime);

        const float yawTorque = rig.chassis->GetTorque().y;
        std::ostringstream log;
        log << std::fixed << std::setprecision(3)
            << "[SteeringForce] test=" << name
            << " steeringInput=" << steering
            << " frontWheelAngle=" << rig.car.GetWheel(WheelIndex::FrontLeft).GetSteeringAngle()
            << " yawTorque=" << yawTorque;
        Logger::Info(log.str());

        if (!std::isfinite(yawTorque) ||
            yawTorque * expectedYawSign <= 0.0f) {
            Logger::Error(std::string("[FAIL] ") + name +
                " produced yaw torque in the wrong direction");
            return false;
        }

        Logger::Info(std::string("[PASS] ") + name + " steering force direction");
        return true;
    }

    bool RunCorneringDirectionTest(
        const char* name,
        float steering,
        CorneringMetrics& metrics
    ) {
        VehicleTestRig rig;
        if (!rig.configLoaded) {
            Logger::Error("[FAIL] Vehicle config: " + rig.configError);
            return false;
        }

        ShiftToGear(rig.car, 1);
        Logger::Info(
            std::string("[CorneringDiagnostics] begin ") + name
        );

        float lateralAccelerationSum = 0.0f;
        float loadDifferenceSum = 0.0f;
        float loadAccelerationProductSum = 0.0f;
        float absoluteLateralAccelerationSum = 0.0f;
        const int steps =
            static_cast<int>(TestDuration / FixedDeltaTime);
        Vec3 previousVelocity = rig.chassis->GetLinearVelocity();
        bool hasPreviousVelocity = false;

        for (int step = 0; step < steps; ++step) {
            const float time = step * FixedDeltaTime;
            float clutch = 0.0f;
            if (time < 0.75f) {
                clutch = 1.0f;
            } else if (time < 2.75f) {
                clutch = std::clamp(
                    1.0f - (time - 0.75f) / 2.0f,
                    0.0f,
                    1.0f
                );
            }

            const float throttle = time < 2.75f ? 0.30f : 0.12f;
            const float steeringInput = time >= 3.0f ? steering : 0.0f;

            rig.car.SetInput(
                throttle,
                0.0f,
                steeringInput,
                clutch
            );
            rig.car.UpdatePhysics(
                rig.physicsWorld,
                FixedDeltaTime
            );
            rig.physicsWorld.Step(FixedDeltaTime);

            if (!IsFinite(rig.chassis->GetPosition()) ||
                !IsFinite(rig.chassis->GetLinearVelocity()) ||
                !IsFinite(rig.chassis->GetAngularVelocity())) {
                Logger::Error(
                    std::string("[FAIL] ") + name +
                    " non-finite chassis state at t=" +
                    std::to_string(time)
                );
                return false;
            }

            const Vec3 velocity =
                rig.chassis->GetLinearVelocity();
            Vec3 worldAcceleration(0.0f, 0.0f, 0.0f);
            if (hasPreviousVelocity) {
                worldAcceleration =
                    (velocity - previousVelocity) / FixedDeltaTime;
            }
            previousVelocity = velocity;
            hasPreviousVelocity = true;

            if (time < 4.0f || time > 8.5f)
                continue;

            const Vec3 vehicleRight =
                rig.chassis->GetOrientation() *
                VehicleCoordinates::Right();
            const float lateralAcceleration =
                worldAcceleration.Dot(vehicleRight);

            const float leftLoad =
                rig.car.GetTire(WheelIndex::FrontLeft).GetNormalLoad() +
                rig.car.GetTire(WheelIndex::RearLeft).GetNormalLoad();
            const float rightLoad =
                rig.car.GetTire(WheelIndex::FrontRight).GetNormalLoad() +
                rig.car.GetTire(WheelIndex::RearRight).GetNormalLoad();
            const float loadDifference = leftLoad - rightLoad;

            const Wheel& frontLeftWheel =
                rig.car.GetWheel(WheelIndex::FrontLeft);
            const Wheel& frontRightWheel =
                rig.car.GetWheel(WheelIndex::FrontRight);
            const Wheel& rearLeftWheel =
                rig.car.GetWheel(WheelIndex::RearLeft);
            const Wheel& rearRightWheel =
                rig.car.GetWheel(WheelIndex::RearRight);
            if (frontLeftWheel.IsGrounded() &&
                frontRightWheel.IsGrounded() &&
                rearLeftWheel.IsGrounded() &&
                rearRightWheel.IsGrounded()) {
                const float leftRayDistance =
                    0.5f * (
                        frontLeftWheel.GetLastRayDistance() +
                        rearLeftWheel.GetLastRayDistance()
                    );
                const float rightRayDistance =
                    0.5f * (
                        frontRightWheel.GetLastRayDistance() +
                        rearRightWheel.GetLastRayDistance()
                    );
                metrics.meanLeftMinusRightRayDistance +=
                    leftRayDistance - rightRayDistance;
                ++metrics.raySamples;
            }

            if (!std::isfinite(lateralAcceleration) ||
                !std::isfinite(loadDifference) ||
                leftLoad < 0.0f ||
                rightLoad < 0.0f) {
                Logger::Error(
                    std::string("[FAIL] ") + name +
                    " invalid lateral acceleration or wheel load"
                );
                return false;
            }

            // Coordinate convention: +Z is forward, -X is right.
            // Positive steering input means right turn, loading the left wheels.
            // Negative steering input means left turn, loading the right wheels.
            // Therefore lateral acceleration and left-minus-right load must
            // have the same sign as the steering input.
            lateralAccelerationSum += lateralAcceleration;
            loadDifferenceSum += loadDifference;
            loadAccelerationProductSum +=
                loadDifference * lateralAcceleration;
            absoluteLateralAccelerationSum +=
                std::abs(lateralAcceleration);
            ++metrics.samples;
        }

        if (metrics.samples == 0) {
            Logger::Error(
                std::string("[FAIL] ") + name +
                " collected no cornering samples"
            );
            return false;
        }

        metrics.meanLateralAcceleration =
            lateralAccelerationSum / metrics.samples;
        metrics.meanLoadDifference =
            loadDifferenceSum / metrics.samples;
        metrics.meanLoadAccelerationProduct =
            loadAccelerationProductSum / metrics.samples;
        metrics.meanAbsoluteLateralAcceleration =
            absoluteLateralAccelerationSum / metrics.samples;
        if (metrics.raySamples > 0)
            metrics.meanLeftMinusRightRayDistance /=
                static_cast<float>(metrics.raySamples);

        std::ostringstream log;
        log << std::fixed << std::setprecision(3)
            << "[CorneringSummary] test=" << name
            << " samples=" << metrics.samples
            << " meanLateralAcceleration="
            << metrics.meanLateralAcceleration
            << " meanAbsLateralAcceleration="
            << metrics.meanAbsoluteLateralAcceleration
            << " meanLeftMinusRightLoad="
            << metrics.meanLoadDifference
            << " meanLeftMinusRightRayDistance="
            << metrics.meanLeftMinusRightRayDistance
            << " raySamples=" << metrics.raySamples
            << " meanLoadAccelerationProduct="
            << metrics.meanLoadAccelerationProduct;
        Logger::Info(log.str());

        if (metrics.meanAbsoluteLateralAcceleration < 0.10f) {
            Logger::Error(
                std::string("[FAIL] ") + name +
                " steering produced insufficient lateral acceleration"
            );
            return false;
        }

        if (metrics.raySamples == 0 ||
            metrics.meanLeftMinusRightRayDistance * steering >= 0.0f) {
            Logger::Error(
                std::string("[FAIL] ") + name +
                " suspension ray distances do not match outside-wheel geometry"
            );
            return false;
        }

        if (metrics.meanLoadAccelerationProduct <= 0.0f ||
            metrics.meanLateralAcceleration *
                metrics.meanLoadDifference <= 0.0f ||
            metrics.meanLateralAcceleration * steering <= 0.0f ||
            metrics.meanLoadDifference * steering <= 0.0f) {
            Logger::Error(
                std::string("[FAIL] ") + name +
                " outside-wheel load transfer has the wrong direction"
            );
            return false;
        }

        Logger::Info(
            std::string("[PASS] ") + name +
            " outside-wheel load transfer direction"
        );
        return true;
    }

    bool RunCorneringLoadTransferDiagnostics() {
        CorneringMetrics positiveSteer;
        CorneringMetrics negativeSteer;

        bool passed = RunCorneringDirectionTest(
            "PositiveSteer",
            0.35f,
            positiveSteer
        );
        passed = RunCorneringDirectionTest(
            "NegativeSteer",
            -0.35f,
            negativeSteer
        ) && passed;

        if (positiveSteer.samples == 0 ||
            negativeSteer.samples == 0)
            return false;

        if (positiveSteer.meanLateralAcceleration *
                negativeSteer.meanLateralAcceleration >= 0.0f ||
            positiveSteer.meanLoadDifference *
                negativeSteer.meanLoadDifference >= 0.0f) {
            Logger::Error(
                "[FAIL] Cornering load transfer did not mirror between left and right turns"
            );
            passed = false;
        }

        if (passed)
            Logger::Info("[PASS] Cornering load transfer diagnostics");
        else
            Logger::Error("[FAIL] Cornering load transfer diagnostics");

        return passed;
    }
}

int main() {
    Logger::Initialize("Logs/vehicle_diagnostics.log");

    bool passed = RunVehicleCoordinateConventionDiagnostics();
    passed = RunRaycastRollGeometryDiagnostics() && passed;
    passed = RunRaycastSurfaceDiagnostics() && passed;
    passed = RunTireModelDiagnostics() && passed;

    passed = RunTireGripCurveDiagnostics() && passed;

    passed = RunScenario(
        "Neutral",
        NeutralInput,
        [](float) { return 0; }
    ) && passed;

    passed = RunScenario(
        "LaunchAndShift",
        LaunchInput,
        LaunchGear
    ) && passed;

    passed = RunScenario(
        "LaunchAndBrake",
        BrakeInput,
        BrakeGear
    ) && passed;

    passed = RunScenario(
        "GentleSlalom",
        GentleSlalomInput,
        SlalomGear
    ) && passed;

    passed = RunSteeringForceDirectionTest("RightSteer", 0.35f, 1.0f) && passed;
    passed = RunSteeringForceDirectionTest("LeftSteer", -0.35f, -1.0f) && passed;

    passed = RunCorneringLoadTransferDiagnostics() && passed;

    if (passed)
        Logger::Info("[PASS] All vehicle diagnostics passed");
    else
        Logger::Error("[FAIL] Vehicle diagnostics failed");

    Logger::Shutdown();
    return passed ? 0 : 1;
}
