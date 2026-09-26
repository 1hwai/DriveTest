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

namespace {
    constexpr float FixedDeltaTime = 1.0f / 120.0f;
    constexpr int SimulationSteps = 120 * 40;

    void ConfigureWheel(
        Car& car,
        WheelIndex index,
        const Vec3& position
    ) {
        car.GetWheel(index).SetLocalPosition(position);
        car.GetWheel(index).SetRadius(0.5f);

        car.GetSuspension(index).SetRestLength(0.8f);
        car.GetSuspension(index).SetMaxLength(1.0f);
        car.GetSuspension(index).SetSpringRate(30000.0f);
        car.GetSuspension(index).SetDamperRate(4500.0f);
    }

    bool IsFinite(const Vec3& value) {
        return std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }
}

int main() {
    Logger::Initialize("Logs/physics_diagnostics.log");

    PhysicsWorld physicsWorld;

    Terrain terrain(
        65,
        100.0f,
        0.0f,
        0.035f,
        5,
        1337
    );

    terrain.GenerateSineWave(
        0.5f,
        3.14159265f * 0.25f
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
        Vec3(0.0f, 5.0f, 0.0f)
    );

    Collider* chassisCollider =
        physicsWorld.CreateCollider();

    chassisCollider->SetShape(
        ColliderShape::Box
    );
    chassisCollider->SetHalfExtents(
        Vec3(1.0f, 0.5f, 1.5f)
    );
    chassisCollider->SetRigidBody(chassis);

    Car car;
    car.SetChassis(chassis);

    ConfigureWheel(
        car,
        WheelIndex::FrontLeft,
        Vec3(-0.9f, -0.2f, 1.1f)
    );
    ConfigureWheel(
        car,
        WheelIndex::FrontRight,
        Vec3(0.9f, -0.2f, 1.1f)
    );
    ConfigureWheel(
        car,
        WheelIndex::RearLeft,
        Vec3(-0.9f, -0.2f, -1.1f)
    );
    ConfigureWheel(
        car,
        WheelIndex::RearRight,
        Vec3(0.9f, -0.2f, -1.1f)
    );

    float maxAbsZ = 0.0f;
    float minY = chassis->GetPosition().y;
    float maxEnergy = 0.0f;

    Logger::Info("[PhysicsDiagnostics] sine terrain suspension test");

    for (int step = 0;
        step < SimulationSteps;
        ++step) {

        car.UpdatePhysics(
            physicsWorld,
            FixedDeltaTime
        );

        physicsWorld.Step(
            FixedDeltaTime
        );

        const Vec3 position =
            chassis->GetPosition();

        const Vec3 velocity =
            chassis->GetLinearVelocity();

        if (!IsFinite(position) ||
            !IsFinite(velocity) ||
            !std::isfinite(
                chassis->GetAngularVelocity().LengthSquared()
            )) {

            std::cerr
                << "[FAIL] Non-finite chassis state at step "
                << step << '\n';

            Logger::Shutdown();
            return 1;
        }

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
                car.GetWheel(
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
    }

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(6)
        << "[Summary] minY="
        << minY
        << " maxAbsZ="
        << maxAbsZ
        << " maxEnergy="
        << maxEnergy
        << " finalY="
        << chassis->GetPosition().y
        << " finalZ="
        << chassis->GetPosition().z;

    Logger::Info(summary.str());

    Logger::Shutdown();

    return 0;
}
