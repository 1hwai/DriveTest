#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "../Core/Debug/Logger.h"
#include "../Physics/Collider.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Vehicle/Car.h"

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

        VehicleTestRig()
            : chassis(nullptr) {
            Collider* ground = physicsWorld.CreateCollider();
            ground->SetShape(ColliderShape::Plane);
            ground->SetPlaneHeight(0.0f);

            chassis = physicsWorld.CreateRigidBody();
            chassis->SetMass(1200.0f);
            chassis->SetBoxInertia(Vec3(1.9f, 0.8f, 2.6f));
            chassis->SetPosition(Vec3(0.0f, 0.95f, 0.0f));
            car.SetChassis(chassis);

            ConfigureWheel(WheelIndex::FrontLeft, Vec3(-0.75f, -0.25f, 1.15f));
            ConfigureWheel(WheelIndex::FrontRight, Vec3(0.75f, -0.25f, 1.15f));
            ConfigureWheel(WheelIndex::RearLeft, Vec3(-0.75f, -0.25f, -1.15f));
            ConfigureWheel(WheelIndex::RearRight, Vec3(0.75f, -0.25f, -1.15f));
        }

        void ConfigureWheel(WheelIndex index, const Vec3& position) {
            car.GetWheel(index).SetLocalPosition(position);
            car.GetWheel(index).SetRadius(0.32f);
            car.GetSuspension(index).SetRestLength(0.45f);
            car.GetSuspension(index).SetMaxLength(0.65f);
            car.GetSuspension(index).SetSpringRate(30000.0f);
            car.GetSuspension(index).SetDamperRate(4500.0f);
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

    bool RunScenario(
        const char* name,
        VehicleTestInput (*inputFunction)(float),
        int (*gearFunction)(float)
    ) {
        VehicleTestRig rig;
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
}

int main() {
    Logger::Initialize("Logs/vehicle_diagnostics.log");

    bool passed = true;

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

    if (passed)
        Logger::Info("[PASS] All vehicle diagnostics passed");
    else
        Logger::Error("[FAIL] Vehicle diagnostics failed");

    Logger::Shutdown();
    return passed ? 0 : 1;
}
