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
            chassis->SetPosition(Vec3(0.0f, config.initialHeight, 0.0f));
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
}

int main() {
    Logger::Initialize("Logs/vehicle_diagnostics.log");

    bool passed = RunTireModelDiagnostics();

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

    if (passed)
        Logger::Info("[PASS] All vehicle diagnostics passed");
    else
        Logger::Error("[FAIL] Vehicle diagnostics failed");

    Logger::Shutdown();
    return passed ? 0 : 1;
}
