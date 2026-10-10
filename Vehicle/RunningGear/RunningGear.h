#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include "DoubleWishbone.h"
#include "IWheelContactProvider.h"
#include "Suspension.h"
#include "Tire.h"
#include "Wheel.h"

class RigidBody;
class PhysicsWorld;
struct VehicleConfig;

enum class WheelIndex {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
    Count
};

constexpr size_t WheelCount =
    static_cast<size_t>(WheelIndex::Count);

class RunningGear {
public:
    RunningGear();
    explicit RunningGear(std::unique_ptr<IWheelContactProvider> contactProvider);

    void SetChassis(RigidBody* chassis);
    RigidBody* GetChassis();
    const RigidBody* GetChassis() const;

    void ApplyConfig(const VehicleConfig& config);
    void UpdatePhysics(
        PhysicsWorld& physicsWorld,
        float steeringInput,
        float brakeInput,
        float leftDriveTorque,
        float rightDriveTorque,
        float deltaTime
    );

    float GetAverageWheelAngularVelocity() const;

    DoubleWishbone& GetSuspensionGeometry(WheelIndex index);
    const DoubleWishbone& GetSuspensionGeometry(WheelIndex index) const;
    Wheel& GetWheel(WheelIndex index);
    const Wheel& GetWheel(WheelIndex index) const;
    Suspension& GetSuspension(WheelIndex index);
    const Suspension& GetSuspension(WheelIndex index) const;
    Tire& GetTire(WheelIndex index);
    const Tire& GetTire(WheelIndex index) const;

private:
    std::unique_ptr<IWheelContactProvider> m_contactProvider;
    RigidBody* m_chassis = nullptr;
    float m_energyTimer = 0.0f;
    float m_brakeTimer = 0.0f;
    float m_tireTimer = 0.0f;
    float m_brakeTorque = 2500.0f;
    float m_maxSteeringAngle = 0.5f;

    std::array<DoubleWishbone, WheelCount> m_suspensionGeometry;
    std::array<float, WheelCount> m_minimumSuspensionTravel{};
    std::array<float, WheelCount> m_maximumSuspensionTravel{};
    std::array<Wheel, WheelCount> m_wheels;
    std::array<Suspension, WheelCount> m_suspensions;
    std::array<Tire, WheelCount> m_tires;
};
