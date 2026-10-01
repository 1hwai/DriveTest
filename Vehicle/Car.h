#pragma once

#include <array>
#include <cstddef>

#include "Wheel.h"
#include "Suspension.h"
#include "Tire.h"
#include "Engine.h"
#include "Transmission.h"
#include "Powertrain.h"
#include "VehicleConfig.h"

class RigidBody;
class PhysicsWorld;

enum class WheelIndex {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
    Count
};

constexpr size_t WheelCount =
    static_cast<size_t>(WheelIndex::Count);

class Car {
public:
    Car();

    void SetChassis(RigidBody* chassis);
    void ApplyConfig(const VehicleConfig& config);

    void SetInput(
        float throttle,
        float brake,
        float steering,
        float clutch
    );

    void UpdatePhysics(
        PhysicsWorld& physicsWorld,
        float deltaTime
    );

    Wheel& GetWheel(WheelIndex index);
    const Wheel& GetWheel(WheelIndex index) const;

    Suspension& GetSuspension(WheelIndex index);
    const Suspension& GetSuspension(WheelIndex index) const;

    Tire& GetTire(WheelIndex index);
    const Tire& GetTire(WheelIndex index) const;

    RigidBody* GetChassis();
    const RigidBody* GetChassis() const;

    Engine& GetEngine();
    const Engine& GetEngine() const;

    Transmission& GetTransmission();
    const Transmission& GetTransmission() const;

    float GetSpeedKmh() const;
    float GetThrottle() const;
    float GetBrake() const;
    float GetSteering() const;
    float GetClutch() const;

private:
    RigidBody* m_chassis;
    float m_throttle;
    float m_brake;
    float m_steering;
    float m_clutch;
    float m_energyTimer;
    float m_brakeTimer;
    float m_tireTimer;
    float m_brakeTorque;
    float m_maxSteeringAngle;

    std::array<Wheel, WheelCount> m_wheels;
    std::array<Suspension, WheelCount> m_suspensions;
    std::array<Tire, WheelCount> m_tires;

    Engine m_engine;
    Transmission m_transmission;
    Powertrain m_powertrain;
};
