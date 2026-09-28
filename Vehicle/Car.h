#pragma once

#include <array>
#include <cstddef>

#include "Wheel.h"
#include "Suspension.h"
#include "Tire.h"
#include "Engine.h"
#include "Transmission.h"
#include "Differential.h"

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

    void SetInput(
        float throttle,
        float brake,
        float steering
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

private:
    RigidBody* m_chassis;
    float m_throttle;
    float m_brake;
    float m_steering;
    float m_energyTimer;
    float m_brakeTimer;
    float m_tireTimer;

    std::array<Wheel, WheelCount> m_wheels;
    std::array<Suspension, WheelCount> m_suspensions;
    std::array<Tire, WheelCount> m_tires;

    Engine m_engine;
    Transmission m_transmission;
    Differential m_differential;
};
