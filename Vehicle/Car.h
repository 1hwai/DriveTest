#pragma once

#include <array>
#include <cstddef>

#include "Wheel.h"
#include "Suspension.h"

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

    void UpdatePhysics(
        PhysicsWorld& physicsWorld,
        float deltaTime
    );

    Wheel& GetWheel(WheelIndex index);
    const Wheel& GetWheel(WheelIndex index) const;

    Suspension& GetSuspension(WheelIndex index);
    const Suspension& GetSuspension(WheelIndex index) const;

    RigidBody* GetChassis();
    const RigidBody* GetChassis() const;

private:
    RigidBody* m_chassis;

    std::array<Wheel, WheelCount> m_wheels;
    std::array<Suspension, WheelCount> m_suspensions;
};
