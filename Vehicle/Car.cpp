#include "Car.h"

#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"

namespace {
    size_t ToIndex(WheelIndex index) {
        return static_cast<size_t>(index);
    }
}

Car::Car()
    : m_chassis(nullptr) {}

void Car::SetChassis(RigidBody* chassis) {
    m_chassis = chassis;
}

void Car::UpdatePhysics(
    PhysicsWorld& physicsWorld,
    float deltaTime
) {
    if (!m_chassis)
        return;

    for (size_t i = 0; i < WheelCount; ++i) {
        m_wheels[i].Update(
            *m_chassis,
            physicsWorld,
            m_suspensions[i],
            deltaTime
        );

        const Vec3 tireForce =
            m_tires[i].CalculateForce(
                *m_chassis,
                m_wheels[i],
                deltaTime
            );

        if (tireForce.LengthSquared() >
            0.000001f) {
            m_chassis->AddForceAtPoint(
                tireForce,
                m_wheels[i].GetContactPoint()
            );

            m_wheels[i].ApplyTireForce(
                *m_chassis,
                tireForce
            );
        }

        m_wheels[i].IntegrateRotation(
            deltaTime
        );
    }
}

Wheel& Car::GetWheel(WheelIndex index) {
    return m_wheels[ToIndex(index)];
}

const Wheel& Car::GetWheel(WheelIndex index) const {
    return m_wheels[ToIndex(index)];
}

Suspension& Car::GetSuspension(WheelIndex index) {
    return m_suspensions[ToIndex(index)];
}

const Suspension& Car::GetSuspension(WheelIndex index) const {
    return m_suspensions[ToIndex(index)];
}

Tire& Car::GetTire(WheelIndex index) {
    return m_tires[ToIndex(index)];
}

const Tire& Car::GetTire(WheelIndex index) const {
    return m_tires[ToIndex(index)];
}

RigidBody* Car::GetChassis() {
    return m_chassis;
}

const RigidBody* Car::GetChassis() const {
    return m_chassis;
}
