#include "Car.h"

#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"

#include <cmath>

namespace {
    size_t ToIndex(WheelIndex index) {
        return static_cast<size_t>(index);
    }
}

Car::Car()
    : m_chassis(nullptr),
    m_planarX(0.0f) {}

void Car::SetChassis(RigidBody* chassis) {
    m_chassis = chassis;

    if (m_chassis) {
        m_planarX =
            m_chassis->GetPosition().x;
    }
}

void Car::UpdatePhysics(
    PhysicsWorld& physicsWorld,
    float deltaTime
) {
    if (!m_chassis)
        return;

    {
        Vec3 position =
            m_chassis->GetPosition();

        position.x = m_planarX;
        m_chassis->SetPosition(position);

        Vec3 velocity =
            m_chassis->GetLinearVelocity();

        velocity.x = 0.0f;
        m_chassis->SetLinearVelocity(velocity);

        Vec3 angularVelocity =
            m_chassis->GetAngularVelocity();

        angularVelocity.y = 0.0f;
        angularVelocity.z = 0.0f;
        m_chassis->SetAngularVelocity(angularVelocity);

        Vec3 forward =
            m_chassis->GetOrientation() *
            Vec3(0.0f, 0.0f, 1.0f);

        const float pitch =
            std::atan2(
                -forward.y,
                forward.z
            );

        m_chassis->SetOrientation(
            Quaternion::FromAxisAngle(
                Vec3(1.0f, 0.0f, 0.0f),
                pitch
            )
        );
    }

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
