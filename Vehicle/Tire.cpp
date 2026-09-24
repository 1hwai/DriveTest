#include "Tire.h"

#include "Wheel.h"
#include "../Physics/RigidBody.h"

#include <cmath>

Tire::Tire()
    : m_staticFriction(1.10f),
    m_dynamicFriction(0.95f),
    m_longitudinalStiffness(9000.0f),
    m_lateralStiffness(11000.0f) {}

void Tire::SetStaticFriction(float friction) {
    m_staticFriction = friction > 0.0f ? friction : 0.0f;
}

void Tire::SetDynamicFriction(float friction) {
    m_dynamicFriction = friction > 0.0f ? friction : 0.0f;
}

void Tire::SetLongitudinalStiffness(float stiffness) {
    m_longitudinalStiffness =
        stiffness > 0.0f ? stiffness : 0.0f;
}

void Tire::SetLateralStiffness(float stiffness) {
    m_lateralStiffness =
        stiffness > 0.0f ? stiffness : 0.0f;
}

float Tire::GetStaticFriction() const {
    return m_staticFriction;
}

float Tire::GetDynamicFriction() const {
    return m_dynamicFriction;
}

float Tire::GetLongitudinalStiffness() const {
    return m_longitudinalStiffness;
}

float Tire::GetLateralStiffness() const {
    return m_lateralStiffness;
}

Vec3 Tire::CalculateForce(
    const RigidBody& body,
    const Wheel& wheel
) const {
    if (!wheel.IsGrounded())
        return Vec3(0.0f, 0.0f, 0.0f);

    const float normalLoad =
        wheel.GetForce();

    if (normalLoad <= 0.0f)
        return Vec3(0.0f, 0.0f, 0.0f);

    const Vec3 contactPoint =
        wheel.GetContactPoint();

    const Vec3 contactVelocity =
        body.GetPointVelocity(contactPoint);

    const Vec3 normal =
        wheel.GetContactNormal();

    Vec3 tangentVelocity =
        contactVelocity -
        normal * contactVelocity.Dot(normal);

    const float tangentSpeedSq =
        tangentVelocity.LengthSquared();

    if (tangentSpeedSq <= 0.00000001f)
        return Vec3(0.0f, 0.0f, 0.0f);

    Vec3 forward =
        body.GetOrientation() *
        Vec3(0.0f, 0.0f, 1.0f);

    forward -=
        normal * forward.Dot(normal);

    if (forward.LengthSquared() <= 0.000001f)
        return Vec3(0.0f, 0.0f, 0.0f);

    forward = forward.Normalized();

    Vec3 lateral =
        normal.Cross(forward);

    if (lateral.LengthSquared() <= 0.000001f)
        return Vec3(0.0f, 0.0f, 0.0f);

    lateral = lateral.Normalized();

    const float longitudinalVelocity =
        tangentVelocity.Dot(forward);

    const float lateralVelocity =
        tangentVelocity.Dot(lateral);

    Vec3 desiredForce =
        forward *
            (-longitudinalVelocity *
             m_longitudinalStiffness) +
        lateral *
            (-lateralVelocity *
             m_lateralStiffness);

    const float desiredMagnitude =
        desiredForce.Length();

    if (desiredMagnitude <= 0.000001f)
        return Vec3(0.0f, 0.0f, 0.0f);

    const float tangentSpeed =
        std::sqrt(tangentSpeedSq);

    const float friction =
        tangentSpeed < 0.5f
        ? m_staticFriction
        : m_dynamicFriction;

    const float maxForce =
        friction * normalLoad;

    if (desiredMagnitude <= maxForce)
        return desiredForce;

    return desiredForce *
        (maxForce / desiredMagnitude);
}
