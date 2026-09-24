#include "Tire.h"

#include "Wheel.h"
#include "../Physics/RigidBody.h"

#include <algorithm>
#include <cmath>

Tire::Tire()
    : m_staticFriction(1.10f),
    m_dynamicFriction(0.95f),
    m_longitudinalStiffness(9000.0f),
    m_lateralStiffness(11000.0f),
    m_rollingResistance(0.015f) {}

void Tire::SetStaticFriction(float friction) {
    m_staticFriction =
        std::max(0.0f, friction);
}

void Tire::SetDynamicFriction(float friction) {
    m_dynamicFriction =
        std::max(0.0f, friction);
}

void Tire::SetLongitudinalStiffness(float stiffness) {
    m_longitudinalStiffness =
        std::max(0.0f, stiffness);
}

void Tire::SetLateralStiffness(float stiffness) {
    m_lateralStiffness =
        std::max(0.0f, stiffness);
}

void Tire::SetRollingResistance(float coefficient) {
    m_rollingResistance =
        std::max(0.0f, coefficient);
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

float Tire::GetRollingResistance() const {
    return m_rollingResistance;
}

Vec3 Tire::CalculateForce(
    const RigidBody& body,
    const Wheel& wheel,
    float deltaTime
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

    Vec3 forward =
        body.GetOrientation() *
        Vec3(0.0f, 0.0f, 1.0f);

    forward -=
        normal * forward.Dot(normal);

    if (forward.LengthSquared() <=
        0.000001f) {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    forward = forward.Normalized();

    Vec3 lateral =
        normal.Cross(forward);

    if (lateral.LengthSquared() <=
        0.000001f) {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    lateral = lateral.Normalized();

    const float longitudinalVelocity =
        contactVelocity.Dot(forward);

    const float lateralVelocity =
        contactVelocity.Dot(lateral);

    const float wheelSurfaceSpeed =
        wheel.GetAngularVelocity() *
        wheel.GetRadius();

    const float longitudinalSlipVelocity =
        longitudinalVelocity -
        wheelSurfaceSpeed;

    const float inverseInertiaContribution =
        wheel.GetRadius() *
        wheel.GetRadius() /
        wheel.GetInertia();

    const Vec3 bodyRadius =
        contactPoint -
        body.GetPosition();

    const Mat3 inverseInertia =
        body.GetWorldInverseInertiaTensor();

    const Vec3 longitudinalTorqueAxis =
        bodyRadius.Cross(forward);

    const float longitudinalBodyInverseMass =
        body.GetInverseMass() +
        longitudinalTorqueAxis.Dot(
            inverseInertia *
            longitudinalTorqueAxis
        );

    const float longitudinalInverseMass =
        longitudinalBodyInverseMass +
        inverseInertiaContribution;

    float longitudinalStiffness =
        m_longitudinalStiffness;

    if (deltaTime > 0.0f &&
        longitudinalInverseMass > 0.0f) {
        longitudinalStiffness =
            m_longitudinalStiffness /
            (
                1.0f +
                m_longitudinalStiffness *
                longitudinalInverseMass *
                deltaTime
            );
    }

    float lateralStiffness =
        m_lateralStiffness;

    const Vec3 lateralTorqueAxis =
        bodyRadius.Cross(lateral);

    const float lateralInverseMass =
        body.GetInverseMass() +
        lateralTorqueAxis.Dot(
            inverseInertia *
            lateralTorqueAxis
        );

    if (deltaTime > 0.0f &&
        lateralInverseMass > 0.0f) {
        lateralStiffness =
            m_lateralStiffness /
            (
                1.0f +
                m_lateralStiffness *
                lateralInverseMass *
                deltaTime
            );
    }

    Vec3 desiredForce =
        forward *
            (-longitudinalSlipVelocity *
             longitudinalStiffness) +
        lateral *
            (-lateralVelocity *
             lateralStiffness);

    const float contactSpeedSq =
        longitudinalVelocity *
            longitudinalVelocity +
        lateralVelocity *
            lateralVelocity;

    if (contactSpeedSq > 0.000001f) {
        const float contactSpeed =
            std::sqrt(contactSpeedSq);

        desiredForce +=
            forward *
            (-longitudinalVelocity /
             contactSpeed *
             m_rollingResistance *
             normalLoad);
    }

    const float desiredMagnitude =
        desiredForce.Length();

    if (desiredMagnitude <= 0.000001f)
        return Vec3(0.0f, 0.0f, 0.0f);

    const float slipSpeed =
        std::max(
            std::abs(longitudinalSlipVelocity),
            std::abs(lateralVelocity)
        );

    const float friction =
        slipSpeed < 0.5f
        ? m_staticFriction
        : m_dynamicFriction;

    const float maxForce =
        friction * normalLoad;

    if (desiredMagnitude <= maxForce)
        return desiredForce;

    return desiredForce *
        (maxForce / desiredMagnitude);
}
