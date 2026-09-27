#include "Tire.h"

#include "Wheel.h"
#include "../Physics/RigidBody.h"
#include "../Core/Math/Quaternion.h"

#include <algorithm>
#include <cmath>

Tire::Tire()
    : m_staticFriction(1.10f),
    m_dynamicFriction(0.95f),
    m_longitudinalStiffness(9000.0f),
    m_lateralStiffness(11000.0f),
    m_rollingResistance(0.015f),
    m_longitudinalVelocity(0.0f),
    m_lateralVelocity(0.0f),
    m_wheelSurfaceSpeed(0.0f),
    m_longitudinalSlipVelocity(0.0f),
    m_slipRatio(0.0f),
    m_slipAngle(0.0f),
    m_normalLoad(0.0f),
    m_longitudinalForce(0.0f) {}

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

float Tire::GetLongitudinalVelocity() const {
    return m_longitudinalVelocity;
}

float Tire::GetLateralVelocity() const {
    return m_lateralVelocity;
}

float Tire::GetWheelSurfaceSpeed() const {
    return m_wheelSurfaceSpeed;
}

float Tire::GetLongitudinalSlipVelocity() const {
    return m_longitudinalSlipVelocity;
}

float Tire::GetSlipRatio() const {
    return m_slipRatio;
}

float Tire::GetSlipAngle() const {
    return m_slipAngle;
}

float Tire::GetNormalLoad() const {
    return m_normalLoad;
}

float Tire::GetLongitudinalForce() const {
    return m_longitudinalForce;
}

Vec3 Tire::CalculateForce(
    const RigidBody& body,
    const Wheel& wheel,
    float deltaTime
) const {
    m_longitudinalVelocity = 0.0f;
    m_lateralVelocity = 0.0f;
    m_wheelSurfaceSpeed = 0.0f;
    m_longitudinalSlipVelocity = 0.0f;
    m_slipRatio = 0.0f;
    m_slipAngle = 0.0f;
    m_normalLoad = 0.0f;
    m_longitudinalForce = 0.0f;

    if (!wheel.IsGrounded() ||
        deltaTime <= 0.0f) {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    const float normalLoad =
        wheel.GetForce();

    m_normalLoad = normalLoad;

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

    forward =
        Quaternion::FromAxisAngle(
            normal,
            wheel.GetSteeringAngle()
        ) * forward;

    const float longitudinalVelocity =
        contactVelocity.Dot(forward);

    const float wheelSurfaceSpeed =
        wheel.GetAngularVelocity() *
        wheel.GetRadius();

    const float longitudinalSlipVelocity =
        longitudinalVelocity -
        wheelSurfaceSpeed;

    Vec3 lateral =
        normal.Cross(forward);

    if (lateral.LengthSquared() <=
        0.000001f) {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    lateral = lateral.Normalized();

    const float lateralVelocity =
        contactVelocity.Dot(lateral);

    constexpr float SlipVelocityEpsilon = 0.1f;

    const float slipRatio =
        (wheelSurfaceSpeed -
         longitudinalVelocity) /
        std::max(
            std::abs(longitudinalVelocity),
            SlipVelocityEpsilon
        );

    const float slipAngle =
        std::atan2(
            lateralVelocity,
            std::abs(longitudinalVelocity)
        );

    m_longitudinalVelocity = longitudinalVelocity;
    m_lateralVelocity = lateralVelocity;
    m_wheelSurfaceSpeed = wheelSurfaceSpeed;
    m_longitudinalSlipVelocity = longitudinalSlipVelocity;
    m_slipRatio = slipRatio;
    m_slipAngle = slipAngle;

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

    const float wheelInverseMass =
        wheel.GetRadius() *
        wheel.GetRadius() /
        wheel.GetInertia();

    const float longitudinalInverseMass =
        longitudinalBodyInverseMass +
        wheelInverseMass;

    const float longitudinalImpulse =
        -longitudinalSlipVelocity /
        longitudinalInverseMass;

    const Vec3 lateralTorqueAxis =
        bodyRadius.Cross(lateral);

    const float lateralInverseMass =
        body.GetInverseMass() +
        lateralTorqueAxis.Dot(
            inverseInertia *
            lateralTorqueAxis
        );

    if (lateralInverseMass <= 0.0f) {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    const float lateralImpulse =
        -lateralVelocity /
        lateralInverseMass;

    const float slipSpeed =
        std::max(
            std::abs(longitudinalSlipVelocity),
            std::abs(lateralVelocity)
        );

    const float friction =
        slipSpeed < 0.5f
        ? m_staticFriction
        : m_dynamicFriction;

    const float maxImpulse =
        friction *
        normalLoad *
        deltaTime;

    const float impulseMagnitude =
        std::sqrt(
            longitudinalImpulse *
                longitudinalImpulse +
            lateralImpulse *
                lateralImpulse
        );

    float impulseScale = 1.0f;

    if (impulseMagnitude > maxImpulse &&
        impulseMagnitude > 0.000001f) {
        impulseScale =
            maxImpulse /
            impulseMagnitude;
    }

    const Vec3 tireImpulse =
        forward *
            (longitudinalImpulse *
             impulseScale) +
        lateral *
            (lateralImpulse *
             impulseScale);

    Vec3 tireForce =
        tireImpulse /
        deltaTime;

    const float contactSpeedSq =
        longitudinalVelocity *
        longitudinalVelocity;

    if (contactSpeedSq > 0.000001f) {
        const float contactSpeed =
            std::sqrt(contactSpeedSq);

        tireForce +=
            forward *
            (-longitudinalVelocity /
             contactSpeed *
             m_rollingResistance *
             normalLoad);
    }

    m_longitudinalForce =
        tireForce.Dot(forward);

    return tireForce;
}
