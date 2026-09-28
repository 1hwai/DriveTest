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
    m_state{
        false,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        Vec3(0.0f, 0.0f, 1.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(0.0f, 0.0f, 0.0f)
    },
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

const TireState& Tire::GetState() const {
    return m_state;
}

float Tire::GetLongitudinalVelocity() const {
    return m_state.longitudinalVelocity;
}

float Tire::GetLateralVelocity() const {
    return m_state.lateralVelocity;
}

float Tire::GetWheelSurfaceSpeed() const {
    return m_state.wheelSurfaceSpeed;
}

float Tire::GetLongitudinalSlipVelocity() const {
    return m_state.longitudinalSlipVelocity;
}

float Tire::GetSlipRatio() const {
    return m_state.slipRatio;
}

float Tire::GetSlipAngle() const {
    return m_state.slipAngle;
}

float Tire::GetNormalLoad() const {
    return m_state.normalLoad;
}

float Tire::GetLongitudinalForce() const {
    return m_longitudinalForce;
}

TireState Tire::CalculateState(
    const RigidBody& body,
    const Wheel& wheel
) const {
    TireState state{
        wheel.IsGrounded(),
        wheel.GetForce(),
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        Vec3(0.0f, 0.0f, 1.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        wheel.GetContactPoint()
    };

    if (!state.grounded ||
        state.normalLoad <= 0.0f) {
        m_state = state;
        return state;
    }

    const Vec3 contactVelocity =
        body.GetPointVelocity(
            state.contactPoint
        );

    const Vec3 normal =
        wheel.GetContactNormal();

    Vec3 forward =
        body.GetOrientation() *
        Vec3(0.0f, 0.0f, 1.0f);

    forward -=
        normal * forward.Dot(normal);

    if (forward.LengthSquared() <=
        0.000001f) {
        m_state = state;
        return state;
    }

    forward = forward.Normalized();

    forward =
        Quaternion::FromAxisAngle(
            normal,
            wheel.GetSteeringAngle()
        ) * forward;

    Vec3 lateral =
        normal.Cross(forward);

    if (lateral.LengthSquared() <=
        0.000001f) {
        m_state = state;
        return state;
    }

    lateral = lateral.Normalized();

    state.forward = forward;
    state.lateral = lateral;

    state.longitudinalVelocity =
        contactVelocity.Dot(forward);

    state.lateralVelocity =
        contactVelocity.Dot(lateral);

    state.wheelSurfaceSpeed =
        wheel.GetAngularVelocity() *
        wheel.GetRadius();

    state.longitudinalSlipVelocity =
        state.longitudinalVelocity -
        state.wheelSurfaceSpeed;

    constexpr float SlipVelocityEpsilon = 0.1f;

    state.slipRatio =
        (state.wheelSurfaceSpeed -
         state.longitudinalVelocity) /
        std::max(
            std::abs(state.longitudinalVelocity),
            SlipVelocityEpsilon
        );

    state.slipAngle =
        std::atan2(
            state.lateralVelocity,
            std::abs(state.longitudinalVelocity)
        );

    m_state = state;
    return state;
}

Vec3 Tire::CalculateForce(
    const TireState& state
) const {
    m_longitudinalForce = 0.0f;

    if (!state.grounded ||
        state.normalLoad <= 0.0f) {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    const float longitudinalForce =
        state.slipRatio *
        m_longitudinalStiffness;

    const float lateralForce =
        -state.slipAngle *
        m_lateralStiffness;

    const float combinedSlip =
        std::sqrt(
            state.slipRatio *
                state.slipRatio +
            state.slipAngle *
                state.slipAngle
        );

    const float friction =
        combinedSlip < 0.05f
        ? m_staticFriction
        : m_dynamicFriction;

    const float maxForce =
        friction *
        state.normalLoad;

    const float forceMagnitude =
        std::sqrt(
            longitudinalForce *
                longitudinalForce +
            lateralForce *
                lateralForce
        );

    float forceScale = 1.0f;

    if (forceMagnitude > maxForce &&
        forceMagnitude > 0.000001f) {
        forceScale =
            maxForce /
            forceMagnitude;
    }

    Vec3 tireForce =
        state.forward *
            (longitudinalForce * forceScale) +
        state.lateral *
            (lateralForce * forceScale);

    const float longitudinalSpeedSq =
        state.longitudinalVelocity *
        state.longitudinalVelocity;

    if (longitudinalSpeedSq > 0.000001f) {
        const float longitudinalSpeed =
            std::sqrt(longitudinalSpeedSq);

        tireForce +=
            state.forward *
            (-state.longitudinalVelocity /
             longitudinalSpeed *
             m_rollingResistance *
             state.normalLoad);
    }

    m_longitudinalForce =
        tireForce.Dot(state.forward);

    return tireForce;
}

Vec3 Tire::CalculateForce(
    const RigidBody& body,
    const Wheel& wheel
) const {
    const TireState state =
        CalculateState(body, wheel);

    return CalculateForce(state);
}
