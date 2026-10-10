#include "Tire.h"

#include "Wheel.h"
#include "../VehicleCoordinates.h"
#include "../../Physics/RigidBody.h"
#include "../../Core/Math/Quaternion.h"

#include <algorithm>
#include <cmath>

Tire::Tire()
    : m_staticFriction(1.10f),
    m_dynamicFriction(0.95f),
    m_longitudinalStiffness(36000.0f),
    m_lateralStiffness(42000.0f),
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
        VehicleCoordinates::Forward(),
        VehicleCoordinates::Right(),
        Vec3(0.0f, 0.0f, 0.0f)
    },
    m_longitudinalForce(0.0f),
    m_lateralForce(0.0f) {}

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

float Tire::CalculateGripForce(
    float slip,
    float normalLoad,
    float peakSlip,
    float stiffness
) const {
    if (normalLoad <= 0.0f ||
        peakSlip <= 0.0f ||
        stiffness <= 0.0f ||
        std::abs(slip) <= 0.000001f)
        return 0.0f;

    const float peakForce =
        m_staticFriction * normalLoad;
    const float slidingForce =
        std::min(
            m_dynamicFriction * normalLoad,
            peakForce * 0.82f
        );

    const float x =
        std::abs(slip) / peakSlip;

    const float initialSlope =
        std::clamp(
            stiffness * peakSlip / peakForce,
            0.5f,
            1.2f
        );

    float forceFactor;

    if (x <= 1.0f) {
        const float x2 = x * x;
        const float x3 = x2 * x;

        forceFactor =
            (initialSlope - 2.0f) * x3 +
            (3.0f - 2.0f * initialSlope) * x2 +
            initialSlope * x;
    } else {
        const float t =
            std::clamp(x - 1.0f, 0.0f, 1.0f);
        const float smoothStep =
            t * t * (3.0f - 2.0f * t);

        forceFactor =
            1.0f +
            (slidingForce / peakForce - 1.0f) *
            smoothStep;
    }

    const float forceMagnitude =
        peakForce *
        std::max(0.0f, forceFactor);

    return std::copysign(
        forceMagnitude,
        slip
    );
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

float Tire::GetLateralForce() const {
    return m_lateralForce;
}

TireState Tire::CalculateState(
    const RigidBody& body,
    const Wheel& wheel
) const {
    TireState state{
        wheel.IsGrounded(),
        wheel.GetNormalLoad(),
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        VehicleCoordinates::Forward(),
        VehicleCoordinates::Right(),
        wheel.GetContactPoint()
    };

    if (!state.grounded) {
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
        wheel.GetWorldOrientation() *
        VehicleCoordinates::Forward();

    forward -=
        normal * forward.Dot(normal);

    if (forward.LengthSquared() <=
        0.000001f) {
        m_state = state;
        return state;
    }

    forward = forward.Normalized();

    // The solved hub orientation already contains steering and suspension
    // camber. Do not apply the steering angle a second time.
    // Lateral is the vehicle's right direction.
    // With +Z forward and +Y up, forward × normal gives -X (right).
    Vec3 lateral =
        forward.Cross(normal);

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

    if (state.normalLoad <= 0.0f) {
        m_state = state;
        return state;
    }

    state.wheelSurfaceSpeed =
        wheel.GetAngularVelocity() *
        wheel.GetRadius();

    state.longitudinalSlipVelocity =
        state.longitudinalVelocity -
        state.wheelSurfaceSpeed;

    constexpr float SlipReferenceSpeed = 1.0f;
    constexpr float SlipAngleReferenceSpeed = 1.0f;

    const float longitudinalSpeed =
        std::abs(state.longitudinalVelocity);

    const float slipSpeedReference =
        std::max(
            {
                longitudinalSpeed,
                std::abs(state.wheelSurfaceSpeed),
                SlipReferenceSpeed
            }
        );

    state.slipRatio =
        (state.wheelSurfaceSpeed -
         state.longitudinalVelocity) /
        slipSpeedReference;

    const float contactSpeed =
        std::sqrt(
            state.longitudinalVelocity *
                state.longitudinalVelocity +
            state.lateralVelocity *
                state.lateralVelocity
        );

    state.slipAngle =
        std::atan2(
            state.lateralVelocity,
            std::max(
                longitudinalSpeed,
                SlipAngleReferenceSpeed
            )
        );

    m_state = state;
    return state;
}

Vec3 Tire::CalculateForce(
    const TireState& state
) const {
    m_longitudinalForce = 0.0f;
    m_lateralForce = 0.0f;

    if (!state.grounded ||
        state.normalLoad <= 0.0f) {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    constexpr float PeakSlipRatio = 0.12f;

    const float longitudinalForce =
        CalculateGripForce(
            state.slipRatio,
            state.normalLoad,
            PeakSlipRatio,
            m_longitudinalStiffness
        );

    const float contactSpeed =
        std::sqrt(
            state.longitudinalVelocity *
                state.longitudinalVelocity +
            state.lateralVelocity *
                state.lateralVelocity
        );

    constexpr float LateralForceFadeStart = 0.02f;
    constexpr float LateralForceFadeEnd = 0.20f;

    const float lateralSpeedFactor =
        std::clamp(
            (contactSpeed - LateralForceFadeStart) /
            (LateralForceFadeEnd - LateralForceFadeStart),
            0.0f,
            1.0f
        );

    constexpr float PeakSlipAngle = 0.105f;

    const float lateralForce =
        -CalculateGripForce(
            state.slipAngle,
            state.normalLoad,
            PeakSlipAngle,
            m_lateralStiffness
        ) *
        lateralSpeedFactor;

    const float combinedSlip =
        std::sqrt(
            state.slipRatio *
                state.slipRatio +
            state.slipAngle *
                state.slipAngle
        );

    constexpr float FrictionTransitionSlip = 0.05f;

    const float frictionBlend =
        std::clamp(
            combinedSlip / FrictionTransitionSlip,
            0.0f,
            1.0f
        );

    const float friction =
        m_staticFriction +
        (m_dynamicFriction - m_staticFriction) *
        frictionBlend;

    const float maxForce =
        friction *
        state.normalLoad;

    float rollingResistanceForce = 0.0f;

    const float longitudinalSpeedSq =
        state.longitudinalVelocity *
        state.longitudinalVelocity;

    if (longitudinalSpeedSq > 0.000001f) {
        const float longitudinalSpeed =
            std::sqrt(longitudinalSpeedSq);

        rollingResistanceForce =
            -state.longitudinalVelocity /
            longitudinalSpeed *
            m_rollingResistance *
            state.normalLoad;
    }

    float totalLongitudinalForce =
        longitudinalForce +
        rollingResistanceForce;

    const float forceMagnitude =
        std::sqrt(
            totalLongitudinalForce *
                totalLongitudinalForce +
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

    totalLongitudinalForce *= forceScale;

    const float finalLateralForce =
        lateralForce *
        forceScale;

    const Vec3 tireForce =
        state.forward * totalLongitudinalForce +
        state.lateral * finalLateralForce;

    m_longitudinalForce =
        totalLongitudinalForce;
    m_lateralForce =
        finalLateralForce;

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
