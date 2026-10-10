#pragma once

#include "../../Core/Math/Vec3.h"

class RigidBody;
class Wheel;

struct TireState {
    bool grounded;
    float normalLoad;
    float longitudinalVelocity;
    float lateralVelocity;
    float wheelSurfaceSpeed;
    float longitudinalSlipVelocity;
    float slipRatio;
    float slipAngle;
    Vec3 forward;
    Vec3 lateral;
    Vec3 contactPoint;
};

class Tire {
public:
    Tire();

    void SetStaticFriction(float friction);
    void SetDynamicFriction(float friction);
    void SetLongitudinalStiffness(float stiffness);
    void SetLateralStiffness(float stiffness);
    void SetRollingResistance(float coefficient);

    float GetStaticFriction() const;
    float GetDynamicFriction() const;
    float GetLongitudinalStiffness() const;
    float GetLateralStiffness() const;
    float GetRollingResistance() const;

    const TireState& GetState() const;

    float GetLongitudinalVelocity() const;
    float GetLateralVelocity() const;
    float GetWheelSurfaceSpeed() const;
    float GetLongitudinalSlipVelocity() const;
    float GetSlipRatio() const;
    float GetSlipAngle() const;
    float GetNormalLoad() const;
    float GetLongitudinalForce() const;
    float GetLateralForce() const;

    TireState CalculateState(
        const RigidBody& body,
        const Wheel& wheel
    ) const;

    Vec3 CalculateForce(
        const TireState& state
    ) const;

    Vec3 CalculateForce(
        const RigidBody& body,
        const Wheel& wheel
    ) const;

private:
    float CalculateGripForce(
        float slip,
        float normalLoad,
        float peakSlip,
        float stiffness
    ) const;

    float m_staticFriction;
    float m_dynamicFriction;
    float m_longitudinalStiffness;
    float m_lateralStiffness;
    float m_rollingResistance;

    mutable TireState m_state;
    mutable float m_longitudinalForce;
    mutable float m_lateralForce;
};
