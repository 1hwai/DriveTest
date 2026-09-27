#pragma once

#include "../Core/Math/Vec3.h"

class RigidBody;
class Wheel;

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

    float GetLongitudinalVelocity() const;
    float GetLateralVelocity() const;
    float GetWheelSurfaceSpeed() const;
    float GetLongitudinalSlipVelocity() const;
    float GetSlipRatio() const;
    float GetSlipAngle() const;
    float GetNormalLoad() const;
    float GetLongitudinalForce() const;

    Vec3 CalculateForce(
        const RigidBody& body,
        const Wheel& wheel,
        float deltaTime
    ) const;

private:
    float m_staticFriction;
    float m_dynamicFriction;
    float m_longitudinalStiffness;
    float m_lateralStiffness;
    float m_rollingResistance;

    mutable float m_longitudinalVelocity;
    mutable float m_lateralVelocity;
    mutable float m_wheelSurfaceSpeed;
    mutable float m_longitudinalSlipVelocity;
    mutable float m_slipRatio;
    mutable float m_slipAngle;
    mutable float m_normalLoad;
    mutable float m_longitudinalForce;
};
