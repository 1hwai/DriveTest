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
};
