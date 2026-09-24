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

    float GetStaticFriction() const;
    float GetDynamicFriction() const;
    float GetLongitudinalStiffness() const;
    float GetLateralStiffness() const;

    Vec3 CalculateForce(
        const RigidBody& body,
        const Wheel& wheel
    ) const;

private:
    float m_staticFriction;
    float m_dynamicFriction;
    float m_longitudinalStiffness;
    float m_lateralStiffness;
};
