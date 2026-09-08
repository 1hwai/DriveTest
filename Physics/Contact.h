#pragma once

#include "../Core/Math/Vec3.h"

class RigidBody;

struct ContactPoint {
    Vec3 position;
    float penetration;

    float normalImpulse;
    float tangentImpulse;

    ContactPoint()
        : position(0.0f, 0.0f, 0.0f),
        penetration(0.0f),
        normalImpulse(0.0f),
        tangentImpulse(0.0f) {}
};

class Contact {
public:
    static constexpr int MaxPoints = 4;

    Contact();

    void SetBodies(RigidBody* bodyA, RigidBody* bodyB);

    RigidBody* GetBodyA();
    const RigidBody* GetBodyA() const;

    RigidBody* GetBodyB();
    const RigidBody* GetBodyB() const;

    void SetNormal(const Vec3& normal);
    const Vec3& GetNormal() const;

    void SetRestitution(float restitution);
    float GetRestitution() const;

    void SetFriction(float friction);
    float GetFriction() const;

    void AddPoint(
        const Vec3& position,
        float penetration
    );

    int GetPointCount() const;

    ContactPoint& GetPoint(int index);
    const ContactPoint& GetPoint(int index) const;

    void ClearPoints();

private:
    RigidBody* m_bodyA;
    RigidBody* m_bodyB;

    Vec3 m_normal;

    ContactPoint m_points[MaxPoints];
    int m_pointCount;

    float m_restitution;
    float m_friction;
};