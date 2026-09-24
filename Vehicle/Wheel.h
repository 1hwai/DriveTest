#pragma once

#include "../Core/Math/Vec3.h"

class PhysicsWorld;
class RigidBody;
class Suspension;

class Wheel {
public:
    Wheel();

    void SetLocalPosition(const Vec3& position);
    const Vec3& GetLocalPosition() const;

    void SetRadius(float radius);
    float GetRadius() const;

    void Update(
        RigidBody& body,
        PhysicsWorld& physicsWorld,
        const Suspension& suspension,
        float deltaTime
    );

    bool IsGrounded() const;
    float GetSuspensionLength() const;
    float GetCompression() const;
    float GetForce() const;
    const Vec3& GetWorldPosition() const;
    const Vec3& GetContactPoint() const;
    const Vec3& GetContactNormal() const;

private:
    Vec3 m_localPosition;
    float m_radius;

    bool m_grounded;
    float m_suspensionLength;
    float m_compression;
    float m_force;

    Vec3 m_worldPosition;
    Vec3 m_contactPoint;
    Vec3 m_contactNormal;
};
