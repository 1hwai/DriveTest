#include "Wheel.h"

#include "Suspension.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"

Wheel::Wheel()
    : m_localPosition(0.0f, 0.0f, 0.0f),
    m_radius(0.5f),
    m_grounded(false),
    m_suspensionLength(0.0f),
    m_compression(0.0f),
    m_force(0.0f),
    m_worldPosition(0.0f, 0.0f, 0.0f),
    m_contactPoint(0.0f, 0.0f, 0.0f) {}

void Wheel::SetLocalPosition(const Vec3& position) {
    m_localPosition = position;
}

const Vec3& Wheel::GetLocalPosition() const {
    return m_localPosition;
}

void Wheel::SetRadius(float radius) {
    m_radius = radius > 0.0f ? radius : 0.5f;
}

float Wheel::GetRadius() const {
    return m_radius;
}

void Wheel::Update(
    RigidBody& body,
    PhysicsWorld& physicsWorld,
    const Suspension& suspension,
    float deltaTime
) {
    const Vec3 worldMount =
        body.GetPosition() +
        body.GetOrientation() * m_localPosition;

    const Vec3 down =
        body.GetOrientation() *
        Vec3(0.0f, -1.0f, 0.0f);

    Ray ray;
    ray.origin = worldMount;
    ray.direction = down;

    RaycastResult result;

    const float maxRayDistance =
        suspension.GetMaxLength() + m_radius;

    if (!physicsWorld.Raycast(
        ray,
        result,
        maxRayDistance,
        &body
    )) {
        m_grounded = false;
        m_suspensionLength =
            suspension.GetMaxLength();
        m_compression = 0.0f;
        m_force = 0.0f;

        m_worldPosition =
            worldMount +
            down * m_suspensionLength;

        m_contactPoint =
            m_worldPosition;

        return;
    }

    const float wheelCenterDistance =
        result.distance - m_radius;

    m_suspensionLength =
        suspension.ClampLength(
            wheelCenterDistance
        );

    m_compression =
        suspension.GetRestLength() -
        m_suspensionLength;

    if (m_compression <= 0.0f) {
        m_compression = 0.0f;
        m_force = 0.0f;
    }
    else {
        const Vec3 pointVelocity =
            body.GetPointVelocity(worldMount);

        const float lengthVelocity =
            pointVelocity.Dot(down);

        const float compressionVelocity =
            -lengthVelocity;

        m_force =
            suspension.CalculateForce(
                m_compression,
                compressionVelocity
            );

        body.AddForceAtPoint(
            -down * m_force,
            worldMount
        );
    }

    m_grounded = true;
    m_contactPoint = result.point;

    m_worldPosition =
        worldMount +
        down * m_suspensionLength;

    (void)deltaTime;
}

bool Wheel::IsGrounded() const {
    return m_grounded;
}

float Wheel::GetSuspensionLength() const {
    return m_suspensionLength;
}

float Wheel::GetCompression() const {
    return m_compression;
}

float Wheel::GetForce() const {
    return m_force;
}

const Vec3& Wheel::GetWorldPosition() const {
    return m_worldPosition;
}

const Vec3& Wheel::GetContactPoint() const {
    return m_contactPoint;
}
