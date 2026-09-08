#include "Contact.h"

Contact::Contact()
    : m_bodyA(nullptr),
    m_bodyB(nullptr),
    m_normal(0.0f, 1.0f, 0.0f),
    m_pointCount(0),
    m_restitution(0.0f),
    m_friction(0.5f) {}

void Contact::SetBodies(
    RigidBody* bodyA,
    RigidBody* bodyB
) {
    m_bodyA = bodyA;
    m_bodyB = bodyB;
}

RigidBody* Contact::GetBodyA() {
    return m_bodyA;
}

const RigidBody* Contact::GetBodyA() const {
    return m_bodyA;
}

RigidBody* Contact::GetBodyB() {
    return m_bodyB;
}

const RigidBody* Contact::GetBodyB() const {
    return m_bodyB;
}

void Contact::SetNormal(const Vec3& normal) {
    m_normal = normal.Normalized();
}

const Vec3& Contact::GetNormal() const {
    return m_normal;
}

void Contact::SetRestitution(float restitution) {
    if (restitution < 0.0f)
        restitution = 0.0f;
    else if (restitution > 1.0f)
        restitution = 1.0f;

    m_restitution = restitution;
}

float Contact::GetRestitution() const {
    return m_restitution;
}

void Contact::SetFriction(float friction) {
    m_friction = friction > 0.0f ? friction : 0.0f;
}

float Contact::GetFriction() const {
    return m_friction;
}

void Contact::AddPoint(
    const Vec3& position,
    float penetration
) {
    if (m_pointCount >= MaxPoints)
        return;

    ContactPoint& point = m_points[m_pointCount];

    point.position = position;
    point.penetration = penetration > 0.0f ? penetration : 0.0f;
    point.normalImpulse = 0.0f;
    point.tangentImpulse = 0.0f;

    ++m_pointCount;
}

int Contact::GetPointCount() const {
    return m_pointCount;
}

ContactPoint& Contact::GetPoint(int index) {
    return m_points[index];
}

const ContactPoint& Contact::GetPoint(int index) const {
    return m_points[index];
}

void Contact::ClearPoints() {
    m_pointCount = 0;
}