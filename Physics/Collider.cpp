#include "Collider.h"

Collider::Collider()
    : m_halfExtents(0.5f, 0.5f, 0.5f),
    m_rigidBody(nullptr) {}

void Collider::SetHalfExtents(
    const Vec3& halfExtents
) {
    m_halfExtents = halfExtents;
}

const Vec3& Collider::GetHalfExtents() const {
    return m_halfExtents;
}

void Collider::SetRigidBody(
    RigidBody* rigidBody
) {
    m_rigidBody = rigidBody;
}

RigidBody* Collider::GetRigidBody() {
    return m_rigidBody;
}

const RigidBody* Collider::GetRigidBody() const {
    return m_rigidBody;
}

Material& Collider::GetMaterial() {
    return m_material;
}

const Material& Collider::GetMaterial() const {
    return m_material;
}