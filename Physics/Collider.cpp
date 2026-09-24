#include "Collider.h"

Collider::Collider()
    : m_shape(ColliderShape::Box),
    m_halfExtents(0.5f, 0.5f, 0.5f),
    m_radius(0.5f),
    m_planeHeight(0.0f),
    m_terrain(nullptr),
    m_rigidBody(nullptr) {}

void Collider::SetShape(ColliderShape shape) {
    m_shape = shape;
}

ColliderShape Collider::GetShape() const {
    return m_shape;
}

void Collider::SetHalfExtents(
    const Vec3& halfExtents
) {
    m_halfExtents = halfExtents;
}

const Vec3& Collider::GetHalfExtents() const {
    return m_halfExtents;
}

void Collider::SetRadius(float radius) {
    m_radius = radius > 0.0f ? radius : 0.0f;
}

float Collider::GetRadius() const {
    return m_radius;
}

void Collider::SetPlaneHeight(float height) {
    m_planeHeight = height;
}

float Collider::GetPlaneHeight() const {
    return m_planeHeight;
}

void Collider::SetTerrain(const Terrain* terrain) {
    m_terrain = terrain;
}

const Terrain* Collider::GetTerrain() const {
    return m_terrain;
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