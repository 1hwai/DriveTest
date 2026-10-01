#include "Object.h"

Object::Object()
    : m_transform(),
    m_name("Object"),
    m_color(1.0f, 1.0f, 1.0f),
    m_scenePersistent(true),
    m_mesh(nullptr),
    m_rigidBody(nullptr),
    m_collider(nullptr) {}

Transform& Object::GetTransform() {
    return m_transform;
}

const Transform& Object::GetTransform() const {
    return m_transform;
}

void Object::SetName(const std::string& name) {
    m_name = name;
}

const std::string& Object::GetName() const {
    return m_name;
}

void Object::SetColor(const Vec3& color) {
    m_color = color;
}

const Vec3& Object::GetColor() const {
    return m_color;
}

void Object::SetScenePersistent(bool persistent) {
    m_scenePersistent = persistent;
}

bool Object::IsScenePersistent() const {
    return m_scenePersistent;
}

void Object::SetMesh(Mesh* mesh) {
    m_mesh = mesh;
}

Mesh* Object::GetMesh() {
    return m_mesh;
}

const Mesh* Object::GetMesh() const {
    return m_mesh;
}

void Object::SetRigidBody(RigidBody* rigidBody) {
    m_rigidBody = rigidBody;
}

RigidBody* Object::GetRigidBody() {
    return m_rigidBody;
}

const RigidBody* Object::GetRigidBody() const {
    return m_rigidBody;
}

void Object::SetCollider(Collider* collider) {
    m_collider = collider;
}

Collider* Object::GetCollider() {
    return m_collider;
}

const Collider* Object::GetCollider() const {
    return m_collider;
}