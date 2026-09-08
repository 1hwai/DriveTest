#include "Object.h"

Object::Object()
    : m_transform(),
    m_mesh(nullptr),
    m_rigidBody(nullptr),
	m_collider(nullptr)
{}

Transform& Object::GetTransform() {
    return m_transform;
}

const Transform& Object::GetTransform() const {
    return m_transform;
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