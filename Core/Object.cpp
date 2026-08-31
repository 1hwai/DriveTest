#include "Object.h"

Object::Object()
    : m_transform(),
    m_mesh(nullptr)
{}

Transform& Object::GetTransform()
{
    return m_transform;
}

const Transform& Object::GetTransform() const
{
    return m_transform;
}

void Object::SetMesh(Mesh* mesh)
{
    m_mesh = mesh;
}

Mesh* Object::GetMesh()
{
    return m_mesh;
}

const Mesh* Object::GetMesh() const
{
    return m_mesh;
}