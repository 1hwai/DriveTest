#pragma once

#include "Math/Transform.h"

class Mesh;

class Object
{
public:
    Object();
    ~Object() = default;

    Transform& GetTransform();
    const Transform& GetTransform() const;

    void SetMesh(Mesh* mesh);

    Mesh* GetMesh();
    const Mesh* GetMesh() const;

private:
    Transform m_transform;
    Mesh* m_mesh;
};