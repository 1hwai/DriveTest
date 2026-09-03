#pragma once

#include "Math/Transform.h"
#include "../Rendering/Mesh.h"

class RigidBody;

class Object {
public:
    Object();
    ~Object() = default;

    Transform& GetTransform();
    const Transform& GetTransform() const;

    void SetMesh(Mesh* mesh);

    Mesh* GetMesh();
    const Mesh* GetMesh() const;

    void SetRigidBody(RigidBody* rigidBody);

    RigidBody* GetRigidBody();
    const RigidBody* GetRigidBody() const;

private:
    Transform m_transform;
    Mesh* m_mesh;
    RigidBody* m_rigidBody;
};