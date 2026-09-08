#pragma once

#include "../Core/Math/Vec3.h"
#include "Material.h"

class RigidBody;

class Collider {
public:
    Collider();
    ~Collider() = default;

    void SetHalfExtents(const Vec3& halfExtents);
    const Vec3& GetHalfExtents() const;

    void SetRigidBody(RigidBody* rigidBody);

    RigidBody* GetRigidBody();
    const RigidBody* GetRigidBody() const;

    Material& GetMaterial();
    const Material& GetMaterial() const;

private:
    Vec3 m_halfExtents;
    RigidBody* m_rigidBody;
    Material m_material;
};