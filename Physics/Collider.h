#pragma once

#include "../Core/Math/Vec3.h"
#include "Material.h"

class RigidBody;

enum class ColliderShape {
    Box,
    Sphere
};

class Collider {
public:
    Collider();
    ~Collider() = default;

    void SetShape(ColliderShape shape);
    ColliderShape GetShape() const;

    void SetHalfExtents(const Vec3& halfExtents);
    const Vec3& GetHalfExtents() const;

    // Sphere 전용. 반지름 r인 구는 CreateSphere() 메시(반지름 0.5)를
    // 스케일 2r로 그린 것과 대응한다(큐브가 halfExtents*2로 스케일되는
    // 것과 같은 관례).
    void SetRadius(float radius);
    float GetRadius() const;

    void SetRigidBody(RigidBody* rigidBody);

    RigidBody* GetRigidBody();
    const RigidBody* GetRigidBody() const;

    Material& GetMaterial();
    const Material& GetMaterial() const;

private:
    ColliderShape m_shape;
    Vec3 m_halfExtents;
    float m_radius;
    RigidBody* m_rigidBody;
    Material m_material;
};