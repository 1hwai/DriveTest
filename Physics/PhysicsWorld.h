#pragma once

#include <memory>
#include <vector>
#include <limits>

#include "RigidBody.h"
#include "Collider.h"
#include "Contact.h"
#include "Collision.h"
#include "../Core/Math/Transform.h"
#include "Solver.h"

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct RaycastResult {
    bool hit = false;
    float distance = 0.0f;
    Vec3 point;
    Vec3 normal;
    Collider* collider = nullptr;
    RigidBody* rigidBody = nullptr;
};

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld() = default;

    void SetGravity(const Vec3& gravity);
    const Vec3& GetGravity() const;

    RigidBody* CreateRigidBody();
    void AddRigidBody(std::unique_ptr<RigidBody> body);
    void DestroyRigidBody(RigidBody* body);

    Collider* CreateCollider();
    void AddCollider(std::unique_ptr<Collider> collider);
    void DestroyCollider(Collider* collider);

    void Step(float deltaTime);

    bool Raycast(
        const Ray& ray,
        RaycastResult& result,
        float maxDistance = std::numeric_limits<float>::infinity(),
        const RigidBody* ignoreBody = nullptr
    ) const;

    const std::vector<std::unique_ptr<RigidBody>>& GetRigidBodies() const;
    const std::vector<std::unique_ptr<Collider>>& GetColliders() const;
    const std::vector<Contact>& GetContacts() const;

    void Clear();

private:
    bool RaycastBox(
        const Ray& ray,
        const Collider& collider,
        const RigidBody& body,
        float maxDistance,
        RaycastResult& result
    ) const;

    bool RaycastSphere(
        const Ray& ray,
        const Collider& collider,
        const RigidBody& body,
        float maxDistance,
        RaycastResult& result
    ) const;

    bool RaycastPlane(
        const Ray& ray,
        const Collider& collider,
        const RigidBody& body,
        float maxDistance,
        RaycastResult& result
    ) const;

    bool RaycastTerrain(
        const Ray& ray,
        const Collider& collider,
        const RigidBody& body,
        float maxDistance,
        RaycastResult& result
    ) const;

private:
    Vec3 m_gravity;
    Solver m_solver;

    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;
    std::vector<std::unique_ptr<Collider>> m_colliders;

    std::vector<Contact> m_contacts;
    std::vector<Contact> m_previousContacts;

    float m_debugTimer;
};