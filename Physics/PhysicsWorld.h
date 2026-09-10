#pragma once

#include <memory>
#include <vector>

#include "RigidBody.h"
#include "Collider.h"
#include "Contact.h"
#include "Collision.h"
#include "../Core/Math/Transform.h"
#include "Solver.h"

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld() = default;

    void SetGravity(const Vec3& gravity);
    const Vec3& GetGravity() const;

    RigidBody* CreateRigidBody();
    void AddRigidBody(std::unique_ptr<RigidBody> body);

    Collider* CreateCollider();
    void AddCollider(std::unique_ptr<Collider> collider);

    void Step(float deltaTime);

    const std::vector<std::unique_ptr<RigidBody>>& GetRigidBodies() const;
    const std::vector<std::unique_ptr<Collider>>& GetColliders() const;
    const std::vector<Contact>& GetContacts() const;

    void Clear();

private:
    Vec3 m_gravity;
    Solver m_solver;

    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;
    std::vector<std::unique_ptr<Collider>> m_colliders;

    std::vector<Contact> m_contacts;
    std::vector<Contact> m_previousContacts;

    float m_debugTimer;
};