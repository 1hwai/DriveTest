#pragma once

#include <memory>
#include <vector>

#include "RigidBody.h"

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld() = default;

    void SetGravity(const Vec3& gravity);
    const Vec3& GetGravity() const;

    RigidBody* CreateRigidBody();
    void AddRigidBody(std::unique_ptr<RigidBody> body);

    void Step(float deltaTime);

    const std::vector<std::unique_ptr<RigidBody>>&
        GetRigidBodies() const;

    void Clear();

private:
    Vec3 m_gravity;

    float m_debugTimer;

    std::vector<std::unique_ptr<RigidBody>>
        m_rigidBodies;
};