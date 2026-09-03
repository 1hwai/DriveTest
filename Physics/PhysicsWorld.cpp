#include "PhysicsWorld.h"
#include "../Core/Debug/Logger.h"

#include "../Core/Debug/Logger.h"

#include <iomanip>
#include <sstream>

PhysicsWorld::PhysicsWorld()
    : m_gravity(0.0f, -9.81f, 0.0f),
    m_debugTimer(0.0f) {}

void PhysicsWorld::SetGravity(const Vec3& gravity) {
    m_gravity = gravity;
}

const Vec3& PhysicsWorld::GetGravity() const {
    return m_gravity;
}

RigidBody* PhysicsWorld::CreateRigidBody() {
    auto body = std::make_unique<RigidBody>();

    RigidBody* result = body.get();

    m_rigidBodies.push_back(std::move(body));

    return result;
}

void PhysicsWorld::AddRigidBody(
    std::unique_ptr<RigidBody> body
) {
    if (body == nullptr)
        return;

    m_rigidBodies.push_back(std::move(body));
}

void PhysicsWorld::Step(float deltaTime) {
    if (deltaTime <= 0.0f)
        return;

    for (auto& body : m_rigidBodies) {
        if (body == nullptr)
            continue;

        body->Integrate(
            deltaTime,
            m_gravity
        );
    }

    m_debugTimer += deltaTime;

    if (m_debugTimer >= 1.0f) {
        m_debugTimer = 0.0f;

        for (const auto& body : m_rigidBodies) {
            if (body == nullptr)
                continue;

            const Vec3& position =
                body->GetPosition();

            const Vec3& velocity =
                body->GetLinearVelocity();

            std::ostringstream stream;

            stream << std::fixed
                << std::setprecision(3)
                << "RigidBody"
                << " position=("
                << position.x << ", "
                << position.y << ", "
                << position.z << ")"
                << " velocity=("
                << velocity.x << ", "
                << velocity.y << ", "
                << velocity.z << ")";

            Logger::Debug(stream.str());
        }
    }
}



const std::vector<std::unique_ptr<RigidBody>>&
PhysicsWorld::GetRigidBodies() const {
    return m_rigidBodies;
}

void PhysicsWorld::Clear() {
    m_rigidBodies.clear();
}