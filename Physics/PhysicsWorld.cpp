#include "PhysicsWorld.h"
#include "../Core/Debug/Logger.h"

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

void PhysicsWorld::AddRigidBody(std::unique_ptr<RigidBody> body) {
    if (body)
        m_rigidBodies.push_back(std::move(body));
}

Collider* PhysicsWorld::CreateCollider() {
    auto collider = std::make_unique<Collider>();
    Collider* result = collider.get();

    m_colliders.push_back(std::move(collider));

    return result;
}

void PhysicsWorld::AddCollider(std::unique_ptr<Collider> collider) {
    if (collider)
        m_colliders.push_back(std::move(collider));
}

void PhysicsWorld::Step(float deltaTime) {
    if (deltaTime <= 0.0f)
        return;

    m_contacts.clear();

    // 1. Integrate
    for (auto& body : m_rigidBodies)
        body->Integrate(deltaTime, m_gravity);

    // 2. Collision detection
    for (size_t i = 0; i < m_colliders.size(); ++i) {
        Collider* colliderA =
            m_colliders[i].get();

        RigidBody* bodyA =
            colliderA->GetRigidBody();

        if (!bodyA)
            continue;

        for (size_t j = i + 1;
            j < m_colliders.size();
            ++j) {

            Collider* colliderB =
                m_colliders[j].get();

            RigidBody* bodyB =
                colliderB->GetRigidBody();

            if (!bodyB)
                continue;

            if (bodyA == bodyB)
                continue;

            // Both bodies are sleeping.
            // Nothing can change their state, so there
            // is no reason to solve this contact.
            if (bodyA->IsSleeping() &&
                bodyB->IsSleeping()) {
                continue;
            }

            // An awake dynamic body touching a sleeping
            // dynamic body wakes the sleeping body.
            if (bodyA->IsSleeping() &&
                bodyB->GetInverseMass() > 0.0f &&
                !bodyB->IsSleeping()) {
                bodyA->Wake();
            }

            if (bodyB->IsSleeping() &&
                bodyA->GetInverseMass() > 0.0f &&
                !bodyA->IsSleeping()) {
                bodyB->Wake();
            }

            Transform transformA;
            transformA.position =
                bodyA->GetPosition();

            transformA.rotation =
                bodyA->GetOrientation();

            Transform transformB;
            transformB.position =
                bodyB->GetPosition();

            transformB.rotation =
                bodyB->GetOrientation();

            Contact contact;

            bool collided =
                Collision::CheckBoxBox(
                    transformA,
                    colliderA->GetHalfExtents(),
                    bodyA,
                    transformB,
                    colliderB->GetHalfExtents(),
                    bodyB,
                    contact
                );

            if (!collided)
                continue;

            float restitution =
                std::max(
                    colliderA->GetMaterial()
                    .GetRestitution(),
                    colliderB->GetMaterial()
                    .GetRestitution()
                );

            float friction =
                std::max(
                    colliderA->GetMaterial()
                    .GetFriction(),
                    colliderB->GetMaterial()
                    .GetFriction()
                );

            contact.SetRestitution(restitution);
            contact.SetFriction(friction);

            Logger::Info(
                "Contact normal: (" +
                std::to_string(
                    contact.GetNormal().x
                ) + ", " +
                std::to_string(
                    contact.GetNormal().y
                ) + ", " +
                std::to_string(
                    contact.GetNormal().z
                ) + ")"
            );

            for (int k = 0;
                k < contact.GetPointCount();
                ++k) {

                const ContactPoint& point =
                    contact.GetPoint(k);

                Logger::Info(
                    "Contact point: (" +
                    std::to_string(
                        point.position.x
                    ) + ", " +
                    std::to_string(
                        point.position.y
                    ) + ", " +
                    std::to_string(
                        point.position.z
                    ) +
                    "), penetration: " +
                    std::to_string(
                        point.penetration
                    )
                );
            }

            m_contacts.push_back(contact);
        }
    }

    // 3. Position correction
    for (Contact& contact : m_contacts)
        m_solver.SolvePosition(contact);

    // 4. Iterative velocity solver
    for (int iteration = 0;
        iteration < m_solver.GetVelocityIterations();
        ++iteration) {

        for (Contact& contact : m_contacts)
            m_solver.SolveVelocity(contact);

        for (Contact& contact : m_contacts)
            m_solver.SolveFriction(contact);
    }

    // 5. Sleep stable bodies
    for (auto& body : m_rigidBodies)
        body->UpdateSleep(deltaTime);

    // 6. Debug
    m_debugTimer += deltaTime;

    if (m_debugTimer >= 1.0f) {
        m_debugTimer = 0.0f;

        Logger::Info(
            "Physics bodies: " +
            std::to_string(
                m_rigidBodies.size()
            ) +
            ", colliders: " +
            std::to_string(
                m_colliders.size()
            ) +
            ", contacts: " +
            std::to_string(
                m_contacts.size()
            )
        );
    }
}

const std::vector<std::unique_ptr<RigidBody>>& PhysicsWorld::GetRigidBodies() const {
    return m_rigidBodies;
}

const std::vector<std::unique_ptr<Collider>>& PhysicsWorld::GetColliders() const {
    return m_colliders;
}

const std::vector<Contact>& PhysicsWorld::GetContacts() const {
    return m_contacts;
}

void PhysicsWorld::Clear() {
    m_contacts.clear();
    m_colliders.clear();
    m_rigidBodies.clear();
}