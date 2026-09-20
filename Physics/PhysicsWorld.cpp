#include "PhysicsWorld.h"

#include <algorithm>

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
    auto body =
        std::make_unique<RigidBody>();

    RigidBody* result =
        body.get();

    m_rigidBodies.push_back(
        std::move(body)
    );

    return result;

}

void PhysicsWorld::AddRigidBody(
    std::unique_ptr<RigidBody> body
) {
    if (body)
        m_rigidBodies.push_back(
            std::move(body)
        );
}

Collider* PhysicsWorld::CreateCollider() {
    auto collider =
        std::make_unique<Collider>();

    Collider* result =
        collider.get();

    m_colliders.push_back(
        std::move(collider)
    );

    return result;

}

void PhysicsWorld::AddCollider(
    std::unique_ptr<Collider> collider
) {
    if (collider)
        m_colliders.push_back(
            std::move(collider)
        );
}

void PhysicsWorld::Step(float deltaTime) {
    if (deltaTime <= 0.0f)
        return;

    m_contacts.clear();

    // 1. Integrate
    for (auto& body : m_rigidBodies) {
        body->Integrate(
            deltaTime,
            m_gravity
        );
    }

    // 2. Collision detection
    for (size_t i = 0;
        i < m_colliders.size();
        ++i) {

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

            if (bodyA->IsSleeping() &&
                bodyB->IsSleeping()) {
                continue;
            }

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

            const ColliderShape shapeA =
                colliderA->GetShape();

            const ColliderShape shapeB =
                colliderB->GetShape();

            bool collided = false;

            if (shapeA == ColliderShape::Box &&
                shapeB == ColliderShape::Box) {

                collided =
                    Collision::CheckBoxBox(
                        transformA,
                        colliderA->GetHalfExtents(),
                        bodyA,
                        transformB,
                        colliderB->GetHalfExtents(),
                        bodyB,
                        contact
                    );
            }
            else if (shapeA == ColliderShape::Sphere &&
                shapeB == ColliderShape::Sphere) {

                collided =
                    Collision::CheckSphereSphere(
                        transformA,
                        colliderA->GetRadius(),
                        bodyA,
                        transformB,
                        colliderB->GetRadius(),
                        bodyB,
                        contact
                    );
            }
            else if (shapeA == ColliderShape::Sphere) {
                // A = sphere, B = box
                collided =
                    Collision::CheckSphereBox(
                        transformA,
                        colliderA->GetRadius(),
                        bodyA,
                        transformB,
                        colliderB->GetHalfExtents(),
                        bodyB,
                        true,
                        contact
                    );
            }
            else {
                // A = box, B = sphere
                collided =
                    Collision::CheckSphereBox(
                        transformB,
                        colliderB->GetRadius(),
                        bodyB,
                        transformA,
                        colliderA->GetHalfExtents(),
                        bodyA,
                        false,
                        contact
                    );
            }

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

            contact.SetRestitution(
                restitution
            );

            contact.SetFriction(
                friction
            );

            for (const Contact& previous :
                m_previousContacts) {

                if (previous.GetBodyA() != bodyA ||
                    previous.GetBodyB() != bodyB) {
                    continue;
                }

                contact.WarmStartFrom(
                    previous
                );

                break;
            }

            m_contacts.push_back(
                contact
            );
        }
    }

    // 3. Position correction (NGS, 여러 번 반복)
    for (int iteration = 0;
        iteration < m_solver.GetPositionIterations();
        ++iteration) {

        for (Contact& contact :
            m_contacts) {

            m_solver.SolvePosition(
                contact
            );
        }
    }

    // 4. Prepare restitution bias (프레임당 1회, 워밍스타트 전)
    for (Contact& contact :
        m_contacts) {

        m_solver.PrepareVelocityBias(
            contact
        );
    }

    // 5. Warm start
    for (Contact& contact :
        m_contacts) {

        m_solver.WarmStart(
            contact
        );
    }

    // 6. Iterative velocity solver
    for (int iteration = 0;
        iteration < m_solver.GetVelocityIterations();
        ++iteration) {

        for (Contact& contact :
            m_contacts) {

            m_solver.SolveVelocity(
                contact
            );
        }

        for (Contact& contact :
            m_contacts) {

            m_solver.SolveFriction(
                contact
            );
        }
    }

    // 7. Sleep
    for (auto& body :
        m_rigidBodies) {

        body->UpdateSleep(
            deltaTime
        );
    }

    // 8. Preserve contact impulses
    m_previousContacts =
        m_contacts;

    // 8. Debug timer
    m_debugTimer += deltaTime;

    if (m_debugTimer >= 1.0f)
        m_debugTimer = 0.0f;

}

const std::vector<std::unique_ptr<RigidBody>>&
PhysicsWorld::GetRigidBodies() const {
    return m_rigidBodies;
}

const std::vector<std::unique_ptr<Collider>>&
PhysicsWorld::GetColliders() const {
    return m_colliders;
}

const std::vector<Contact>&
PhysicsWorld::GetContacts() const {
    return m_contacts;
}

void PhysicsWorld::Clear() {
    m_contacts.clear();
    m_previousContacts.clear();
    m_colliders.clear();
    m_rigidBodies.clear();
}