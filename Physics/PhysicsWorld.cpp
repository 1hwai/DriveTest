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

bool PhysicsWorld::Raycast(
    const Raycast& ray,
    RaycastResult& result,
    float maxDistance
) const {
    result = RaycastResult();

    const float directionLength = ray.direction.Length();

    if (directionLength <= 0.000001f ||
        maxDistance < 0.0f) {
        return false;
    }

    Raycast normalizedRay = ray;
    normalizedRay.direction =
        ray.direction / directionLength;

    bool hit = false;
    float closestDistance = maxDistance;

    for (const auto& collider : m_colliders) {
        if (!collider)
            continue;

        const RigidBody* body =
            collider->GetRigidBody();

        if (!body)
            continue;

        RaycastResult candidate;

        bool candidateHit = false;

        if (collider->GetShape() ==
            ColliderShape::Box) {

            candidateHit =
                RaycastBox(
                    normalizedRay,
                    *collider,
                    *body,
                    closestDistance,
                    candidate
                );
        }
        else if (collider->GetShape() ==
            ColliderShape::Sphere) {

            candidateHit =
                RaycastSphere(
                    normalizedRay,
                    *collider,
                    *body,
                    closestDistance,
                    candidate
                );
        }

        if (!candidateHit)
            continue;

        if (!hit ||
            candidate.distance < closestDistance) {

            hit = true;
            closestDistance = candidate.distance;
            result = candidate;
        }
    }

    return hit;
}

bool PhysicsWorld::RaycastBox(
    const Raycast& ray,
    const Collider& collider,
    const RigidBody& body,
    float maxDistance,
    RaycastResult& result
) const {
    const Quaternion rotation =
        body.GetOrientation();

    const Quaternion inverseRotation =
        rotation.Conjugate();

    const Vec3 localOrigin =
        inverseRotation *
        (ray.origin - body.GetPosition());

    const Vec3 localDirection =
        inverseRotation *
        ray.direction;

    const Vec3& halfExtents =
        collider.GetHalfExtents();

    float tMin = 0.0f;
    float tMax = maxDistance;
    int hitAxis = -1;
    float hitSign = 0.0f;

    constexpr float Epsilon = 0.000001f;

    for (int axis = 0; axis < 3; ++axis) {
        const float origin =
            axis == 0 ? localOrigin.x :
            axis == 1 ? localOrigin.y :
            localOrigin.z;

        const float direction =
            axis == 0 ? localDirection.x :
            axis == 1 ? localDirection.y :
            localDirection.z;

        const float extent =
            axis == 0 ? halfExtents.x :
            axis == 1 ? halfExtents.y :
            halfExtents.z;

        if (std::fabs(direction) <= Epsilon) {
            if (origin < -extent ||
                origin > extent) {
                return false;
            }

            continue;
        }

        float t1 =
            (-extent - origin) / direction;

        float t2 =
            (extent - origin) / direction;

        float nearSign = -1.0f;

        if (t1 > t2) {
            std::swap(t1, t2);
            nearSign = 1.0f;
        }

        if (t1 > tMin) {
            tMin = t1;
            hitAxis = axis;
            hitSign = nearSign;
        }

        tMax = std::min(tMax, t2);

        if (tMin > tMax)
            return false;
    }

    float distance = tMin;

    if (distance < 0.0f) {
        distance = tMax;

        if (distance < 0.0f)
            return false;

        const Vec3 localPoint =
            localOrigin +
            localDirection * distance;

        const float distances[3] = {
            std::fabs(std::fabs(localPoint.x) - halfExtents.x),
            std::fabs(std::fabs(localPoint.y) - halfExtents.y),
            std::fabs(std::fabs(localPoint.z) - halfExtents.z)
        };

        hitAxis = 0;

        if (distances[1] < distances[hitAxis])
            hitAxis = 1;

        if (distances[2] < distances[hitAxis])
            hitAxis = 2;

        hitSign =
            hitAxis == 0 ? (localPoint.x >= 0.0f ? 1.0f : -1.0f) :
            hitAxis == 1 ? (localPoint.y >= 0.0f ? 1.0f : -1.0f) :
            (localPoint.z >= 0.0f ? 1.0f : -1.0f);
    }

    if (distance > maxDistance)
        return false;

    Vec3 localNormal(0.0f, 0.0f, 0.0f);

    if (hitAxis == 0)
        localNormal.x = hitSign;
    else if (hitAxis == 1)
        localNormal.y = hitSign;
    else if (hitAxis == 2)
        localNormal.z = hitSign;
    else
        return false;

    result.hit = true;
    result.distance = distance;
    result.point =
        ray.origin +
        ray.direction * distance;
    result.normal =
        (rotation * localNormal).Normalized();
    result.collider =
        const_cast<Collider*>(&collider);
    result.rigidBody =
        const_cast<RigidBody*>(&body);

    return true;
}

bool PhysicsWorld::RaycastSphere(
    const Raycast& ray,
    const Collider& collider,
    const RigidBody& body,
    float maxDistance,
    RaycastResult& result
) const {
    const Vec3 offset =
        ray.origin - body.GetPosition();

    const float radius =
        collider.GetRadius();

    const float a =
        ray.direction.Dot(ray.direction);

    const float b =
        2.0f *
        offset.Dot(ray.direction);

    const float c =
        offset.Dot(offset) -
        radius * radius;

    const float discriminant =
        b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
        return false;

    const float sqrtDiscriminant =
        std::sqrt(discriminant);

    float t =
        (-b - sqrtDiscriminant) /
        (2.0f * a);

    if (t < 0.0f) {
        t =
            (-b + sqrtDiscriminant) /
            (2.0f * a);
    }

    if (t < 0.0f ||
        t > maxDistance) {
        return false;
    }

    const Vec3 point =
        ray.origin +
        ray.direction * t;

    const Vec3 normal =
        (point - body.GetPosition()).Normalized();

    result.hit = true;
    result.distance = t;
    result.point = point;
    result.normal = normal;
    result.collider =
        const_cast<Collider*>(&collider);
    result.rigidBody =
        const_cast<RigidBody*>(&body);

    return true;
}
