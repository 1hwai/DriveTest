#include "SceneFactory.h"

#include "../Core/Object.h"
#include "../Physics/Collider.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Rendering/Mesh.h"
#include "../Rendering/MeshManager.h"

SceneFactory::SceneFactory(
    MeshManager& meshManager,
    PhysicsWorld& physicsWorld
)
    : m_meshManager(meshManager),
    m_physicsWorld(physicsWorld) {}

std::unique_ptr<Object> SceneFactory::CreateBox(
    const BoxSettings& settings
) {
    Mesh* mesh = m_meshManager.Get("cube");

    if (mesh == nullptr)
        return nullptr;

    auto object = std::make_unique<Object>();
    object->SetName(settings.name);
    object->SetMesh(mesh);

    RigidBody* body =
        m_physicsWorld.CreateRigidBody();

    body->SetPosition(settings.position);
    body->SetMass(settings.mass);
    body->SetBoxInertia(settings.halfExtents * 2.0f);

    Collider* collider =
        m_physicsWorld.CreateCollider();

    collider->SetShape(ColliderShape::Box);
    collider->SetHalfExtents(settings.halfExtents);
    collider->SetRigidBody(body);
    collider->GetMaterial().SetRestitution(
        settings.restitution
    );
    collider->GetMaterial().SetFriction(
        settings.friction
    );

    object->SetRigidBody(body);
    object->SetCollider(collider);

    object->GetTransform().position =
        body->GetPosition();
    object->GetTransform().scale =
        settings.halfExtents * 2.0f;

    return object;
}

std::unique_ptr<Object> SceneFactory::CreateSphere(
    const SphereSettings& settings
) {
    Mesh* mesh = m_meshManager.Get("sphere");

    if (mesh == nullptr)
        return nullptr;

    auto object = std::make_unique<Object>();
    object->SetName(settings.name);
    object->SetMesh(mesh);

    RigidBody* body =
        m_physicsWorld.CreateRigidBody();

    body->SetPosition(settings.position);
    body->SetMass(settings.mass);
    body->SetSphereInertia(settings.radius);

    Collider* collider =
        m_physicsWorld.CreateCollider();

    collider->SetShape(ColliderShape::Sphere);
    collider->SetRadius(settings.radius);
    collider->SetRigidBody(body);
    collider->GetMaterial().SetRestitution(
        settings.restitution
    );
    collider->GetMaterial().SetFriction(
        settings.friction
    );

    object->SetRigidBody(body);
    object->SetCollider(collider);

    object->GetTransform().position =
        body->GetPosition();

    const float diameter =
        settings.radius * 2.0f;

    object->GetTransform().scale =
        Vec3(diameter, diameter, diameter);

    return object;
}