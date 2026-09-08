#include "World.h"

World::World()
    : m_physicsWorld(nullptr) {}

World::~World() {
    Shutdown();
}

bool World::Initialize(
    MeshManager& meshManager,
    PhysicsWorld& physicsWorld
) {
    m_physicsWorld = &physicsWorld;

    // Zero gravity for testing purposes
    //m_physicsWorld->SetGravity(Vec3(0.0f, 0.0f, 0.0f));

    Mesh* cubeMesh = meshManager.Get("cube");

    // Object 1 - Dynamic
    auto object = std::make_unique<Object>();
    object->SetMesh(cubeMesh);

    RigidBody* body = m_physicsWorld->CreateRigidBody();

    body->SetPosition(Vec3(0.3f, 1.0f, 0.0f));
    body->SetMass(1.0f);
    body->SetBoxInertia(Vec3(1.0f, 1.0f, 1.0f));
    body->SetLinearVelocity(Vec3(0.0f, 0.0f, 0.0f));

    Collider* collider = m_physicsWorld->CreateCollider();
    collider->SetHalfExtents(Vec3(0.5f, 0.5f, 0.5f));
    collider->SetRigidBody(body);

    collider->GetMaterial().SetRestitution(0.0f);
    collider->GetMaterial().SetFriction(0.0f);

    object->SetRigidBody(body);
    object->SetCollider(collider);

    object->GetTransform().position =
        body->GetPosition();

    AddObject(std::move(object));


    // Object 2 - Static Ground
    auto object2 = std::make_unique<Object>();
    object2->SetMesh(cubeMesh);

    RigidBody* body2 =
        m_physicsWorld->CreateRigidBody();

    body2->SetPosition(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    body2->SetMass(0.0f);

    body2->SetBoxInertia(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    Collider* collider2 =
        m_physicsWorld->CreateCollider();

    collider2->SetHalfExtents(
        Vec3(0.5f, 0.5f, 0.5f)
    );

    collider2->SetRigidBody(body2);

    collider2->GetMaterial().SetRestitution(0.0f);
    collider2->GetMaterial().SetFriction(0.0f);

    object2->SetRigidBody(body2);
    object2->SetCollider(collider2);

    object2->GetTransform().position =
        body2->GetPosition();

    AddObject(std::move(object2));

    return true;
}

void World::Update(
    float deltaTime,
    const InputManager& input
) {
    const Keyboard& keyboard = input.GetKeyboard();
    constexpr float cameraSpeed = 5.0f;

    Vec3 movement(0.0f, 0.0f, 0.0f);

    if (keyboard.IsDown(SDL_SCANCODE_W))
        movement.z += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_S))
        movement.z -= 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_D))
        movement.x += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_A))
        movement.x -= 1.0f;

    m_camera.MoveLocal(movement * cameraSpeed * deltaTime);

    for (auto& object : m_objects) {
        if (object == nullptr)
            continue;

        RigidBody* body = object->GetRigidBody();

        if (body == nullptr)
            continue;

        object->GetTransform().position = body->GetPosition();
        object->GetTransform().rotation = body->GetOrientation();
    }
}

Camera& World::GetCamera() {
    return m_camera;
}

const Camera& World::GetCamera() const {
    return m_camera;
}

const std::vector<std::unique_ptr<Object>>&
World::GetObjects() const {
    return m_objects;
}

void World::AddObject(std::unique_ptr<Object> object) {
    m_objects.push_back(std::move(object));
}

void World::Shutdown() {
    m_objects.clear();
    m_physicsWorld = nullptr;
}