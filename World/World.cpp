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
    // m_physicsWorld->SetGravity(Vec3(0.0f, 0.0f, 0.0f));

    Mesh* cubeMesh = meshManager.Get("cube");

    // Object 1 - Dynamic
    //auto object1 = std::make_unique<Object>();
    //object1->SetMesh(cubeMesh);

    //RigidBody* body1 =
    //    m_physicsWorld->CreateRigidBody();

    //body1->SetPosition(
    //    Vec3(0.0f, 3.0f, 0.0f)
    //);

    //body1->SetMass(1.0f);
    //body1->SetBoxInertia(
    //    Vec3(1.0f, 1.0f, 1.0f)
    //);

    //Collider* collider1 =
    //    m_physicsWorld->CreateCollider();

    //collider1->SetHalfExtents(
    //    Vec3(0.5f, 0.5f, 0.5f)
    //);

    //collider1->SetRigidBody(body1);

    //collider1->GetMaterial().SetRestitution(0.0f);
    //collider1->GetMaterial().SetFriction(0.0f);

    //object1->SetRigidBody(body1);
    //object1->SetCollider(collider1);

    //object1->GetTransform().position =
    //    body1->GetPosition();

    //AddObject(std::move(object1));


    // Object 2 - Dynamic
    auto object2 = std::make_unique<Object>();
    object2->SetMesh(cubeMesh);

    RigidBody* body2 =
        m_physicsWorld->CreateRigidBody();

    body2->SetPosition(
        Vec3(0.7f, 1.0f, 0.0f)
    );

    body2->SetMass(1.0f);
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


    // Object 3 - Static Ground
    auto object3 = std::make_unique<Object>();
    object3->SetMesh(cubeMesh);

    RigidBody* body3 =
        m_physicsWorld->CreateRigidBody();

    body3->SetPosition(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    body3->SetMass(0.0f);

    Collider* collider3 =
        m_physicsWorld->CreateCollider();

    collider3->SetHalfExtents(
        Vec3(0.5f, 0.5f, 0.5f)
    );

    collider3->SetRigidBody(body3);

    collider3->GetMaterial().SetRestitution(0.0f);
    collider3->GetMaterial().SetFriction(0.0f);

    object3->SetRigidBody(body3);
    object3->SetCollider(collider3);

    object3->GetTransform().position =
        body3->GetPosition();

    AddObject(std::move(object3));

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