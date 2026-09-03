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

    auto object = std::make_unique<Object>();

    Mesh* cubeMesh =
        meshManager.Get("cube");

    object->SetMesh(cubeMesh);

    RigidBody* body =
        m_physicsWorld->CreateRigidBody();

    body->SetPosition(
        Vec3(0.0f, 5.0f, 5.0f)
    );

    object->SetRigidBody(body);

    object->GetTransform().position =
        body->GetPosition();

    AddObject(std::move(object));

    return true;
}

void World::Update(
    float deltaTime,
    const InputManager& input
) {
    const Keyboard& keyboard =
        input.GetKeyboard();

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

    m_camera.MoveLocal(
        movement * cameraSpeed * deltaTime
    );

    for (auto& object : m_objects) {
        if (object == nullptr)
            continue;

        RigidBody* body =
            object->GetRigidBody();

        if (body == nullptr)
            continue;

        object->GetTransform().position =
            body->GetPosition();

        object->GetTransform().rotation =
            body->GetOrientation();
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

void World::AddObject(
    std::unique_ptr<Object> object
) {
    m_objects.push_back(std::move(object));
}

void World::Shutdown() {
    m_objects.clear();
    m_physicsWorld = nullptr;
}