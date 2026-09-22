#include "World.h"

namespace {
    constexpr float CameraRotationSpeed = 1.5f;
    constexpr float CameraPitchLimit = 1.55334306f;
}

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

    m_sceneFactory =
        std::make_unique<SceneFactory>(
            meshManager,
            physicsWorld
        );

    if (!meshManager.CreateSphere("sphere"))
        return false;

    auto ground = m_sceneFactory->CreateBox({
        "Ground",
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(1.0f, 0.5f, 1.0f),
        0.0f,
        0.0f,
        0.0f
    });

    if (!ground)
        return false;

    AddObject(std::move(ground));

    auto sphere = m_sceneFactory->CreateSphere({
        "Sphere",
        Vec3(0.0f, 2.5f, 0.0f),
        0.5f,
        1.0f,
        0.9f,
        0.4f
    });

    if (!sphere)
        return false;

    AddObject(std::move(sphere));

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

    if (keyboard.IsDown(SDL_SCANCODE_E))
        movement.y += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_Q))
        movement.y -= 1.0f;

    m_camera.MoveLocal(movement * cameraSpeed * deltaTime);

    Vec3 rotation(0.0f, 0.0f, 0.0f);

    if (keyboard.IsDown(SDL_SCANCODE_LEFT))
        rotation.y -= 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_RIGHT))
        rotation.y += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_UP))
        rotation.x += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_DOWN))
        rotation.x -= 1.0f;

    m_camera.Rotate(
        rotation * CameraRotationSpeed * deltaTime
    );

    Vec3 cameraRotation = m_camera.GetRotation();

    if (cameraRotation.x > CameraPitchLimit)
        cameraRotation.x = CameraPitchLimit;

    if (cameraRotation.x < -CameraPitchLimit)
        cameraRotation.x = -CameraPitchLimit;

    cameraRotation.z = 0.0f;
    m_camera.SetRotation(cameraRotation);

    for (auto& object : m_objects) {
        if (object == nullptr)
            continue;

        RigidBody* body = object->GetRigidBody();

        if (body == nullptr)
            continue;

        object->GetTransform().position = body->GetPosition();
        object->GetTransform().rotation = body->GetOrientation();

        const Collider* collider = object->GetCollider();

        if (collider != nullptr) {
            if (collider->GetShape() == ColliderShape::Sphere) {
                const float diameter = collider->GetRadius() * 2.0f;
                object->GetTransform().scale = Vec3(diameter, diameter, diameter);
            }
            else {
                object->GetTransform().scale = collider->GetHalfExtents() * 2.0f;
            }
        }
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

Object* World::CreateBox(const BoxSettings& settings) {
    if (!m_sceneFactory)
        return nullptr;

    auto object = m_sceneFactory->CreateBox(settings);

    if (!object)
        return nullptr;

    Object* result = object.get();
    AddObject(std::move(object));
    return result;
}

Object* World::CreateSphere(const SphereSettings& settings) {
    if (!m_sceneFactory)
        return nullptr;

    auto object = m_sceneFactory->CreateSphere(settings);

    if (!object)
        return nullptr;

    Object* result = object.get();
    AddObject(std::move(object));
    return result;
}

void World::ClearObjects() {
    if (!m_physicsWorld) {
        m_objects.clear();
        return;
    }

    for (auto& object : m_objects) {
        if (!object)
            continue;

        if (Collider* collider = object->GetCollider())
            m_physicsWorld->DestroyCollider(collider);

        if (RigidBody* body = object->GetRigidBody())
            m_physicsWorld->DestroyRigidBody(body);
    }

    m_objects.clear();
}

bool World::DestroyObject(Object* object) {
    if (!object || !m_physicsWorld)
        return false;

    Collider* collider = object->GetCollider();
    RigidBody* body = object->GetRigidBody();

    if (collider)
        m_physicsWorld->DestroyCollider(collider);

    if (body)
        m_physicsWorld->DestroyRigidBody(body);

    for (auto it = m_objects.begin();
        it != m_objects.end();
        ++it) {

        if (it->get() != object)
            continue;

        m_objects.erase(it);
        return true;
    }

    return false;
}

void World::Shutdown() {
    ClearObjects();
    m_sceneFactory.reset();
    m_physicsWorld = nullptr;
}