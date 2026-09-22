#include "World.h"

#include "../Core/Debug/Logger.h"

namespace {
    constexpr float CameraRotationSpeed = 1.5f;
    constexpr float CameraPitchLimit = 1.55334306f;
    constexpr float WheelInputSpeed = 0.5f;
}

World::World()
    : m_physicsWorld(nullptr),
    m_testChassisObject(nullptr),
    m_testWheelObject(nullptr),
    m_suspensionDebugTimer(0.0f) {}

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

    if (!meshManager.CreateWheel("wheel"))
        return false;

    auto ground = m_sceneFactory->CreateBox({
        "Ground",
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(5.0f, 0.5f, 5.0f),
        0.0f,
        0.0f,
        0.0f
    });

    if (!ground)
        return false;

    AddObject(std::move(ground));

    auto chassis = m_sceneFactory->CreateBox({
        "SuspensionTestChassis",
        Vec3(0.0f, 2.0f, 0.0f),
        Vec3(1.0f, 0.5f, 1.5f),
        1200.0f,
        0.0f,
        0.5f
    });

    if (!chassis)
        return false;

    m_testChassisObject = chassis.get();
    AddObject(std::move(chassis));

    auto wheel = std::make_unique<Object>();
    wheel->SetName("TestWheel");
    wheel->SetMesh(meshManager.Get("wheel"));
    AddObject(std::move(wheel));
    m_testWheelObject = m_objects.back().get();

    m_testWheel.SetLocalPosition(
        Vec3(0.0f, -0.2f, 0.0f)
    );
    m_testWheel.SetRadius(0.5f);

    m_testSuspension.SetRestLength(0.8f);
    m_testSuspension.SetMaxLength(1.0f);
    m_testSuspension.SetSpringRate(30000.0f);
    m_testSuspension.SetDamperRate(4500.0f);

    return true;
}

void World::UpdatePhysics(
    float deltaTime,
    const InputManager& input
) {
    if (!m_testChassisObject)
        return;

    RigidBody* body =
        m_testChassisObject->GetRigidBody();

    if (!body)
        return;

    const Keyboard& keyboard =
        input.GetKeyboard();

    Vec3 wheelPosition =
        m_testWheel.GetLocalPosition();

    if (keyboard.IsDown(SDL_SCANCODE_R))
        wheelPosition.y += WheelInputSpeed * deltaTime;

    if (keyboard.IsDown(SDL_SCANCODE_F))
        wheelPosition.y -= WheelInputSpeed * deltaTime;

    if (wheelPosition.y > 0.4f)
        wheelPosition.y = 0.4f;

    if (wheelPosition.y < -0.8f)
        wheelPosition.y = -0.8f;

    m_testWheel.SetLocalPosition(wheelPosition);

    m_testWheel.Update(
        *body,
        *m_physicsWorld,
        m_testSuspension,
        deltaTime
    );

    m_suspensionDebugTimer += deltaTime;

    if (m_suspensionDebugTimer >= 0.25f) {
        m_suspensionDebugTimer = 0.0f;

        Logger::Debug(
            "[Suspension] grounded=" +
            std::to_string(m_testWheel.IsGrounded()) +
            " length=" +
            std::to_string(m_testWheel.GetSuspensionLength()) +
            " compression=" +
            std::to_string(m_testWheel.GetCompression()) +
            " force=" +
            std::to_string(m_testWheel.GetForce()) +
            " mountY=" +
            std::to_string(wheelPosition.y)
        );
    }
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
        rotation.y += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_RIGHT))
        rotation.y -= 1.0f;

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

    if (m_testWheelObject && m_testChassisObject) {
        m_testWheelObject->GetTransform().position =
            m_testWheel.GetWorldPosition();

        m_testWheelObject->GetTransform().rotation =
            m_testChassisObject->GetTransform().rotation;

        const float diameter =
            m_testWheel.GetRadius() * 2.0f;

        m_testWheelObject->GetTransform().scale =
            Vec3(diameter, diameter, diameter);
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
        m_testChassisObject = nullptr;
        m_testWheelObject = nullptr;
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
    m_testChassisObject = nullptr;
    m_testWheelObject = nullptr;
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

        if (object == m_testChassisObject)
            m_testChassisObject = nullptr;

        if (object == m_testWheelObject)
            m_testWheelObject = nullptr;

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
