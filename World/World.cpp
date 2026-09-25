#include "World.h"

#include "../Core/Debug/Logger.h"

#include <cmath>
#include <string>

namespace {
    constexpr float CameraRotationSpeed = 1.5f;
    constexpr float CameraPitchLimit = 1.55334306f;
    constexpr float ChaseCameraDistance = 7.0f;
    constexpr float ChaseCameraHeight = 3.0f;
    constexpr float ChaseCameraLookHeight = 0.8f;
}

World::World()
    : m_cameraMode(CameraMode::Free),
    m_physicsWorld(nullptr),
    m_carChassisObject(nullptr),
    m_suspensionDebugTimer(0.0f) {
    m_wheelObjects.fill(nullptr);
}

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

    m_terrain =
        std::make_unique<Terrain>(
            65,
            100.0f,
            0.0f,
            0.035f,
            5,
            1337
        );

    if (!meshManager.CreateTerrain(
        "terrain",
        *m_terrain
    ))
        return false;

    if (!meshManager.CreateWheel("wheel"))
        return false;

    auto ground =
        m_sceneFactory->CreateTerrain({
            "Terrain",
            0.0f,
            0.5f
        }, *m_terrain);

    if (!ground)
        return false;

    AddObject(std::move(ground));

    auto chassis = m_sceneFactory->CreateBox({
        "CarChassis",
        Vec3(0.0f, 5.0f, 0.0f),
        Vec3(1.0f, 0.5f, 1.5f),
        1200.0f,
        0.0f,
        0.5f
    });

    if (!chassis)
        return false;

    m_carChassisObject = chassis.get();
    m_car.SetChassis(chassis->GetRigidBody());
    AddObject(std::move(chassis));

    const Vec3 wheelPositions[WheelCount] = {
        Vec3(-0.9f, -0.2f, 1.1f),
        Vec3(0.9f, -0.2f, 1.1f),
        Vec3(-0.9f, -0.2f, -1.1f),
        Vec3(0.9f, -0.2f, -1.1f)
    };

    for (size_t i = 0; i < WheelCount; ++i) {
        const WheelIndex index =
            static_cast<WheelIndex>(i);

        m_car.GetWheel(index).SetLocalPosition(
            wheelPositions[i]
        );

        m_car.GetWheel(index).SetRadius(0.5f);

        m_car.GetSuspension(index).SetRestLength(0.8f);
        m_car.GetSuspension(index).SetMaxLength(1.0f);
        m_car.GetSuspension(index).SetSpringRate(30000.0f);
        m_car.GetSuspension(index).SetDamperRate(4500.0f);

        auto wheel = std::make_unique<Object>();
        wheel->SetName("CarWheel");
        wheel->SetMesh(meshManager.Get("wheel"));
        AddObject(std::move(wheel));

        m_wheelObjects[i] = m_objects.back().get();
    }

    return true;
}

void World::UpdatePhysics(
    float deltaTime,
    const InputManager& input
) {
    (void)input;

    m_car.UpdatePhysics(
        *m_physicsWorld,
        deltaTime
    );

    m_suspensionDebugTimer += deltaTime;

    if (m_suspensionDebugTimer >= 0.25f) {
        m_suspensionDebugTimer = 0.0f;

        const Wheel& frontLeft =
            m_car.GetWheel(WheelIndex::FrontLeft);

        const Wheel& frontRight =
            m_car.GetWheel(WheelIndex::FrontRight);

        const Wheel& rearLeft =
            m_car.GetWheel(WheelIndex::RearLeft);

        const Wheel& rearRight =
            m_car.GetWheel(WheelIndex::RearRight);

        Logger::Debug(
            "[Suspension] " +
            std::string("FL=") +
            std::to_string(frontLeft.GetCompression()) +
            " FR=" +
            std::to_string(frontRight.GetCompression()) +
            " RL=" +
            std::to_string(rearLeft.GetCompression()) +
            " RR=" +
            std::to_string(rearRight.GetCompression())
        );

        Logger::Debug(
            "[Wheel] " +
            std::string("FL=") +
            std::to_string(frontLeft.GetAngularVelocity()) +
            " FR=" +
            std::to_string(frontRight.GetAngularVelocity()) +
            " RL=" +
            std::to_string(rearLeft.GetAngularVelocity()) +
            " RR=" +
            std::to_string(rearRight.GetAngularVelocity())
        );
    }
}

void World::Update(
    float deltaTime,
    const InputManager& input
) {
    if (m_cameraMode == CameraMode::Free) {
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

        m_camera.MoveLocal(
            movement * cameraSpeed * deltaTime
        );

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
    }
    else if (m_carChassisObject != nullptr) {
        const Transform& chassisTransform =
            m_carChassisObject->GetTransform();

        Vec3 forward =
            chassisTransform.rotation *
            Vec3(0.0f, 0.0f, 1.0f);

        forward.y = 0.0f;

        if (forward.LengthSquared() > 0.000001f)
            forward = forward.Normalized();
        else
            forward = Vec3(0.0f, 0.0f, 1.0f);

        const Vec3 cameraPosition =
            chassisTransform.position -
            forward * ChaseCameraDistance +
            Vec3(0.0f, ChaseCameraHeight, 0.0f);

        const Vec3 target =
            chassisTransform.position +
            Vec3(0.0f, ChaseCameraLookHeight, 0.0f);

        const Vec3 direction =
            (target - cameraPosition).Normalized();

        const float yaw =
            std::atan2(
                -direction.x,
                -direction.z
            );

        const float pitch =
            std::asin(direction.y);

        m_camera.SetPosition(cameraPosition);
        m_camera.SetRotation(
            Vec3(pitch, yaw, 0.0f)
        );
    }

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
                object->GetTransform().scale =
                    Vec3(diameter, diameter, diameter);
            }
            else {
                object->GetTransform().scale =
                    collider->GetHalfExtents() * 2.0f;
            }
        }
    }

    if (m_carChassisObject) {
        const Quaternion& chassisRotation =
            m_carChassisObject->GetTransform().rotation;

        for (size_t i = 0; i < WheelCount; ++i) {
            if (!m_wheelObjects[i])
                continue;

            m_wheelObjects[i]->GetTransform().position =
                m_car.GetWheel(
                    static_cast<WheelIndex>(i)
                ).GetWorldPosition();

            const float wheelAngle =
                m_car.GetWheel(
                    static_cast<WheelIndex>(i)
                ).GetRotationAngle();

            const Quaternion wheelSpin =
                Quaternion::FromAxisAngle(
                    Vec3(1.0f, 0.0f, 0.0f),
                    -wheelAngle
                );

            m_wheelObjects[i]->GetTransform().rotation =
                chassisRotation * wheelSpin;

            const float diameter =
                m_car.GetWheel(
                    static_cast<WheelIndex>(i)
                ).GetRadius() * 2.0f;

            m_wheelObjects[i]->GetTransform().scale =
                Vec3(diameter, diameter, diameter);
        }
    }
}

Camera& World::GetCamera() {
    return m_camera;
}

const Camera& World::GetCamera() const {
    return m_camera;
}

void World::SetCameraMode(CameraMode mode) {
    if (m_cameraMode == mode)
        return;

    m_cameraMode = mode;

    if (m_cameraMode == CameraMode::Free) {
        if (m_carChassisObject != nullptr) {
            m_camera.SetPosition(
                m_carChassisObject->GetTransform().position +
                Vec3(0.0f, 3.0f, -7.0f)
            );
            m_camera.SetRotation(
                Vec3(-0.2f, 0.0f, 0.0f)
            );
        }
    }
}

CameraMode World::GetCameraMode() const {
    return m_cameraMode;
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
        m_car.SetChassis(nullptr);
        m_carChassisObject = nullptr;
        m_wheelObjects.fill(nullptr);
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
    m_car.SetChassis(nullptr);
    m_carChassisObject = nullptr;
    m_wheelObjects.fill(nullptr);
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

        if (object == m_carChassisObject) {
            m_car.SetChassis(nullptr);
            m_carChassisObject = nullptr;
        }

        for (size_t i = 0; i < WheelCount; ++i) {
            if (object == m_wheelObjects[i])
                m_wheelObjects[i] = nullptr;
        }

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
