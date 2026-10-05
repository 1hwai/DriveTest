#include "Scene.h"
#include "TestTrack.h"
#include "../Physics/Terrain.h"

#include "../Core/Debug/Logger.h"

#include <cmath>
#include <string>
#include <iostream>

namespace {
    constexpr float CameraRotationSpeed = 1.5f;
    constexpr float CameraPitchLimit = 1.55334306f;
    constexpr float ChaseCameraDistance = 7.0f;
    constexpr float ChaseCameraHeight = 3.0f;
    constexpr float ChaseCameraLookHeight = 0.8f;
}

Scene::Scene()
    : m_cameraMode(CameraMode::Free),
    m_physicsWorld(nullptr),
    m_carChassisObject(nullptr),
    m_suspensionDebugTimer(0.0f) {
    m_wheelObjects.fill(nullptr);
}

Scene::~Scene() {
    Shutdown();
}

bool Scene::Initialize(
    MeshManager& meshManager,
    PhysicsWorld& physicsWorld
) {
    m_physicsWorld = &physicsWorld;

    const std::string vehicleConfigPath =
        std::string(DRIVETEST_PROJECT_ROOT) + "/Assets/Vehicles/TestCar/vehicle.ini";
    std::string configError;
    if (!m_vehicleConfig.Load(vehicleConfigPath, configError)) {
        Logger::Error(configError);
        return false;
    }
    if (!m_vehicleConfigWatcher.Initialize(vehicleConfigPath, configError)) {
        Logger::Error(configError);
        return false;
    }
    m_car.ApplyConfig(m_vehicleConfig);

    m_sceneFactory =
        std::make_unique<SceneFactory>(
            meshManager,
            physicsWorld
        );

    const std::string carModelPath = std::string(DRIVETEST_PROJECT_ROOT) + "/Assets/Vehicles/subaru_impreza.glb";
    if (!meshManager.LoadFromFile("impreza_body", carModelPath) ||
        !meshManager.LoadFromFile("impreza_wheel", carModelPath, true)) {
        Logger::Error("Failed to load Subaru Impreza body or wheel mesh: " + carModelPath);
        return false;
    }

    m_track = std::make_unique<TestTrack>();
    const std::string stagePath = std::string(DRIVETEST_PROJECT_ROOT) + "/Stages/TestTrack.stage";
    if (!m_track->Initialize(meshManager, *m_sceneFactory, stagePath))
        return false;

    for (auto& object : m_track->TakeObjects())
        AddObject(std::move(object));

    auto chassis = m_sceneFactory->CreateBox({
        "CarChassis",
        Vec3(
            0.0f,
            m_track->GetTerrain().GetHeight(0.0f, 0.0f) + m_vehicleConfig.spawnClearance,
            0.0f
        ),
        m_vehicleConfig.colliderHalfExtents,
        m_vehicleConfig.mass,
        0.0f,
        0.5f
    });

    if (!chassis)
        return false;

    chassis->SetScenePersistent(false);
    chassis->SetColor(Vec3(0.72f, 0.08f, 0.06f));
    chassis->SetMesh(nullptr);
    m_carChassisObject = chassis.get();
    m_car.SetChassis(chassis->GetRigidBody());
    AddObject(std::move(chassis));

    for (size_t i = 0; i < WheelCount; ++i) {
        auto wheel = std::make_unique<Object>();
        wheel->SetName("CarWheel");
        wheel->SetScenePersistent(false);
        wheel->SetColor(Vec3(1.0f, 1.0f, 1.0f));
        wheel->SetMesh(meshManager.Get("impreza_wheel"));
        AddObject(std::move(wheel));
        m_wheelObjects[i] = m_objects.back().get();
    }


    auto carBody = std::make_unique<Object>();
    carBody->SetName("1998 Impreza 22B Body");
    carBody->SetScenePersistent(false);
    carBody->SetColor(Vec3(1.0f, 1.0f, 1.0f));
    carBody->SetMesh(meshManager.Get("impreza_body"));
    AddObject(std::move(carBody));
    m_carBodyParts.push_back({m_objects.back().get(), Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f)});
    



    return true;
}

void Scene::ReloadVehicleConfigIfChanged(float deltaTime) {
    std::string error;
    const VehicleConfigReloadResult result =
        m_vehicleConfigWatcher.Poll(deltaTime, m_vehicleConfig, error);

    if (result == VehicleConfigReloadResult::Unchanged)
        return;

    if (result == VehicleConfigReloadResult::Error) {
        Logger::Error("Vehicle config reload rejected: " + error);
        return;
    }

    m_car.ApplyConfig(m_vehicleConfig);

    if (m_carChassisObject != nullptr) {
        RigidBody* chassis = m_carChassisObject->GetRigidBody();
        Collider* collider = m_carChassisObject->GetCollider();
        if (chassis != nullptr) {
            chassis->SetMass(m_vehicleConfig.mass);
            chassis->SetBoxInertia(m_vehicleConfig.colliderHalfExtents * 2.0f);
        }
        if (collider != nullptr)
            collider->SetHalfExtents(m_vehicleConfig.colliderHalfExtents);
    }

    Logger::Info("Vehicle config hot-reloaded: " + m_vehicleConfigWatcher.GetPath());
}

void Scene::UpdatePhysics(
    float deltaTime,
    const VehicleInput& input
) {
    if (!m_physicsWorld)
        return;

    ReloadVehicleConfigIfChanged(deltaTime);

    if (m_cameraMode == CameraMode::Free) {
        m_car.SetInput(0.0f, 0.0f, 0.0f, 0.0f);
        m_car.UpdatePhysics(*m_physicsWorld, deltaTime);
        return;
    }

    m_car.SetInput(
        input.throttle,
        input.brake,
        input.steering,
        input.clutch
    );

    m_car.UpdatePhysics(
        *m_physicsWorld,
        deltaTime
    );

    m_suspensionDebugTimer += deltaTime;

    if (m_suspensionDebugTimer >= 0.25f) {
        m_suspensionDebugTimer = 0.0f;
        const Wheel& frontLeft = m_car.GetWheel(WheelIndex::FrontLeft);
        const Wheel& frontRight = m_car.GetWheel(WheelIndex::FrontRight);
        const Wheel& rearLeft = m_car.GetWheel(WheelIndex::RearLeft);
        const Wheel& rearRight = m_car.GetWheel(WheelIndex::RearRight);

        Logger::Debug(
            "[Suspension] " +
            std::string("FL=") + std::to_string(frontLeft.GetCompression()) +
            " FR=" + std::to_string(frontRight.GetCompression()) +
            " RL=" + std::to_string(rearLeft.GetCompression()) +
            " RR=" + std::to_string(rearRight.GetCompression())
        );

        Logger::Debug(
            "[Wheel] " +
            std::string("FL=") + std::to_string(frontLeft.GetAngularVelocity()) +
            " FR=" + std::to_string(frontRight.GetAngularVelocity()) +
            " RL=" + std::to_string(rearLeft.GetAngularVelocity()) +
            " RR=" + std::to_string(rearRight.GetAngularVelocity())
        );

        Logger::Debug(
            "[Powertrain] gear=" +
            std::to_string(m_car.GetTransmission().GetGear()) +
            " rpm=" + std::to_string(m_car.GetEngine().GetRPM()) +
            " torque=" + std::to_string(m_car.GetEngine().GetTorque()) +
            " clutch=" + std::to_string(m_car.GetClutch()) +
            " running=" + std::to_string(m_car.GetEngine().IsRunning())
        );
    }
}

void Scene::Update(
    float deltaTime,
    const Keyboard& keyboard,
    const VehicleInput& input
) {
    if (input.shiftUp)
        m_car.GetTransmission().ShiftUp();

    if (input.shiftDown)
        m_car.GetTransmission().ShiftDown();

    if (m_cameraMode == CameraMode::Free) {
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

        for (const VehicleVisualPart& part : m_carBodyParts) {
            if (!part.object)
                continue;
            Transform& transform = part.object->GetTransform();
            transform.position = m_carChassisObject->GetTransform().position +
                chassisRotation * part.localPosition;
            transform.rotation = chassisRotation;
            transform.scale = part.scale;
        }

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

            // Use the same steering-angle sign convention as the tire model:
            // positive = right, negative = left.
            const Quaternion wheelSteering =
                Quaternion::FromAxisAngle(
                    Vec3(0.0f, 1.0f, 0.0f),
                    m_car.GetWheel(
                        static_cast<WheelIndex>(i)
                    ).GetSteeringAngle()
                );

            const Quaternion wheelSpin =
                Quaternion::FromAxisAngle(
                    Vec3(1.0f, 0.0f, 0.0f),
                    wheelAngle
                );

            m_wheelObjects[i]->GetTransform().rotation =
                chassisRotation *
                wheelSteering *
                wheelSpin;

            const float diameter =
                m_car.GetWheel(
                    static_cast<WheelIndex>(i)
                ).GetRadius() * 2.0f;

            m_wheelObjects[i]->GetTransform().scale =
                Vec3(diameter, diameter, diameter);
        }
    }
}

Camera& Scene::GetCamera() {
    return m_camera;
}

const Camera& Scene::GetCamera() const {
    return m_camera;
}

Car& Scene::GetCar() {
    return m_car;
}

const Car& Scene::GetCar() const {
    return m_car;
}

void Scene::SetCameraMode(CameraMode mode) {
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

CameraMode Scene::GetCameraMode() const {
    return m_cameraMode;
}

const std::vector<std::unique_ptr<Object>>&
Scene::GetObjects() const {
    return m_objects;
}

void Scene::AddObject(std::unique_ptr<Object> object) {
    m_objects.push_back(std::move(object));
}

Object* Scene::CreateBox(const BoxSettings& settings) {
    if (!m_sceneFactory)
        return nullptr;

    auto object = m_sceneFactory->CreateBox(settings);

    if (!object)
        return nullptr;

    Object* result = object.get();
    AddObject(std::move(object));
    return result;
}

Object* Scene::CreateSphere(const SphereSettings& settings) {
    if (!m_sceneFactory)
        return nullptr;

    auto object = m_sceneFactory->CreateSphere(settings);

    if (!object)
        return nullptr;

    Object* result = object.get();
    AddObject(std::move(object));
    return result;
}

void Scene::ClearObjects() {
    if (!m_physicsWorld) {
        m_objects.clear();
        m_carBodyParts.clear();
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
    m_carBodyParts.clear();
    m_car.SetChassis(nullptr);
    m_carChassisObject = nullptr;
    m_wheelObjects.fill(nullptr);
}

bool Scene::DestroyObject(Object* object) {
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

        for (auto partIt = m_carBodyParts.begin(); partIt != m_carBodyParts.end();) {
            if (partIt->object == object)
                partIt = m_carBodyParts.erase(partIt);
            else
                ++partIt;
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

void Scene::ClearPersistentObjects() {
    for (auto it = m_objects.begin(); it != m_objects.end();) {
        Object* object = it->get();

        if (!object || !object->IsScenePersistent()) {
            ++it;
            continue;
        }

        if (m_physicsWorld) {
            if (Collider* collider = object->GetCollider())
                m_physicsWorld->DestroyCollider(collider);

            if (RigidBody* body = object->GetRigidBody())
                m_physicsWorld->DestroyRigidBody(body);
        }

        it = m_objects.erase(it);
    }
}

void Scene::Shutdown() {
    ClearObjects();
    m_track.reset();
    m_sceneFactory.reset();
    m_physicsWorld = nullptr;
}
