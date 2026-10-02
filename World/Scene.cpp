#include "Scene.h"

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

    VehicleConfig vehicleConfig;
    std::string configError;
    const std::string configPath =
        std::string(DRIVETEST_PROJECT_ROOT) + "/Assets/Vehicles/TestCar/vehicle.ini";
    if (!vehicleConfig.Load(configPath, configError)) {
        Logger::Error(configError);
        return false;
    }
    m_car.ApplyConfig(vehicleConfig);

    m_sceneFactory =
        std::make_unique<SceneFactory>(
            meshManager,
            physicsWorld
        );

    m_terrain =
        std::make_unique<Terrain>(
            513,
            5000.0f,
            4.0f,
            0.0025f,
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

    ground->SetScenePersistent(false);
    ground->SetColor(Vec3(0.24f, 0.48f, 0.19f));
    AddObject(std::move(ground));

    auto chassis = m_sceneFactory->CreateBox({
        "CarChassis",
        Vec3(
            0.0f,
            m_terrain->GetHeight(0.0f, 0.0f) + vehicleConfig.spawnClearance,
            0.0f
        ),
        vehicleConfig.colliderHalfExtents,
        vehicleConfig.mass,
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
        wheel->SetColor(Vec3(0.07f, 0.07f, 0.07f));
        wheel->SetMesh(meshManager.Get("wheel"));
        AddObject(std::move(wheel));
        m_wheelObjects[i] = m_objects.back().get();
    }

    const auto addBodyPart = [&](const char* name, const Vec3& position, const Vec3& size, const Vec3& color) {
        auto part = std::make_unique<Object>();
        part->SetName(name);
        part->SetScenePersistent(false);
        part->SetColor(color);
        part->SetMesh(meshManager.Get("cube"));
        AddObject(std::move(part));
        m_carBodyParts.push_back({m_objects.back().get(), position, size});
    };

    const Vec3 bodyColor(0.72f, 0.08f, 0.06f);
    const Vec3 glassColor(0.055f, 0.09f, 0.12f);
    const Vec3 trimColor(0.12f, 0.12f, 0.12f);
    addBodyPart("CarHood", Vec3(0.0f, 0.12f, 1.67f), Vec3(1.68f, 0.18f, 0.92f), bodyColor);
    addBodyPart("CarRearDeck", Vec3(0.0f, 0.12f, -1.67f), Vec3(1.68f, 0.18f, 0.92f), bodyColor);
    addBodyPart("CarFloor", Vec3(0.0f, -0.13f, 0.0f), Vec3(1.48f, 0.14f, 1.12f), bodyColor);
    addBodyPart("CarCabin", Vec3(0.0f, 0.25f, 0.0f), Vec3(1.43f, 0.42f, 1.30f), bodyColor);
    addBodyPart("CarRoof", Vec3(0.0f, 0.49f, -0.02f), Vec3(1.20f, 0.12f, 0.78f), bodyColor);
    addBodyPart("CarFrontGlass", Vec3(0.0f, 0.39f, 0.665f), Vec3(1.10f, 0.20f, 0.025f), glassColor);
    addBodyPart("CarRearGlass", Vec3(0.0f, 0.39f, -0.665f), Vec3(1.10f, 0.20f, 0.025f), glassColor);
    addBodyPart("CarLeftWindow", Vec3(-0.72f, 0.31f, 0.0f), Vec3(0.025f, 0.22f, 0.82f), glassColor);
    addBodyPart("CarRightWindow", Vec3(0.72f, 0.31f, 0.0f), Vec3(0.025f, 0.22f, 0.82f), glassColor);
    addBodyPart("CarFrontBumper", Vec3(0.0f, -0.14f, 2.13f), Vec3(1.76f, 0.13f, 0.14f), trimColor);
    addBodyPart("CarRearBumper", Vec3(0.0f, -0.14f, -2.13f), Vec3(1.76f, 0.13f, 0.14f), trimColor);
    addBodyPart("CarLeftSill", Vec3(-0.79f, -0.15f, 0.0f), Vec3(0.12f, 0.16f, 1.05f), bodyColor);
    addBodyPart("CarRightSill", Vec3(0.79f, -0.15f, 0.0f), Vec3(0.12f, 0.16f, 1.05f), bodyColor);
    const float frontZ = 0.5f * (vehicleConfig.wheelPositions[0].z + vehicleConfig.wheelPositions[1].z);
    const float rearZ = 0.5f * (vehicleConfig.wheelPositions[2].z + vehicleConfig.wheelPositions[3].z);
    const float leftX = 0.5f * (vehicleConfig.wheelPositions[0].x + vehicleConfig.wheelPositions[2].x);
    const float rightX = 0.5f * (vehicleConfig.wheelPositions[1].x + vehicleConfig.wheelPositions[3].x);
    addBodyPart("CarLeftFenderFront", Vec3(leftX, 0.16f, frontZ), Vec3(0.18f, 0.12f, 0.62f), bodyColor);
    addBodyPart("CarRightFenderFront", Vec3(rightX, 0.16f, frontZ), Vec3(0.18f, 0.12f, 0.62f), bodyColor);
    addBodyPart("CarLeftFenderRear", Vec3(leftX, 0.16f, rearZ), Vec3(0.18f, 0.12f, 0.62f), bodyColor);
    addBodyPart("CarRightFenderRear", Vec3(rightX, 0.16f, rearZ), Vec3(0.18f, 0.12f, 0.62f), bodyColor);

    return true;
}

void Scene::UpdatePhysics(
    float deltaTime,
    const VehicleInput& input
) {
    if (!m_physicsWorld)
        return;

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
                    -wheelAngle
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
    m_sceneFactory.reset();
    m_physicsWorld = nullptr;
}
