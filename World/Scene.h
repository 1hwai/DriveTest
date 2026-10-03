#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <memory>
#include <vector>

#include "../Rendering/Camera.h"
#include "../Input/Keyboard.h"
#include "../Input/VehicleInput.h"
#include "../Core/Object.h"
#include "../Rendering/MeshManager.h"
#include "../Physics/PhysicsWorld.h"
#include "TestTrack.h"
#include "../Vehicle/Car.h"
#include "SceneFactory.h"

enum class CameraMode {
    Free,
    Chase
};

class Scene {
public:
    Scene();
    ~Scene();

    bool Initialize(
        MeshManager& meshManager,
        PhysicsWorld& physicsWorld
    );

    void UpdatePhysics(
        float deltaTime,
        const VehicleInput& input
    );

    void Update(
        float deltaTime,
        const Keyboard& keyboard,
        const VehicleInput& input
    );

    Camera& GetCamera();
    const Camera& GetCamera() const;

    Car& GetCar();
    const Car& GetCar() const;

    void SetCameraMode(CameraMode mode);
    CameraMode GetCameraMode() const;

    const std::vector<std::unique_ptr<Object>>&
        GetObjects() const;

    void AddObject(std::unique_ptr<Object> object);

    Object* CreateBox(const BoxSettings& settings);
    Object* CreateSphere(const SphereSettings& settings);
    bool DestroyObject(Object* object);
    void ClearObjects();
    void ClearPersistentObjects();

    void Shutdown();

private:
    void ReloadVehicleConfigIfChanged();

    Camera m_camera;
    CameraMode m_cameraMode;
    std::vector<std::unique_ptr<Object>> m_objects;

    PhysicsWorld* m_physicsWorld;
    std::unique_ptr<SceneFactory> m_sceneFactory;
    std::unique_ptr<TestTrack> m_track;

    Car m_car;
    VehicleConfig m_vehicleConfig;
    std::string m_vehicleConfigPath;
    std::filesystem::file_time_type m_vehicleConfigLastWriteTime;
    float m_vehicleConfigCheckTimer;
    Object* m_carChassisObject;
    std::array<Object*, WheelCount> m_wheelObjects;

    struct VehicleVisualPart {
        Object* object;
        Vec3 localPosition;
        Vec3 scale;
    };
    std::vector<VehicleVisualPart> m_carBodyParts;
    float m_suspensionDebugTimer;
};