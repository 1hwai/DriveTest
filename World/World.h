#pragma once

#include <memory>
#include <vector>

#include "../Rendering/Camera.h"
#include "../Input/InputManager.h"
#include "../Core/Object.h"
#include "../Rendering/MeshManager.h"
#include "../Physics/PhysicsWorld.h"
#include "../Vehicle/Wheel.h"
#include "../Vehicle/Suspension.h"
#include "SceneFactory.h"

class World {
public:
    World();
    ~World();

    bool Initialize(
        MeshManager& meshManager,
        PhysicsWorld& physicsWorld
    );

    void UpdatePhysics(
        float deltaTime,
        const InputManager& input
    );

    void Update(
        float deltaTime,
        const InputManager& input
    );

    Camera& GetCamera();
    const Camera& GetCamera() const;

    const std::vector<std::unique_ptr<Object>>&
        GetObjects() const;

    void AddObject(std::unique_ptr<Object> object);

    Object* CreateBox(const BoxSettings& settings);
    Object* CreateSphere(const SphereSettings& settings);
    bool DestroyObject(Object* object);
    void ClearObjects();

    void Shutdown();

private:
    Camera m_camera;
    std::vector<std::unique_ptr<Object>> m_objects;

    PhysicsWorld* m_physicsWorld;
    std::unique_ptr<SceneFactory> m_sceneFactory;

    Wheel m_testWheel;
    Suspension m_testSuspension;
    Object* m_testChassisObject;
    Object* m_testWheelObject;
    float m_suspensionDebugTimer;
};
