#pragma once

#include <memory>
#include <vector>

#include "../Rendering/Camera.h"
#include "../Input/InputManager.h"
#include "../Core/Object.h"
#include "../Rendering/MeshManager.h"
#include "../Physics/PhysicsWorld.h"

class World {
public:
    World();
    ~World();

    bool Initialize(
        MeshManager& meshManager,
        PhysicsWorld& physicsWorld
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

    void Shutdown();

private:
    void TestRaycast() const;

private:
    Camera m_camera;
    std::vector<std::unique_ptr<Object>> m_objects;

    PhysicsWorld* m_physicsWorld;
};
