#pragma once

#include "Time.h"
#include "../Input/InputManager.h"
#include "../Rendering/Renderer.h"
#include "../World/World.h"
#include "../Rendering/MeshManager.h"
#include "../Core/Debug/Logger.h"
#include "../Physics/PhysicsWorld.h"

class Application {
public:
    Application();
    ~Application();

    bool Initialize();
    int Run();

private:
    void ProcessEvents();
    void Update();
    void Render();
    void Shutdown();

private:
    bool m_running;

    Time m_time;
    InputManager m_input;

    Renderer m_renderer;
    MeshManager m_meshManager;
    World m_world;
    PhysicsWorld m_physicsWorld;

    float m_physicsAccumulator;
    float m_fpsTimer;
    int m_frameCount;
    int m_physicsStepCount;

    static constexpr float PhysicsFixedDeltaTime = 1.0f / 120.0f;
    static constexpr int MaxPhysicsStepsPerFrame = 8;
};