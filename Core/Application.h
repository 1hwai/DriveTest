#pragma once

#include "Time.h"
#include "SimulationController.h"
#include "Debug/Performance.h"
#include "../Input/InputManager.h"
#include "../Rendering/Renderer.h"
#include "../World/World.h"
#include "../Rendering/MeshManager.h"
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
    SimulationController m_simulation;
    Performance m_performance;
    InputManager m_input;

    Renderer m_renderer;
    MeshManager m_meshManager;
    World m_world;
    PhysicsWorld m_physicsWorld;

    float m_physicsAccumulator;

    static constexpr float PhysicsFixedDeltaTime = 1.0f / 120.0f;
    static constexpr int MaxPhysicsStepsPerFrame = 8;
};