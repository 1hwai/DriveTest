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

};