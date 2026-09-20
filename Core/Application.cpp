#include "Application.h"

Application::Application()
    : m_running(false),
    m_physicsAccumulator(0.0f) {}

Application::~Application() {
    Shutdown();
}

bool Application::Initialize() {
    if (!m_renderer.Initialize())
        return false;

    if (!m_meshManager.CreateCube("cube"))
        return false;

    if (!m_world.Initialize(
        m_meshManager,
        m_physicsWorld
    ))
        return false;

    m_time.Reset();
    m_physicsAccumulator = 0.0f;
    m_performance.Reset();

    m_running = true;

    return true;
}

int Application::Run() {
    while (m_running) {
        m_time.Update();

        ProcessEvents();

        Update();
        Render();
    }

    return 0;
}

void Application::Render() {
    m_renderer.Render(m_world);
}

void Application::Update() {
    const float deltaTime =
        m_time.GetDeltaTime();

    m_physicsAccumulator += deltaTime;

    int physicsStepsThisFrame = 0;

    while (m_physicsAccumulator >= PhysicsFixedDeltaTime &&
        physicsStepsThisFrame < MaxPhysicsStepsPerFrame) {

        m_physicsWorld.Step(
            PhysicsFixedDeltaTime
        );

        m_physicsAccumulator -=
            PhysicsFixedDeltaTime;

        ++physicsStepsThisFrame;
        m_performance.RecordPhysicsStep();
    }

    if (physicsStepsThisFrame >= MaxPhysicsStepsPerFrame &&
        m_physicsAccumulator >= PhysicsFixedDeltaTime) {

        m_physicsAccumulator = 0.0f;
    }

    m_world.Update(
        deltaTime,
        m_input
    );

    m_performance.UpdateFrame(deltaTime);
}

void Application::ProcessEvents() {
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
        m_input.ProcessEvent(event);
    }

    if (m_input.QuitRequested()) {
        m_running = false;
    }
}

void Application::Shutdown() {
    if (!m_running)
        return;

    m_world.Shutdown();
    m_physicsWorld.Clear();
    m_meshManager.Clear();
    m_renderer.Shutdown();

    m_running = false;
}
