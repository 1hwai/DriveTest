#include "Application.h"

Application::Application()
    : m_running(false) {}

Application::~Application() {
    Shutdown();
}

bool Application::Initialize() {
    if (!m_renderer.Initialize())
        return false;

    if (!m_meshManager.CreateCube("cube"))
        return false;

    if (!m_world.Initialize(m_meshManager))
        return false;

    m_time.Reset();

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

    m_world.Update(
        deltaTime,
        m_input
    );
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
    m_meshManager.Clear();
    m_renderer.Shutdown();

    m_running = false;
}