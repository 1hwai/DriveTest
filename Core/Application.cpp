#include "Application.h"

#include <sstream>

Application::Application()
    : m_running(false),
    m_physicsAccumulator(0.0f),
    m_fpsTimer(0.0f),
    m_frameCount(0),
    m_physicsStepCount(0) {}

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

    Ray ray;
    ray.origin = Vec3(0.0f, 5.0f, 0.0f);
    ray.direction = Vec3(0.0f, -1.0f, 0.0f);

    RaycastResult result;

    if (m_physicsWorld.Raycast(
        ray,
        result
    )) {
        std::ostringstream message;
        message << "[Raycast Test] HIT distance="
            << result.distance
            << " point=("
            << result.point.x << ", "
            << result.point.y << ", "
            << result.point.z << ") normal=("
            << result.normal.x << ", "
            << result.normal.y << ", "
            << result.normal.z << ")";

        Logger::Info(message.str());
    }
    else {
        Logger::Error(
            "[Raycast Test] MISS"
        );
    }

    m_time.Reset();

    m_physicsAccumulator = 0.0f;
    m_fpsTimer = 0.0f;
    m_frameCount = 0;
    m_physicsStepCount = 0;

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
        ++m_physicsStepCount;
    }

    if (physicsStepsThisFrame >= MaxPhysicsStepsPerFrame &&
        m_physicsAccumulator >= PhysicsFixedDeltaTime) {

        m_physicsAccumulator = 0.0f;
    }

    m_world.Update(
        deltaTime,
        m_input
    );

    ++m_frameCount;
    m_fpsTimer += deltaTime;

    if (m_fpsTimer >= 1.0f) {
        std::ostringstream message;
        message << "[Performance] FPS="
            << m_frameCount
            << " PhysicsHz="
            << m_physicsStepCount;

        Logger::Debug(message.str());

        m_fpsTimer = 0.0f;
        m_frameCount = 0;
        m_physicsStepCount = 0;
    }
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