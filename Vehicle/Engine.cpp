#include "Engine.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr float Pi = 3.14159265358979323846f;
    constexpr float IdleControlGain = 8.0f;
    constexpr float MaxIdleControlTorque = 60.0f;
}

Engine::Engine()
    : m_idleRPM(900.0f),
    m_stallRPM(550.0f),
    m_redlineRPM(7000.0f),
    m_peakTorque(280.0f),
    m_inertia(0.25f),
    m_angularVelocity(
        900.0f * 2.0f * Pi / 60.0f
    ),
    m_torque(0.0f),
    m_loadTorque(0.0f),
    m_running(true) {}

void Engine::Configure(float idleRPM, float stallRPM, float redlineRPM, float peakTorque, float inertia) {
    if (idleRPM <= stallRPM || redlineRPM <= idleRPM || peakTorque < 0.0f || inertia <= 0.0f)
        return;
    m_idleRPM = idleRPM;
    m_stallRPM = stallRPM;
    m_redlineRPM = redlineRPM;
    m_peakTorque = peakTorque;
    m_inertia = inertia;
    m_angularVelocity = m_idleRPM * 2.0f * Pi / 60.0f;
    m_torque = 0.0f;
    m_loadTorque = 0.0f;
    m_running = true;
}

void Engine::Update(
    float throttle,
    float loadTorque,
    float drivetrainTorque,
    float deltaTime
) {
    if (deltaTime <= 0.0f || !m_running)
        return;

    throttle = std::clamp(throttle, 0.0f, 1.0f);
    m_loadTorque = std::max(0.0f, loadTorque);

    const float idleAngularVelocity =
        m_idleRPM * 2.0f * Pi / 60.0f;

    const float stallAngularVelocity =
        m_stallRPM * 2.0f * Pi / 60.0f;

    const float redlineAngularVelocity =
        m_redlineRPM * 2.0f * Pi / 60.0f;

    const float maxTorque =
        GetTorqueAtRPM(GetRPM());

    m_torque =
        maxTorque * throttle;

    const float idleError =
        idleAngularVelocity -
        m_angularVelocity;

    const float idleControlTorque =
        std::clamp(
            idleError * IdleControlGain,
            0.0f,
            MaxIdleControlTorque
        );

    const float speedAboveIdle =
        std::max(
            0.0f,
            m_angularVelocity - idleAngularVelocity
        );

    const float engineFrictionTorque =
        18.0f +
        speedAboveIdle * 0.03f;

    const float netTorque =
        m_torque +
        idleControlTorque -
        m_loadTorque +
        drivetrainTorque -
        engineFrictionTorque;

    m_angularVelocity +=
        (netTorque / m_inertia) * deltaTime;

    if (m_angularVelocity <= stallAngularVelocity) {
        m_angularVelocity = 0.0f;
        m_torque = 0.0f;
        m_loadTorque = 0.0f;
        m_running = false;
        return;
    }

    m_angularVelocity =
        std::min(
            m_angularVelocity,
            redlineAngularVelocity
        );
}

float Engine::GetRPM() const {
    return m_angularVelocity *
        60.0f /
        (2.0f * Pi);
}

float Engine::GetAngularVelocity() const {
    return m_angularVelocity;
}

float Engine::GetTorque() const {
    return m_torque;
}

float Engine::GetLoadTorque() const {
    return m_loadTorque;
}

float Engine::GetIdleRPM() const {
    return m_idleRPM;
}

float Engine::GetRedlineRPM() const {
    return m_redlineRPM;
}

bool Engine::IsRunning() const {
    return m_running;
}

float Engine::GetTorqueAtRPM(float rpm) const {
    const float x = std::clamp(
        (rpm - m_idleRPM) /
        (m_redlineRPM - m_idleRPM),
        0.0f,
        1.0f
    );

    const float shape =
        0.7f +
        x -
        0.7f * x * x;

    return m_peakTorque * shape;
}
