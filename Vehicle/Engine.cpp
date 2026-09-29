#include "Engine.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr float Pi = 3.14159265358979323846f;
}

Engine::Engine()
    : m_idleRPM(900.0f),
    m_redlineRPM(7000.0f),
    m_peakTorque(280.0f),
    m_inertia(0.25f),
    m_angularVelocity(
        900.0f * 2.0f * Pi / 60.0f
    ),
    m_torque(0.0f),
    m_loadTorque(0.0f) {}

void Engine::Update(
    float throttle,
    float loadTorque,
    float drivetrainTorque,
    float deltaTime
) {
    if (deltaTime <= 0.0f)
        return;

    throttle = std::clamp(throttle, 0.0f, 1.0f);
    m_loadTorque = std::max(0.0f, loadTorque);

    const float idleAngularVelocity =
        m_idleRPM * 2.0f * Pi / 60.0f;

    const float redlineAngularVelocity =
        m_redlineRPM * 2.0f * Pi / 60.0f;

    const float maxTorque =
        GetTorqueAtRPM(GetRPM());

    m_torque =
        maxTorque * throttle;

    const float speedAboveIdle =
        std::max(
            0.0f,
            m_angularVelocity - idleAngularVelocity
        );

    const float engineFrictionTorque =
        18.0f +
        speedAboveIdle * 0.03f;

    const float netTorque =
        m_torque -
        m_loadTorque +
        drivetrainTorque -
        engineFrictionTorque;

    m_angularVelocity +=
        (netTorque / m_inertia) * deltaTime;

    m_angularVelocity = std::clamp(
        m_angularVelocity,
        idleAngularVelocity,
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
