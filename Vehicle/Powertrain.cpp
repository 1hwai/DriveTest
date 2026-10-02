#include "Powertrain.h"

#include <algorithm>
#include <cmath>

Powertrain::Powertrain()
    : m_engine(),
    m_transmission(),
    m_differential(),
    m_leftDriveTorque(0.0f),
    m_rightDriveTorque(0.0f) {}

void Powertrain::Update(
    float throttle,
    float clutch,
    float drivenWheelAngularVelocity,
    float deltaTime
) {
    constexpr float ClutchStiffness = 2.5f;
    constexpr float MaxClutchTorque = 100.0f;

    float clutchTorque = 0.0f;
    const float gearRatio = m_transmission.GetGearRatio();

    if (m_engine.IsRunning() &&
        std::abs(gearRatio) > 0.0001f) {

        const float drivetrainAngularVelocity =
            drivenWheelAngularVelocity *
            gearRatio *
            m_transmission.GetFinalDriveRatio();

        const float clutchEngagement = 1.0f - clutch;
        const float angularVelocityError =
            drivetrainAngularVelocity -
            m_engine.GetAngularVelocity();

        const float clutchCapacity =
            MaxClutchTorque * clutchEngagement;

        clutchTorque = std::clamp(
            angularVelocityError * ClutchStiffness,
            -clutchCapacity,
            clutchCapacity
        );
    }

    m_engine.Update(
        throttle,
        0.0f,
        clutchTorque,
        deltaTime
    );

    const float drivetrainTorque =
        -clutchTorque *
        gearRatio *
        m_transmission.GetFinalDriveRatio();

    m_differential.DistributeTorque(
        drivetrainTorque,
        m_leftDriveTorque,
        m_rightDriveTorque
    );
}

void Powertrain::ShiftUp() {
    m_transmission.ShiftUp();
}

void Powertrain::ShiftDown() {
    m_transmission.ShiftDown();
}

Engine& Powertrain::GetEngine() {
    return m_engine;
}

const Engine& Powertrain::GetEngine() const {
    return m_engine;
}

Transmission& Powertrain::GetTransmission() {
    return m_transmission;
}

const Transmission& Powertrain::GetTransmission() const {
    return m_transmission;
}

float Powertrain::GetLeftDriveTorque() const {
    return m_leftDriveTorque;
}

float Powertrain::GetRightDriveTorque() const {
    return m_rightDriveTorque;
}
