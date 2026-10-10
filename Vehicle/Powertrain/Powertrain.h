#pragma once

#include "Engine.h"
#include "Transmission.h"
#include "Differential.h"

class Powertrain {
public:
    Powertrain();

    void Update(
        float throttle,
        float clutch,
        float drivenWheelAngularVelocity,
        float deltaTime
    );

    void ShiftUp();
    void ShiftDown();

    Engine& GetEngine();
    const Engine& GetEngine() const;

    Transmission& GetTransmission();
    const Transmission& GetTransmission() const;

    float GetLeftDriveTorque() const;
    float GetRightDriveTorque() const;

private:
    Engine m_engine;
    Transmission m_transmission;
    Differential m_differential;
    float m_leftDriveTorque;
    float m_rightDriveTorque;
};
