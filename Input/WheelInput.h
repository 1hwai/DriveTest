#pragma once

class WheelInput {
public:
    WheelInput();

    void Update();

    float GetSteering() const;
    float GetThrottle() const;
    float GetBrake() const;
    float GetClutch() const;

private:
    float m_steering;
    float m_throttle;
    float m_brake;
    float m_clutch;
};