#include "WheelInput.h"

WheelInput::WheelInput()
    : m_steering(0.0f),
    m_throttle(0.0f),
    m_brake(0.0f),
    m_clutch(0.0f) {}

void WheelInput::Update() {
    // TODO:
    // 실제 레이싱 휠/페달 장치 입력을 여기서 갱신한다.
}

float WheelInput::GetSteering() const {
    return m_steering;
}

float WheelInput::GetThrottle() const {
    return m_throttle;
}

float WheelInput::GetBrake() const {
    return m_brake;
}

float WheelInput::GetClutch() const {
    return m_clutch;
}