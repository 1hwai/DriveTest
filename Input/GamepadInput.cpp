#include "GamepadInput.h"

#include <algorithm>
#include <cmath>

GamepadInput::GamepadInput()
    : m_gamepad(nullptr),
    m_steering(0.0f),
    m_throttle(0.0f),
    m_brake(0.0f),
    m_clutch(0.0f),
    m_shiftUpPressed(false),
    m_shiftDownPressed(false),
    m_shiftUpPending(false),
    m_shiftDownPending(false) {}

GamepadInput::~GamepadInput() {
    Close();
}

void GamepadInput::ProcessEvent(const SDL_Event& event) {
    switch (event.type) {
    case SDL_EVENT_GAMEPAD_ADDED:
        if (m_gamepad == nullptr)
            Open(event.gdevice.which);
        break;

    case SDL_EVENT_GAMEPAD_REMOVED:
        if (m_gamepad != nullptr &&
            SDL_GetGamepadID(m_gamepad) == event.gdevice.which) {

            Close();
        }
        break;

    case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        if (m_gamepad == nullptr ||
            event.gbutton.which != SDL_GetGamepadID(m_gamepad))
            break;

        if (event.gbutton.button == SDL_GAMEPAD_BUTTON_SOUTH) {
            m_shiftUpPending = true;
        }
        else if (event.gbutton.button == SDL_GAMEPAD_BUTTON_WEST) {
            m_shiftDownPending = true;
        }
        break;

    default:
        break;
    }
}

void GamepadInput::Update() {
    m_shiftUpPressed = m_shiftUpPending;
    m_shiftDownPressed = m_shiftDownPending;
    m_shiftUpPending = false;
    m_shiftDownPending = false;

    if (m_gamepad == nullptr)
        return;

    constexpr float SteeringDeadzone = 0.15f;

    const float rawSteering =
        static_cast<float>(
            SDL_GetGamepadAxis(
                m_gamepad,
                SDL_GAMEPAD_AXIS_LEFTX
            )
        ) / 32767.0f;

    const float steeringMagnitude = std::abs(rawSteering);
    if (steeringMagnitude <= SteeringDeadzone) {
        m_steering = 0.0f;
    }
    else {
        m_steering =
            std::copysign(
                (steeringMagnitude - SteeringDeadzone) /
                (1.0f - SteeringDeadzone),
                rawSteering
            );
    }

    m_throttle =
        static_cast<float>(
            SDL_GetGamepadAxis(
                m_gamepad,
                SDL_GAMEPAD_AXIS_RIGHT_TRIGGER
            )
        ) / 32767.0f;

    m_brake =
        static_cast<float>(
            SDL_GetGamepadAxis(
                m_gamepad,
                SDL_GAMEPAD_AXIS_LEFT_TRIGGER
            )
        ) / 32767.0f;

    constexpr float ClutchDeadzone = 8000.0f;

    const float clutchAxis =
        -static_cast<float>(
            SDL_GetGamepadAxis(
                m_gamepad,
                SDL_GAMEPAD_AXIS_RIGHTY
            )
        );

    m_clutch =
        (clutchAxis - ClutchDeadzone) /
        (32767.0f - ClutchDeadzone);

    m_steering = std::clamp(m_steering, -1.0f, 1.0f);
    m_throttle = std::clamp(m_throttle, 0.0f, 1.0f);
    m_brake = std::clamp(m_brake, 0.0f, 1.0f);
    m_clutch = std::clamp(m_clutch, 0.0f, 1.0f);
}

bool GamepadInput::IsConnected() const {
    return m_gamepad != nullptr;
}

float GamepadInput::GetSteering() const {
    return m_steering;
}

float GamepadInput::GetThrottle() const {
    return m_throttle;
}

float GamepadInput::GetBrake() const {
    return m_brake;
}

float GamepadInput::GetClutch() const {
    return m_clutch;
}

bool GamepadInput::IsShiftUpPressed() const {
    return m_shiftUpPressed;
}

bool GamepadInput::IsShiftDownPressed() const {
    return m_shiftDownPressed;
}

void GamepadInput::Open(SDL_JoystickID instanceId) {
    if (m_gamepad != nullptr)
        return;

    m_gamepad = SDL_OpenGamepad(instanceId);
}

void GamepadInput::Close() {
    if (m_gamepad != nullptr) {
        SDL_CloseGamepad(m_gamepad);
        m_gamepad = nullptr;
    }

    m_steering = 0.0f;
    m_throttle = 0.0f;
    m_brake = 0.0f;
    m_clutch = 0.0f;
    m_shiftUpPressed = false;
    m_shiftDownPressed = false;
    m_shiftUpPending = false;
    m_shiftDownPending = false;
}