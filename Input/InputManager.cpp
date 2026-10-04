#include "InputManager.h"

InputManager::InputManager()
    : m_keyboard(),
    m_wheelInput(),
    m_gamepadInput(),
    m_quitRequested(false) {}

void InputManager::ProcessEvent(const SDL_Event& event) {
    if (event.type == SDL_EVENT_QUIT)
        m_quitRequested = true;

    m_keyboard.ProcessEvent(event);
    m_gamepadInput.ProcessEvent(event);
}

void InputManager::Update() {
    m_keyboard.Update();
    m_wheelInput.Update();
    m_gamepadInput.Update();
}

const Keyboard& InputManager::GetKeyboard() const {
    return m_keyboard;
}

const WheelInput& InputManager::GetWheelInput() const {
    return m_wheelInput;
}

const GamepadInput& InputManager::GetGamepadInput() const {
    return m_gamepadInput;
}

VehicleInput InputManager::GetVehicleInput() const {
    VehicleInput input;

    const bool gamepadConnected =
        m_gamepadInput.IsConnected();

    input.throttle =
        gamepadConnected
            ? m_gamepadInput.GetThrottle()
            : (m_keyboard.IsDown(SDL_SCANCODE_W) ? 1.0f : 0.0f);

    input.brake =
        gamepadConnected
            ? m_gamepadInput.GetBrake()
            : (m_keyboard.IsDown(SDL_SCANCODE_S) ? 1.0f : 0.0f);

    // Keep the shared vehicle-input convention consistent across devices:
    // positive = right, negative = left.
    input.steering =
        gamepadConnected
            ? m_gamepadInput.GetSteering()
            : (m_keyboard.IsDown(SDL_SCANCODE_D) ? 1.0f : 0.0f) -
              (m_keyboard.IsDown(SDL_SCANCODE_A) ? 1.0f : 0.0f);

    input.clutch =
        gamepadConnected
            ? m_gamepadInput.GetClutch()
            : 0.0f;

    input.shiftUp =
        m_keyboard.IsPressed(SDL_SCANCODE_EQUALS) ||
        m_keyboard.IsPressed(SDL_SCANCODE_KP_PLUS) ||
        m_gamepadInput.IsShiftUpPressed();

    input.shiftDown =
        m_keyboard.IsPressed(SDL_SCANCODE_MINUS) ||
        m_keyboard.IsPressed(SDL_SCANCODE_KP_MINUS) ||
        m_gamepadInput.IsShiftDownPressed();

    return input;
}

bool InputManager::QuitRequested() const {
    return m_quitRequested;
}
