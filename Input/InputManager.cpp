#include "InputManager.h"

InputManager::InputManager()
    : m_keyboard(),
    m_wheelInput(),
    m_gamepadInput(),
    m_quitRequested(false),
    m_vehicleInput{} {}

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

    const bool gamepadConnected =
        m_gamepadInput.IsConnected();

    m_vehicleInput.throttle =
        gamepadConnected
            ? m_gamepadInput.GetThrottle()
            : (m_keyboard.IsDown(SDL_SCANCODE_W) ? 1.0f : 0.0f);

    m_vehicleInput.brake =
        gamepadConnected
            ? m_gamepadInput.GetBrake()
            : (m_keyboard.IsDown(SDL_SCANCODE_S) ? 1.0f : 0.0f);

    m_vehicleInput.steering =
        gamepadConnected
            ? m_gamepadInput.GetSteering()
            : (m_keyboard.IsDown(SDL_SCANCODE_A) ? 1.0f : 0.0f) -
              (m_keyboard.IsDown(SDL_SCANCODE_D) ? 1.0f : 0.0f);

    m_vehicleInput.clutch =
        gamepadConnected
            ? m_gamepadInput.GetClutch()
            : 0.0f;

    m_vehicleInput.shiftUp =
        m_keyboard.IsPressed(SDL_SCANCODE_EQUALS) ||
        m_keyboard.IsPressed(SDL_SCANCODE_KP_PLUS) ||
        m_gamepadInput.IsShiftUpPressed();

    m_vehicleInput.shiftDown =
        m_keyboard.IsPressed(SDL_SCANCODE_MINUS) ||
        m_keyboard.IsPressed(SDL_SCANCODE_KP_MINUS) ||
        m_gamepadInput.IsShiftDownPressed();
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

const VehicleInput& InputManager::GetVehicleInput() const {
    return m_vehicleInput;
}

bool InputManager::QuitRequested() const {
    return m_quitRequested;
}