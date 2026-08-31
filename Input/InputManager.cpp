#include "InputManager.h"

InputManager::InputManager()
    : m_keyboard(),
    m_wheelInput(),
    m_quitRequested(false) {}

void InputManager::ProcessEvent(const SDL_Event& event) {
    if (event.type == SDL_EVENT_QUIT) {
        m_quitRequested = true;
    }

    m_keyboard.ProcessEvent(event);
}

void InputManager::Update() {
    m_keyboard.Update();
    m_wheelInput.Update();
}

const Keyboard& InputManager::GetKeyboard() const {
    return m_keyboard;
}

const WheelInput& InputManager::GetWheelInput() const {
    return m_wheelInput;
}

bool InputManager::QuitRequested() const {
    return m_quitRequested;
}