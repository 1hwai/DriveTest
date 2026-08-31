#pragma once

#include <SDL3/SDL.h>

#include "Keyboard.h"
#include "WheelInput.h"

class InputManager {
public:
    InputManager();

    void ProcessEvent(const SDL_Event& event);
    void Update();

    const Keyboard& GetKeyboard() const;
    const WheelInput& GetWheelInput() const;

    bool QuitRequested() const;

private:
    Keyboard m_keyboard;
    WheelInput m_wheelInput;

    bool m_quitRequested;
};