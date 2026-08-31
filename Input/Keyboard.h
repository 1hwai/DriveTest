#pragma once

#include <SDL3/SDL.h>

class Keyboard {
public:
    Keyboard();

    void ProcessEvent(const SDL_Event& event);
    void Update();

    bool IsDown(SDL_Scancode key) const;
    bool IsPressed(SDL_Scancode key) const;
    bool IsReleased(SDL_Scancode key) const;

private:
    bool m_current[SDL_SCANCODE_COUNT];
    bool m_previous[SDL_SCANCODE_COUNT];
};