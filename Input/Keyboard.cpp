#include "Keyboard.h"

#include <cstring>

Keyboard::Keyboard()
    : m_current{},
    m_previous{},
    m_pressed{},
    m_released{} {}

void Keyboard::ProcessEvent(const SDL_Event& event) {
    switch (event.type) {
    case SDL_EVENT_KEY_DOWN:
        if (event.key.scancode >= 0 &&
            event.key.scancode < SDL_SCANCODE_COUNT) {

            if (!event.key.repeat &&
                !m_current[event.key.scancode]) {

                m_current[event.key.scancode] = true;
                m_pressed[event.key.scancode] = true;
            }
        }
        break;

    case SDL_EVENT_KEY_UP:
        if (event.key.scancode >= 0 &&
            event.key.scancode < SDL_SCANCODE_COUNT) {

            m_current[event.key.scancode] = false;
            m_released[event.key.scancode] = true;
        }
        break;

    default:
        break;
    }
}

void Keyboard::Update() {
    std::memcpy(
        m_previous,
        m_current,
        sizeof(m_current)
    );

    std::memset(
        m_pressed,
        false,
        sizeof(m_pressed)
    );

    std::memset(
        m_released,
        false,
        sizeof(m_released)
    );
}

bool Keyboard::IsDown(SDL_Scancode key) const {
    if (key < 0 || key >= SDL_SCANCODE_COUNT)
        return false;

    return m_current[key];
}

bool Keyboard::IsPressed(SDL_Scancode key) const {
    if (key < 0 || key >= SDL_SCANCODE_COUNT)
        return false;

    return m_pressed[key];
}

bool Keyboard::IsReleased(SDL_Scancode key) const {
    if (key < 0 || key >= SDL_SCANCODE_COUNT)
        return false;

    return m_released[key];
}