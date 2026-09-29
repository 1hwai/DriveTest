#pragma once

#include <SDL3/SDL.h>

class GamepadInput {
public:
    GamepadInput();
    ~GamepadInput();

    void ProcessEvent(const SDL_Event& event);
    void Update();

    bool IsConnected() const;

    float GetSteering() const;
    float GetThrottle() const;
    float GetBrake() const;
    float GetClutch() const;

    bool IsShiftUpPressed() const;
    bool IsShiftDownPressed() const;

private:
    void Open(SDL_JoystickID instanceId);
    void Close();

    SDL_Gamepad* m_gamepad;
    float m_steering;
    float m_throttle;
    float m_brake;
    float m_clutch;
    bool m_shiftUpPressed;
    bool m_shiftDownPressed;
    bool m_shiftUpPending;
    bool m_shiftDownPending;
};