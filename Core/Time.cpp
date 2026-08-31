#include "Time.h"

#include <SDL3/SDL.h>

Time::Time()
    : m_previousTime(0.0),
    m_deltaTime(0.0f),
    m_totalTime(0.0f) {}

void Time::Reset() {
    m_previousTime =
        static_cast<double>(SDL_GetPerformanceCounter());

    m_deltaTime = 0.0f;
    m_totalTime = 0.0f;
}

void Time::Update() {
    const double currentTime =
        static_cast<double>(SDL_GetPerformanceCounter());

    const double frequency =
        static_cast<double>(SDL_GetPerformanceFrequency());

    m_deltaTime = static_cast<float>(
        (currentTime - m_previousTime) / frequency
        );

    m_previousTime = currentTime;

    m_totalTime += m_deltaTime;
}

float Time::GetDeltaTime() const {
    return m_deltaTime;
}

float Time::GetTotalTime() const {
    return m_totalTime;
}