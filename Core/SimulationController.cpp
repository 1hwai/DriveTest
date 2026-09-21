#include "SimulationController.h"

SimulationController::SimulationController()
    : m_paused(false),
    m_singleStepRequested(false),
    m_timeScale(1.0f) {}

void SimulationController::Reset() {
    m_paused = false;
    m_singleStepRequested = false;
    m_timeScale = 1.0f;
}

void SimulationController::Pause() {
    m_paused = true;
}

void SimulationController::Resume() {
    m_paused = false;
    m_singleStepRequested = false;
}

bool SimulationController::IsPaused() const {
    return m_paused;
}

void SimulationController::SetTimeScale(float timeScale) {
    m_timeScale = timeScale > 0.0f ? timeScale : 0.0f;
}

float SimulationController::GetTimeScale() const {
    return m_timeScale;
}

void SimulationController::RequestSingleStep() {
    m_paused = true;
    m_singleStepRequested = true;
}

bool SimulationController::ConsumeSingleStep() {
    if (!m_singleStepRequested)
        return false;

    m_singleStepRequested = false;
    return true;
}
