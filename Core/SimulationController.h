#pragma once

class SimulationController {
public:
    SimulationController();

    void Reset();

    void Pause();
    void Resume();

    bool IsPaused() const;

    void SetTimeScale(float timeScale);
    float GetTimeScale() const;

    void RequestSingleStep();
    bool ConsumeSingleStep();

private:
    bool m_paused;
    bool m_singleStepRequested;
    float m_timeScale;
};