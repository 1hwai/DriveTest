#pragma once

class Time {
public:
    Time();

    void Reset();
    void Update();

    float GetDeltaTime() const;
    float GetTotalTime() const;

private:
    double m_previousTime;
    float m_deltaTime;
    float m_totalTime;
};