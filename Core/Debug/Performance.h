#pragma once

class Performance {
public:
    Performance();

    void Reset();
    void UpdateFrame(float deltaTime);
    void RecordPhysicsStep();

private:
    float m_timer;
    int m_frameCount;
    int m_physicsStepCount;
};
