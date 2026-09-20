#include "Performance.h"

#include <cmath>
#include <sstream>

#include "Logger.h"

Performance::Performance()
    : m_timer(0.0f),
    m_frameCount(0),
    m_physicsStepCount(0) {}

void Performance::Reset() {
    m_timer = 0.0f;
    m_frameCount = 0;
    m_physicsStepCount = 0;
}

void Performance::UpdateFrame(float deltaTime) {
    ++m_frameCount;
    m_timer += deltaTime;

    if (m_timer < 1.0f)
        return;

    const float elapsedTime = m_timer;
    const int fps = static_cast<int>(
        std::round(
            static_cast<float>(m_frameCount) / elapsedTime
        )
    );
    const int physicsHz = static_cast<int>(
        std::round(
            static_cast<float>(m_physicsStepCount) / elapsedTime
        )
    );

    std::ostringstream message;
    message << "[Performance] FPS="
        << fps
        << " PhysicsHz="
        << physicsHz;

    Logger::Debug(message.str());

    Reset();
}

void Performance::RecordPhysicsStep() {
    ++m_physicsStepCount;
}
