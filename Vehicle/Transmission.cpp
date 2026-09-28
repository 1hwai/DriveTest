#include "Transmission.h"

#include <cmath>

Transmission::Transmission()
    : Transmission(
        {
            3.50f,
            2.10f,
            1.50f,
            1.10f,
            0.85f
        },
        3.20f,
        4.10f
    ) {}

Transmission::Transmission(
    const std::vector<float>& gearRatios,
    float reverseRatio,
    float finalDriveRatio
)
    : m_gearRatios(gearRatios),
    m_reverseRatio(reverseRatio),
    m_finalDriveRatio(finalDriveRatio),
    m_currentGear(1) {
    SetGearRatios(gearRatios);
}

void Transmission::SetGearRatios(
    const std::vector<float>& gearRatios
) {
    if (gearRatios.size() < 4)
        return;

    m_gearRatios = gearRatios;
    m_currentGear =
        std::min(
            m_currentGear,
            static_cast<int>(m_gearRatios.size())
        );
}

void Transmission::ShiftUp() {
    const int maxGear =
        static_cast<int>(m_gearRatios.size());

    if (m_currentGear < maxGear)
        ++m_currentGear;
    else if (m_currentGear == -1)
        m_currentGear = 0;
}

void Transmission::ShiftDown() {
    if (m_currentGear > 1)
        --m_currentGear;
    else if (m_currentGear == 1)
        m_currentGear = 0;
    else if (m_currentGear == 0)
        m_currentGear = -1;
}

int Transmission::GetGear() const {
    return m_currentGear;
}

float Transmission::GetGearRatio() const {
    if (m_currentGear > 0) {
        const size_t index =
            static_cast<size_t>(m_currentGear - 1);

        if (index < m_gearRatios.size())
            return m_gearRatios[index];
    }

    if (m_currentGear == -1)
        return -m_reverseRatio;

    return 0.0f;
}

float Transmission::GetFinalDriveRatio() const {
    return m_finalDriveRatio;
}

float Transmission::GetOutputTorque(float inputTorque) const {
    return inputTorque *
        GetGearRatio() *
        m_finalDriveRatio;
}

float Transmission::GetInputLoadTorque(float outputTorque) const {
    const float ratio =
        GetGearRatio() *
        m_finalDriveRatio;

    if (std::abs(ratio) < 0.0001f)
        return 0.0f;

    return outputTorque / ratio;
}
