#pragma once

#include <vector>

class Transmission {
public:
    Transmission();

    void ShiftUp();
    void ShiftDown();

    int GetGear() const;
    float GetGearRatio() const;
    float GetFinalDriveRatio() const;

    float GetOutputTorque(float inputTorque) const;
    float GetInputLoadTorque(float outputTorque) const;

private:
    std::vector<float> m_gearRatios;
    float m_reverseRatio;
    float m_finalDriveRatio;
    int m_currentGear;
};
