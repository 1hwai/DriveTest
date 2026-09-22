#pragma once

class Suspension {
public:
    Suspension();

    void SetRestLength(float length);
    float GetRestLength() const;

    void SetMaxLength(float length);
    float GetMaxLength() const;

    void SetSpringRate(float rate);
    float GetSpringRate() const;

    void SetDamperRate(float rate);
    float GetDamperRate() const;

    float ClampLength(float length) const;

    float CalculateForce(
        float compression,
        float compressionVelocity
    ) const;

private:
    float m_restLength;
    float m_maxLength;
    float m_springRate;
    float m_damperRate;
};
