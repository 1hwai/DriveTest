#pragma once

class Suspension {
public:
    Suspension();
    void SetRestLength(float length);
    float GetRestLength() const;
    void SetMaxLength(float length);
    float GetMaxLength() const;
    void SetBumpTravel(float travel);
    float GetBumpTravel() const;
    void SetReboundTravel(float travel);
    float GetReboundTravel() const;
    void SetSpringRate(float rate);
    float GetSpringRate() const;
    void SetDamperRate(float rate);
    float GetDamperRate() const;
    void SetCompressionDamperRate(float rate);
    float GetCompressionDamperRate() const;
    void SetReboundDamperRate(float rate);
    float GetReboundDamperRate() const;
    float ClampLength(float length) const;
    float CalculateForce(float compression, float compressionVelocity) const;
private:
    float m_restLength;
    float m_maxLength;
    float m_bumpTravel;
    float m_springRate;
    float m_compressionDamperRate;
    float m_reboundDamperRate;
};