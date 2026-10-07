#include "Suspension.h"
#include <algorithm>

Suspension::Suspension()
    : m_restLength(0.8f), m_maxLength(1.0f), m_bumpTravel(0.8f),
      m_springRate(30000.0f), m_compressionDamperRate(4500.0f),
      m_reboundDamperRate(4500.0f), m_length(0.8f),
      m_compression(0.0f), m_compressionVelocity(0.0f) {}

void Suspension::SetRestLength(float length) {
    m_restLength = std::max(0.0f, length);
    m_maxLength = std::max(m_maxLength, m_restLength);
    m_bumpTravel = std::min(m_bumpTravel, m_restLength);
}
float Suspension::GetRestLength() const { return m_restLength; }

void Suspension::SetMaxLength(float length) {
    m_maxLength = std::max(m_restLength, length);
}
float Suspension::GetMaxLength() const { return m_maxLength; }

void Suspension::SetBumpTravel(float travel) {
    m_bumpTravel = std::clamp(travel, 0.0f, m_restLength);
}
float Suspension::GetBumpTravel() const { return m_bumpTravel; }

void Suspension::SetReboundTravel(float travel) {
    m_maxLength = m_restLength + std::max(0.0f, travel);
}
float Suspension::GetReboundTravel() const { return m_maxLength - m_restLength; }

void Suspension::SetSpringRate(float rate) {
    m_springRate = std::max(0.0f, rate);
}
float Suspension::GetSpringRate() const { return m_springRate; }

void Suspension::SetDamperRate(float rate) {
    SetCompressionDamperRate(rate);
    SetReboundDamperRate(rate);
}
float Suspension::GetDamperRate() const { return m_compressionDamperRate; }

void Suspension::SetCompressionDamperRate(float rate) {
    m_compressionDamperRate = std::max(0.0f, rate);
}
float Suspension::GetCompressionDamperRate() const { return m_compressionDamperRate; }

void Suspension::SetReboundDamperRate(float rate) {
    m_reboundDamperRate = std::max(0.0f, rate);
}
float Suspension::GetReboundDamperRate() const { return m_reboundDamperRate; }

float Suspension::ClampLength(float length) const {
    return std::clamp(
        length,
        std::max(0.0f, m_restLength - m_bumpTravel),
        m_maxLength
    );
}

void Suspension::UpdateLength(float length, float deltaTime) {
    const float previousCompression = m_compression;
    m_length = ClampLength(length);
    m_compression = std::max(0.0f, m_restLength - m_length);

    if (deltaTime > 0.0f)
        m_compressionVelocity =
            (m_compression - previousCompression) / deltaTime;
    else
        m_compressionVelocity = 0.0f;
}

float Suspension::GetLength() const { return m_length; }
float Suspension::GetCompression() const { return m_compression; }
float Suspension::GetCompressionVelocity() const {
    return m_compressionVelocity;
}

float Suspension::CalculateForce() const {
    if (m_compression <= 0.0f)
        return 0.0f;

    const float damperRate =
        m_compressionVelocity >= 0.0f
            ? m_compressionDamperRate
            : m_reboundDamperRate;

    return std::max(
        0.0f,
        m_springRate * m_compression +
        damperRate * m_compressionVelocity
    );
}
