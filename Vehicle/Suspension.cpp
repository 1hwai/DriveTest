#include "Suspension.h"

#include <algorithm>

Suspension::Suspension()
    : m_restLength(0.8f),
    m_maxLength(1.0f),
    m_springRate(30000.0f),
    m_damperRate(4500.0f) {}

void Suspension::SetRestLength(float length) {
    m_restLength =
        std::max(0.0f, length);

    if (m_maxLength < m_restLength)
        m_maxLength = m_restLength;
}

float Suspension::GetRestLength() const {
    return m_restLength;
}

void Suspension::SetMaxLength(float length) {
    m_maxLength =
        std::max(m_restLength, length);
}

float Suspension::GetMaxLength() const {
    return m_maxLength;
}

void Suspension::SetSpringRate(float rate) {
    m_springRate =
        std::max(0.0f, rate);
}

float Suspension::GetSpringRate() const {
    return m_springRate;
}

void Suspension::SetDamperRate(float rate) {
    m_damperRate =
        std::max(0.0f, rate);
}

float Suspension::GetDamperRate() const {
    return m_damperRate;
}

float Suspension::ClampLength(float length) const {
    return std::clamp(
        length,
        0.0f,
        m_maxLength
    );
}

float Suspension::CalculateForce(
    float compression,
    float compressionVelocity
) const {
    if (compression <= 0.0f)
        return 0.0f;

    const float springForce =
        m_springRate * compression;

    const float damperForce =
        m_damperRate * compressionVelocity;

    return std::max(
        0.0f,
        springForce + damperForce
    );
}
