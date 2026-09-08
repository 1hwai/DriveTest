#include "Material.h"

Material::Material()
    : m_restitution(0.0f), m_friction(0.5f) {}

void Material::SetRestitution(float restitution) {
    if (restitution < 0.0f)
        restitution = 0.0f;
    else if (restitution > 1.0f)
        restitution = 1.0f;

    m_restitution = restitution;
}

float Material::GetRestitution() const {
    return m_restitution;
}

void Material::SetFriction(float friction) {
    m_friction = friction > 0.0f ? friction : 0.0f;
}

float Material::GetFriction() const {
    return m_friction;
}