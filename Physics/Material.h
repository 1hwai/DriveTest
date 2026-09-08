#pragma once

class Material {
public:
    Material();

    void SetRestitution(float restitution);
    float GetRestitution() const;

    void SetFriction(float friction);
    float GetFriction() const;

private:
    float m_restitution;
    float m_friction;
};