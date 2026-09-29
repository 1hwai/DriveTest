#pragma once

class Engine {
public:
    Engine();

    void Update(
        float throttle,
        float loadTorque,
        float drivetrainTorque,
        float deltaTime
    );

    float GetRPM() const;
    float GetAngularVelocity() const;
    float GetTorque() const;
    float GetLoadTorque() const;
    float GetIdleRPM() const;
    float GetRedlineRPM() const;

private:
    float GetTorqueAtRPM(float rpm) const;

    float m_idleRPM;
    float m_redlineRPM;
    float m_peakTorque;
    float m_inertia;

    float m_angularVelocity;
    float m_torque;
    float m_loadTorque;
};
