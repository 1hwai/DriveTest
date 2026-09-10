#pragma once

#include "../Core/Math/Vec3.h"
#include "../Core/Math/Quaternion.h"
#include "../Core/Math/Mat3.h"

class RigidBody {
public:
    RigidBody();
    ~RigidBody() = default;

    void SetMass(float mass);
    float GetMass() const;
    float GetInverseMass() const;

    void SetBoxInertia(const Vec3& size);

    void SetPosition(const Vec3& position);
    const Vec3& GetPosition() const;

    void SetOrientation(const Quaternion& orientation);
    const Quaternion& GetOrientation() const;

    void SetLinearVelocity(const Vec3& velocity, bool wake = true);
    const Vec3& GetLinearVelocity() const;

    void SetAngularVelocity(const Vec3& velocity, bool wake = true);
    const Vec3& GetAngularVelocity() const;

    void AddForce(const Vec3& force);
    void AddTorque(const Vec3& torque);

    const Vec3& GetForce() const;
    const Vec3& GetTorque() const;

    void ClearForces();

    void SetInertiaTensor(const Mat3& inertiaTensor);
    const Mat3& GetInertiaTensor() const;

    void SetInverseInertiaTensor(const Mat3& inverseInertiaTensor);
    const Mat3& GetInverseInertiaTensor() const;

    Mat3 GetWorldInverseInertiaTensor() const;

    void Integrate(float deltaTime, const Vec3& gravity);

    void UpdateSleep(float deltaTime);

    void Wake();
    void Sleep();

    bool IsSleeping() const;

private:
    Vec3 m_position;
    Quaternion m_orientation;
    Vec3 m_linearVelocity;
    Vec3 m_angularVelocity;
    Vec3 m_force;
    Vec3 m_torque;
    float m_mass;
    float m_inverseMass;
    Mat3 m_inertiaTensor;
    Mat3 m_inverseInertiaTensor;

    bool m_isSleeping;
    float m_sleepTimer;

    static constexpr float SleepLinearVelocityThreshold = 0.05f;
    static constexpr float SleepAngularVelocityThreshold = 0.05f;
    static constexpr float SleepTimeThreshold = 0.5f;
};