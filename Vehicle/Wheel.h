#pragma once

#include "../Core/Math/Quaternion.h"
#include "../Core/Math/Vec3.h"
#include "WheelContact.h"

class PhysicsWorld;
class IWheelContactProvider;
class RigidBody;
class Suspension;

class Wheel {
public:
    Wheel();

    void SetLocalPosition(const Vec3& position);
    const Vec3& GetLocalPosition() const;

    void SetHubPosition(const Vec3& position);
    void SetHubState(const Vec3& position, const Quaternion& orientation);

    void SetRadius(float radius);
    float GetRadius() const;

    void SetInertia(float inertia);
    float GetInertia() const;

    void SetDriveTorque(float torque);
    float GetDriveTorque() const;

    void SetBrakeTorque(float torque);
    float GetBrakeTorque() const;

    void SetSteeringAngle(float angle);
    float GetSteeringAngle() const;

    void Update(
        int index,
        RigidBody& body,
        const IWheelContactProvider& contactProvider,
        const PhysicsWorld& physicsWorld,
        Suspension& suspension,
        const Vec3& springMountA,
        const Vec3& springMountB,
        float deltaTime
    );

    void ApplyTireForce(
        const RigidBody& body,
        const Vec3& force
    );

    void IntegrateRotation(float deltaTime);

    bool IsGrounded() const;
    float GetSuspensionLength() const;
    float GetLastContactDistance() const;
    const WheelContactResult& GetContactResult() const;
    float GetCompression() const;
    float GetForce() const;
    float GetNormalLoad() const;
    float GetCompressionVelocity() const;
    float GetSpringForce() const;
    float GetDamperForce() const;
    float GetSuspensionPower() const;
    float GetSuspensionResidual() const;
    float GetAngularVelocity() const;
    float GetRotationAngle() const;
    float GetTireReactionTorque() const;
    const Vec3& GetWorldPosition() const;
    const Quaternion& GetWorldOrientation() const;
    const Vec3& GetContactPoint() const;
    const Vec3& GetContactNormal() const;

private:
    Vec3 m_localPosition;
    float m_radius;
    float m_inertia;

    float m_driveTorque;
    float m_brakeTorque;
    float m_steeringAngle;
    float m_angularVelocity;
    float m_rotationAngle;
    float m_tireReactionTorque;

    bool m_grounded;
    float m_suspensionLength;
    float m_lastContactDistance;
    float m_compression;
    float m_previousCompression;
    float m_force;
    float m_normalLoad;
    float m_compressionVelocity;
    float m_springForce;
    float m_damperForce;
    float m_suspensionPower;
    float m_suspensionResidual;
    bool m_hasPreviousCompression;

    Vec3 m_worldPosition;
    Quaternion m_worldOrientation;
    Vec3 m_contactPoint;
    Vec3 m_contactNormal;
    WheelContactResult m_contactResult;
};
