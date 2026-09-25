#include "Wheel.h"

#include "Suspension.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"

#include <algorithm>
#include <cmath>

Wheel::Wheel()
    : m_localPosition(0.0f, 0.0f, 0.0f),
    m_radius(0.5f),
    m_inertia(1.8f),
    m_driveTorque(0.0f),
    m_brakeTorque(0.0f),
    m_angularVelocity(0.0f),
    m_rotationAngle(0.0f),
    m_grounded(false),
    m_suspensionLength(0.0f),
    m_compression(0.0f),
    m_previousCompression(0.0f),
    m_force(0.0f),
    m_suspensionPower(0.0f),
    m_suspensionResidual(0.0f),
    m_hasPreviousCompression(false),
    m_worldPosition(0.0f, 0.0f, 0.0f),
    m_contactPoint(0.0f, 0.0f, 0.0f),
    m_contactNormal(0.0f, 1.0f, 0.0f) {}

void Wheel::SetLocalPosition(const Vec3& position) {
    m_localPosition = position;
}

const Vec3& Wheel::GetLocalPosition() const {
    return m_localPosition;
}

void Wheel::SetRadius(float radius) {
    m_radius = radius > 0.0f ? radius : 0.5f;
}

float Wheel::GetRadius() const {
    return m_radius;
}

void Wheel::SetInertia(float inertia) {
    m_inertia = inertia > 0.0f ? inertia : 0.0001f;
}

float Wheel::GetInertia() const {
    return m_inertia;
}

void Wheel::SetDriveTorque(float torque) {
    m_driveTorque = torque;
}

float Wheel::GetDriveTorque() const {
    return m_driveTorque;
}

void Wheel::SetBrakeTorque(float torque) {
    m_brakeTorque =
        std::max(0.0f, torque);
}

float Wheel::GetBrakeTorque() const {
    return m_brakeTorque;
}

void Wheel::Update(
    RigidBody& body,
    PhysicsWorld& physicsWorld,
    const Suspension& suspension,
    float deltaTime
) {
    const Vec3 worldMount =
        body.GetPosition() +
        body.GetOrientation() * m_localPosition;

    const Vec3 down =
        body.GetOrientation() *
        Vec3(0.0f, -1.0f, 0.0f);

    Ray ray;
    ray.origin = worldMount;
    ray.direction = down;

    RaycastResult result;

    const float maxRayDistance =
        suspension.GetMaxLength() + m_radius;

    if (!physicsWorld.Raycast(
        ray,
        result,
        maxRayDistance,
        &body
    )) {
        m_grounded = false;
        m_suspensionLength =
            suspension.GetMaxLength();
        m_compression = 0.0f;
        m_force = 0.0f;
        m_suspensionPower = 0.0f;
        m_suspensionResidual = 0.0f;
        m_hasPreviousCompression = false;
        m_contactNormal =
            Vec3(0.0f, 1.0f, 0.0f);

        m_worldPosition =
            worldMount +
            down * m_suspensionLength;

        m_contactPoint =
            m_worldPosition;

        return;
    }

    const float suspensionLength =
        suspension.ClampLength(
            result.distance - m_radius
        );

    const float compression =
        suspension.GetRestLength() -
        suspensionLength;

    const float denominator =
        result.normal.Dot(down);

    m_suspensionLength =
        suspensionLength;

    m_compression =
        std::max(0.0f, compression);

    m_suspensionPower = 0.0f;
    m_suspensionResidual = 0.0f;
    m_force = 0.0f;

    if (m_compression > 0.0f &&
        denominator < -0.1f) {
        const float inverseContactDotSuspension =
            -1.0f / denominator;

        const Vec3 contactVelocity =
            body.GetPointVelocity(result.point);

        const float projectedVelocity =
            result.normal.Dot(contactVelocity);

        const float compressionVelocity =
            projectedVelocity *
            inverseContactDotSuspension;

        m_force =
            suspension.CalculateForce(
                m_compression,
                compressionVelocity
            );

        const Vec3 suspensionForce =
            result.normal * m_force;

        const float springPower =
            suspension.GetSpringRate() *
            m_compression *
            compressionVelocity;

        const float damperPower =
            suspension.GetDamperRate() *
            compressionVelocity *
            compressionVelocity;

        m_suspensionPower =
            suspensionForce.Dot(
                contactVelocity
            );

        m_suspensionResidual =
            m_suspensionPower +
            springPower +
            damperPower;

        body.AddForceAtPoint(
            suspensionForce,
            result.point
        );
    }

    m_grounded = true;

    m_previousCompression = m_compression;
    m_hasPreviousCompression = true;

    m_contactPoint = result.point;
    m_contactNormal = result.normal;

    m_worldPosition =
        worldMount +
        down * m_suspensionLength;
}

void Wheel::ApplyTireForce(
    const RigidBody& body,
    const Vec3& force
) {
    if (!m_grounded)
        return;

    const Vec3 axle =
        body.GetOrientation() *
        Vec3(1.0f, 0.0f, 0.0f);

    const Vec3 radiusVector =
        m_contactPoint -
        m_worldPosition;

    const Vec3 torque =
        radiusVector.Cross(force);

    const float wheelTorque =
        torque.Dot(axle);

    m_driveTorque +=
        wheelTorque;
}

void Wheel::IntegrateRotation(float deltaTime) {
    if (deltaTime <= 0.0f)
        return;

    float brakeTorque = 0.0f;

    if (std::abs(m_angularVelocity) > 0.0001f)
        brakeTorque =
            m_brakeTorque *
            (m_angularVelocity > 0.0f ? 1.0f : -1.0f);

    const float netTorque =
        m_driveTorque -
        brakeTorque;

    m_angularVelocity +=
        (netTorque / m_inertia) *
        deltaTime;

    if (std::abs(m_angularVelocity) < 0.0001f)
        m_angularVelocity = 0.0f;

    m_rotationAngle +=
        m_angularVelocity * deltaTime;

    m_driveTorque = 0.0f;
}

bool Wheel::IsGrounded() const {
    return m_grounded;
}

float Wheel::GetSuspensionLength() const {
    return m_suspensionLength;
}

float Wheel::GetCompression() const {
    return m_compression;
}

float Wheel::GetForce() const {
    return m_force;
}

float Wheel::GetSuspensionPower() const {
    return m_suspensionPower;
}

float Wheel::GetSuspensionResidual() const {
    return m_suspensionResidual;
}

float Wheel::GetAngularVelocity() const {
    return m_angularVelocity;
}

float Wheel::GetRotationAngle() const {
    return m_rotationAngle;
}

const Vec3& Wheel::GetWorldPosition() const {
    return m_worldPosition;
}

const Vec3& Wheel::GetContactPoint() const {
    return m_contactPoint;
}

const Vec3& Wheel::GetContactNormal() const {
    return m_contactNormal;
}
