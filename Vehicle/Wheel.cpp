#include "Wheel.h"

#include "Suspension.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Core/Debug/Logger.h"

#include <algorithm>
#include <cmath>
#include <string>

Wheel::Wheel()
    : m_localPosition(0.0f, 0.0f, 0.0f),
    m_radius(0.5f),
    m_inertia(1.8f),
    m_driveTorque(0.0f),
    m_brakeTorque(0.0f),
    m_steeringAngle(0.0f),
    m_angularVelocity(0.0f),
    m_rotationAngle(0.0f),
    m_tireReactionTorque(0.0f),
    m_grounded(false),
    m_suspensionLength(0.0f),
    m_lastRayDistance(0.0f),
    m_lastRayShape(-1),
    m_compression(0.0f),
    m_previousCompression(0.0f),
    m_force(0.0f),
    m_normalLoad(0.0f),
    m_compressionVelocity(0.0f),
    m_springForce(0.0f),
    m_damperForce(0.0f),
    m_suspensionPower(0.0f),
    m_suspensionResidual(0.0f),
    m_hasPreviousCompression(false),
    m_worldPosition(0.0f, 0.0f, 0.0f),
    m_worldOrientation(Quaternion::Identity()),
    m_contactPoint(0.0f, 0.0f, 0.0f),
    m_contactNormal(0.0f, 1.0f, 0.0f) {}

void Wheel::SetLocalPosition(const Vec3& position) {
    m_localPosition = position;
}

void Wheel::SetHubPosition(const Vec3& position) {
    m_worldPosition = position;
}

void Wheel::SetHubState(
    const Vec3& position,
    const Quaternion& orientation
) {
    m_worldPosition = position;
    m_worldOrientation = orientation;
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

void Wheel::SetSteeringAngle(float angle) {
    m_steeringAngle = -angle;
}

float Wheel::GetSteeringAngle() const {
    return m_steeringAngle;
}

void Wheel::Update(
    int index,
    RigidBody& body,
    PhysicsWorld& physicsWorld,
    const Suspension& suspension,
    float deltaTime
) {
    const Vec3 worldMount = m_worldPosition;

    // Suspension travel is world-vertical. The wheel mount follows the
    // chassis, but compression itself must not introduce lateral or
    // longitudinal wheel movement when the body rolls.
    const Vec3 down(0.0f, -1.0f, 0.0f);

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

        constexpr float FreeReboundSpeed = 4.0f;
        m_suspensionLength =
            std::min(
                suspension.GetMaxLength(),
                m_suspensionLength +
                    FreeReboundSpeed * deltaTime
            );
        m_lastRayDistance = maxRayDistance;
        m_lastRayShape = -1;
        m_compression = 0.0f;
        m_force = 0.0f;
        m_normalLoad = 0.0f;
        m_compressionVelocity = 0.0f;
        m_springForce = 0.0f;
        m_damperForce = 0.0f;
        m_suspensionPower = 0.0f;
        m_suspensionResidual = 0.0f;
        m_hasPreviousCompression = false;
        m_contactNormal =
            Vec3(0.0f, 1.0f, 0.0f);
        m_tireReactionTorque = 0.0f;

        m_contactPoint = m_worldPosition;

        return;
    }

    m_lastRayDistance = result.distance;
    const float rawSuspensionLength =
        result.distance - m_radius;
    const float suspensionLength =
        suspension.ClampLength(rawSuspensionLength);
    m_lastRayShape =
        result.collider
        ? static_cast<int>(result.collider->GetShape())
        : -1;

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
    m_normalLoad = 0.0f;
    m_compressionVelocity = 0.0f;
    m_springForce = 0.0f;
    m_damperForce = 0.0f;
    m_tireReactionTorque = 0.0f;

    if (m_compression > 0.0f &&
        denominator < -0.1f) {
        const Vec3 mountVelocity =
            body.GetPointVelocity(worldMount);

        const Vec3 suspensionDirectionVelocity =
            body.GetAngularVelocity().Cross(down);

        const Vec3 rayPointVelocity =
            mountVelocity +
            suspensionDirectionVelocity *
            result.distance;

        const float projectedVelocity =
            result.normal.Dot(rayPointVelocity);

        m_compressionVelocity =
            projectedVelocity / denominator;

        m_springForce =
            suspension.GetSpringRate() *
            m_compression;

        const float damperRate = m_compressionVelocity >= 0.0f
            ? suspension.GetCompressionDamperRate()
            : suspension.GetReboundDamperRate();
        m_damperForce = damperRate * m_compressionVelocity;

        m_force =
            std::max(
                0.0f,
                m_springForce + m_damperForce
            );

        // The suspension force acts along the suspension axis. Only its
        // component along the contact normal contributes to normal load.
        // Since down points into the ground, -dot(normal, down) is the
        // positive projection factor for a valid suspension contact.
        m_normalLoad =
            m_force * -denominator;

        // The suspension transmits force along its own axis.
        // Keep the contact-normal load for the tire model separate.
        const Vec3 suspensionForce = down * -m_force;

        const float springPower =
            m_springForce *
            m_compressionVelocity;

        const float damperPower =
            damperRate *
            m_compressionVelocity *
            m_compressionVelocity;

        m_suspensionPower =
            suspensionForce.Dot(
                body.GetPointVelocity(worldMount)
            );

        m_suspensionResidual =
            m_suspensionPower +
            springPower +
            damperPower;

        body.AddForceAtPoint(
            suspensionForce,
            worldMount
        );
    }

    m_grounded = true;

    m_previousCompression = m_compression;
    m_hasPreviousCompression = true;

    m_contactPoint = result.point;
    m_contactNormal = result.normal;


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

    m_tireReactionTorque =
        torque.Dot(axle);

    m_driveTorque +=
        m_tireReactionTorque;
}

void Wheel::IntegrateRotation(float deltaTime) {
    if (deltaTime <= 0.0f)
        return;

    const float angularVelocity = m_angularVelocity;
    const float driveTorque = m_driveTorque;

    if (std::abs(angularVelocity) <= 0.0001f) {
        if (std::abs(driveTorque) <= m_brakeTorque) {
            m_angularVelocity = 0.0f;
        } else {
            const float brakeDirection =
                driveTorque > 0.0f ? 1.0f : -1.0f;

            const float netTorque =
                driveTorque -
                m_brakeTorque * brakeDirection;

            m_angularVelocity =
                (netTorque / m_inertia) *
                deltaTime;
        }
    } else {
        const float brakeDirection =
            angularVelocity > 0.0f ? 1.0f : -1.0f;

        const float netTorque =
            driveTorque -
            m_brakeTorque * brakeDirection;

        const float newAngularVelocity =
            angularVelocity +
            (netTorque / m_inertia) *
            deltaTime;

        if ((angularVelocity > 0.0f &&
             newAngularVelocity < 0.0f &&
             driveTorque <= m_brakeTorque) ||
            (angularVelocity < 0.0f &&
             newAngularVelocity > 0.0f &&
             driveTorque >= -m_brakeTorque)) {
            m_angularVelocity = 0.0f;
        } else {
            m_angularVelocity =
                newAngularVelocity;
        }
    }

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

float Wheel::GetLastRayDistance() const {
    return m_lastRayDistance;
}

int Wheel::GetLastRayShape() const {
    return m_lastRayShape;
}

float Wheel::GetCompression() const {
    return m_compression;
}

float Wheel::GetForce() const {
    return m_force;
}

float Wheel::GetNormalLoad() const {
    return m_normalLoad;
}

float Wheel::GetCompressionVelocity() const {
    return m_compressionVelocity;
}

float Wheel::GetSpringForce() const {
    return m_springForce;
}

float Wheel::GetDamperForce() const {
    return m_damperForce;
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

float Wheel::GetTireReactionTorque() const {
    return m_tireReactionTorque;
}

const Vec3& Wheel::GetWorldPosition() const {
    return m_worldPosition;
}

const Quaternion& Wheel::GetWorldOrientation() const {
    return m_worldOrientation;
}

const Vec3& Wheel::GetContactPoint() const {
    return m_contactPoint;
}

const Vec3& Wheel::GetContactNormal() const {
    return m_contactNormal;
}
