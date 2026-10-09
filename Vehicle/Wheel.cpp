#include "Wheel.h"

#include "Suspension.h"
#include "IWheelContactProvider.h"
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
    m_lastContactDistance(0.0f),
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
    const IWheelContactProvider& contactProvider,
    const PhysicsWorld& physicsWorld,
    const Suspension& suspension,
    float deltaTime
) {
    (void)index;
    const Vec3 worldMount = m_worldPosition;
    const float maxReach = suspension.GetMaxLength() + m_radius;
    const WheelContactInput input{
        worldMount,
        m_worldOrientation,
        m_radius,
        maxReach
    };

    m_contactResult = contactProvider.Query(
        input,
        physicsWorld,
        &body
    );

    if (!m_contactResult.HasContact()) {
        m_grounded = false;

        constexpr float FreeReboundSpeed = 4.0f;
        m_suspensionLength =
            std::min(
                suspension.GetMaxLength(),
                m_suspensionLength + FreeReboundSpeed * deltaTime
            );
        m_lastContactDistance = maxReach;
        m_compression = 0.0f;
        m_force = 0.0f;
        m_normalLoad = 0.0f;
        m_compressionVelocity = 0.0f;
        m_springForce = 0.0f;
        m_damperForce = 0.0f;
        m_suspensionPower = 0.0f;
        m_suspensionResidual = 0.0f;
        m_hasPreviousCompression = false;
        m_contactNormal = Vec3(0.0f, 1.0f, 0.0f);
        m_tireReactionTorque = 0.0f;
        m_contactPoint = m_worldPosition;
        return;
    }

    const WheelContactSample& contact =
        m_contactResult.samples.front();
    m_lastContactDistance = contact.hasQueryDistance
        ? contact.queryDistance
        : (contact.point - worldMount).Length();

    // Preserve the existing world-vertical suspension behavior for this
    // provider-boundary work unit. Suspension mount-point kinematics are a
    // separate follow-up; this value is geometric separation, not a query API.
    const Vec3 up(0.0f, 1.0f, 0.0f);
    const Vec3 down(0.0f, -1.0f, 0.0f);
    const float contactOffset =
        std::max(0.0f, (worldMount - contact.point).Dot(up));
    const float rawSuspensionLength =
        contactOffset - m_radius;
    const float suspensionLength =
        suspension.ClampLength(rawSuspensionLength);

    const float compression =
        suspension.GetRestLength() - suspensionLength;
    const float denominator =
        contact.normal.Dot(down);

    m_suspensionLength = suspensionLength;
    m_compression = std::max(0.0f, compression);
    m_suspensionPower = 0.0f;
    m_suspensionResidual = 0.0f;
    m_force = 0.0f;
    m_normalLoad = 0.0f;
    m_compressionVelocity = 0.0f;
    m_springForce = 0.0f;
    m_damperForce = 0.0f;
    m_tireReactionTorque = 0.0f;

    if (m_compression > 0.0f && denominator < -0.1f) {
        const Vec3 mountVelocity =
            body.GetPointVelocity(worldMount);
        const Vec3 suspensionDirectionVelocity =
            body.GetAngularVelocity().Cross(down);
        const Vec3 contactPointVelocity =
            mountVelocity +
            suspensionDirectionVelocity * contactOffset;
        const float projectedVelocity =
            contact.normal.Dot(contactPointVelocity);

        m_compressionVelocity =
            projectedVelocity / denominator;
        m_springForce =
            suspension.GetSpringRate() * m_compression;

        const float damperRate = m_compressionVelocity >= 0.0f
            ? suspension.GetCompressionDamperRate()
            : suspension.GetReboundDamperRate();
        m_damperForce = damperRate * m_compressionVelocity;
        m_force = std::max(0.0f, m_springForce + m_damperForce);
        m_normalLoad = m_force * -denominator;

        const Vec3 suspensionForce = down * -m_force;
        const float springPower =
            m_springForce * m_compressionVelocity;
        const float damperPower =
            damperRate *
            m_compressionVelocity *
            m_compressionVelocity;

        m_suspensionPower =
            suspensionForce.Dot(body.GetPointVelocity(worldMount));
        m_suspensionResidual =
            m_suspensionPower + springPower + damperPower;

        body.AddForceAtPoint(suspensionForce, worldMount);
    }

    m_grounded = true;
    m_previousCompression = m_compression;
    m_hasPreviousCompression = true;
    m_contactPoint = contact.point;
    m_contactNormal = contact.normal;
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

float Wheel::GetLastContactDistance() const {
    return m_lastContactDistance;
}

const WheelContactResult& Wheel::GetContactResult() const {
    return m_contactResult;
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
