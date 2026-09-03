#include "RigidBody.h"

RigidBody::RigidBody()
    : m_position(0.0f, 0.0f, 0.0f),
    m_orientation(Quaternion::Identity()),
    m_linearVelocity(0.0f, 0.0f, 0.0f),
    m_angularVelocity(0.0f, 0.0f, 0.0f),
    m_force(0.0f, 0.0f, 0.0f),
    m_torque(0.0f, 0.0f, 0.0f),
    m_mass(1.0f),
    m_inverseMass(1.0f),
    m_inertiaTensor(Mat3::Identity()),
    m_inverseInertiaTensor(Mat3::Identity())
{}

void RigidBody::SetMass(float mass)
{
    if (mass <= 0.0f) {
        m_mass = 0.0f;
        m_inverseMass = 0.0f;
        return;
    }

    m_mass = mass;
    m_inverseMass = 1.0f / mass;
}

float RigidBody::GetMass() const
{
    return m_mass;
}

float RigidBody::GetInverseMass() const
{
    return m_inverseMass;
}

void RigidBody::SetPosition(const Vec3& position)
{
    m_position = position;
}

const Vec3& RigidBody::GetPosition() const
{
    return m_position;
}

void RigidBody::SetOrientation(
    const Quaternion& orientation
)
{
    m_orientation = orientation.Normalized();
}

const Quaternion& RigidBody::GetOrientation() const
{
    return m_orientation;
}

void RigidBody::SetLinearVelocity(
    const Vec3& velocity
)
{
    m_linearVelocity = velocity;
}

const Vec3& RigidBody::GetLinearVelocity() const
{
    return m_linearVelocity;
}

void RigidBody::SetAngularVelocity(
    const Vec3& velocity
)
{
    m_angularVelocity = velocity;
}

const Vec3& RigidBody::GetAngularVelocity() const
{
    return m_angularVelocity;
}

void RigidBody::AddForce(const Vec3& force)
{
    m_force += force;
}

void RigidBody::AddTorque(const Vec3& torque)
{
    m_torque += torque;
}

const Vec3& RigidBody::GetForce() const
{
    return m_force;
}

const Vec3& RigidBody::GetTorque() const
{
    return m_torque;
}

void RigidBody::ClearForces()
{
    m_force = Vec3(0.0f, 0.0f, 0.0f);
    m_torque = Vec3(0.0f, 0.0f, 0.0f);
}

void RigidBody::SetInertiaTensor(
    const Mat3& inertiaTensor
)
{
    m_inertiaTensor = inertiaTensor;
}

const Mat3& RigidBody::GetInertiaTensor() const
{
    return m_inertiaTensor;
}

void RigidBody::SetInverseInertiaTensor(
    const Mat3& inverseInertiaTensor
)
{
    m_inverseInertiaTensor = inverseInertiaTensor;
}

const Mat3& RigidBody::GetInverseInertiaTensor() const
{
    return m_inverseInertiaTensor;
}

void RigidBody::Integrate(
    float deltaTime,
    const Vec3& gravity
) {
    if (deltaTime <= 0.0f)
        return;

    if (m_inverseMass <= 0.0f)
        return;

    const Vec3 linearAcceleration =
        gravity +
        m_force * m_inverseMass;

    m_linearVelocity +=
        linearAcceleration * deltaTime;

    m_position +=
        m_linearVelocity * deltaTime;

    const Vec3 angularAcceleration =
        m_inverseInertiaTensor * m_torque;

    m_angularVelocity +=
        angularAcceleration * deltaTime;

    const Quaternion angularVelocityQuaternion(
        0.0f,
        m_angularVelocity.x,
        m_angularVelocity.y,
        m_angularVelocity.z
    );

    const Quaternion orientationDelta =
        angularVelocityQuaternion *
        m_orientation;

    m_orientation.w +=
        0.5f * orientationDelta.w * deltaTime;

    m_orientation.x +=
        0.5f * orientationDelta.x * deltaTime;

    m_orientation.y +=
        0.5f * orientationDelta.y * deltaTime;

    m_orientation.z +=
        0.5f * orientationDelta.z * deltaTime;

    m_orientation.Normalize();

    ClearForces();
}