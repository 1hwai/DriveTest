#include "RigidBody.h"

#include <cmath>

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
    m_inverseInertiaTensor(Mat3::Identity()),
    m_isSleeping(false),
    m_sleepTimer(0.0f) {}

void RigidBody::SetMass(float mass) {
    if (mass <= 0.0f) {
        m_mass = 0.0f;
        m_inverseMass = 0.0f;
        m_isSleeping = false;
        m_sleepTimer = 0.0f;
        return;
    }

    m_mass = mass;
    m_inverseMass = 1.0f / mass;

    Wake();
}

float RigidBody::GetMass() const {
    return m_mass;
}

float RigidBody::GetInverseMass() const {
    return m_inverseMass;
}

void RigidBody::SetBoxInertia(const Vec3& size) {
    float x2 = size.x * size.x;
    float y2 = size.y * size.y;
    float z2 = size.z * size.z;

    float factor = m_mass / 12.0f;

    float ix = factor * (y2 + z2);
    float iy = factor * (x2 + z2);
    float iz = factor * (x2 + y2);

    m_inertiaTensor =
        Mat3::Diagonal(Vec3(ix, iy, iz));

    constexpr float epsilon = 0.000001f;

    float iix = ix > epsilon ? 1.0f / ix : 0.0f;
    float iiy = iy > epsilon ? 1.0f / iy : 0.0f;
    float iiz = iz > epsilon ? 1.0f / iz : 0.0f;

    m_inverseInertiaTensor =
        Mat3::Diagonal(Vec3(iix, iiy, iiz));
}

void RigidBody::SetPosition(const Vec3& position) {
    m_position = position;
}

const Vec3& RigidBody::GetPosition() const {
    return m_position;
}

void RigidBody::SetOrientation(const Quaternion& orientation) {
    m_orientation = orientation;
    m_orientation.Normalize();
}

const Quaternion& RigidBody::GetOrientation() const {
    return m_orientation;
}

void RigidBody::SetLinearVelocity(const Vec3& velocity) {
    m_linearVelocity = velocity;
    Wake();
}

const Vec3& RigidBody::GetLinearVelocity() const {
    return m_linearVelocity;
}

void RigidBody::SetAngularVelocity(const Vec3& velocity) {
    m_angularVelocity = velocity;
    Wake();
}

const Vec3& RigidBody::GetAngularVelocity() const {
    return m_angularVelocity;
}

void RigidBody::AddForce(const Vec3& force) {
    m_force += force;
    Wake();
}

void RigidBody::AddTorque(const Vec3& torque) {
    m_torque += torque;
    Wake();
}

const Vec3& RigidBody::GetForce() const {
    return m_force;
}

const Vec3& RigidBody::GetTorque() const {
    return m_torque;
}

void RigidBody::ClearForces() {
    m_force = Vec3(0.0f, 0.0f, 0.0f);
    m_torque = Vec3(0.0f, 0.0f, 0.0f);
}

void RigidBody::SetInertiaTensor(
    const Mat3& inertiaTensor
) {
    m_inertiaTensor = inertiaTensor;
}

const Mat3& RigidBody::GetInertiaTensor() const {
    return m_inertiaTensor;
}

void RigidBody::SetInverseInertiaTensor(
    const Mat3& inverseInertiaTensor
) {
    m_inverseInertiaTensor = inverseInertiaTensor;
}

const Mat3& RigidBody::GetInverseInertiaTensor() const {
    return m_inverseInertiaTensor;
}

Mat3 RigidBody::GetWorldInverseInertiaTensor() const {
    Mat3 rotation =
        m_orientation.ToMat3();

    return rotation *
        m_inverseInertiaTensor *
        rotation.Transposed();
}

void RigidBody::Integrate(
    float deltaTime,
    const Vec3& gravity
) {
    if (deltaTime <= 0.0f)
        return;

    if (m_inverseMass <= 0.0f)
        return;

    if (m_isSleeping)
        return;

    Vec3 acceleration =
        gravity +
        m_force * m_inverseMass;

    m_linearVelocity +=
        acceleration * deltaTime;

    m_position +=
        m_linearVelocity * deltaTime;

    Mat3 worldInverseInertia =
        GetWorldInverseInertiaTensor();

    Vec3 angularAcceleration =
        worldInverseInertia * m_torque;

    m_angularVelocity +=
        angularAcceleration * deltaTime;

    constexpr float AngularVelocitySleepEpsilonSq =
        1e-8f;

    if (m_angularVelocity.LengthSquared() <
        AngularVelocitySleepEpsilonSq) {
        m_angularVelocity =
            Vec3(0.0f, 0.0f, 0.0f);
    }

    Quaternion angularVelocityQuat(
        0.0f,
        m_angularVelocity.x,
        m_angularVelocity.y,
        m_angularVelocity.z
    );

    Quaternion orientationDerivative =
        angularVelocityQuat *
        m_orientation;

    m_orientation.w +=
        0.5f *
        orientationDerivative.w *
        deltaTime;

    m_orientation.x +=
        0.5f *
        orientationDerivative.x *
        deltaTime;

    m_orientation.y +=
        0.5f *
        orientationDerivative.y *
        deltaTime;

    m_orientation.z +=
        0.5f *
        orientationDerivative.z *
        deltaTime;

    m_orientation.Normalize();

    ClearForces();
}

void RigidBody::UpdateSleep(float deltaTime) {
    if (m_inverseMass <= 0.0f)
        return;

    if (m_isSleeping)
        return;

    constexpr float LinearThresholdSq =
        SleepLinearVelocityThreshold *
        SleepLinearVelocityThreshold;

    constexpr float AngularThresholdSq =
        SleepAngularVelocityThreshold *
        SleepAngularVelocityThreshold;

    if (m_linearVelocity.LengthSquared() <
        LinearThresholdSq &&
        m_angularVelocity.LengthSquared() <
        AngularThresholdSq) {

        m_sleepTimer += deltaTime;

        if (m_sleepTimer >= SleepTimeThreshold)
            Sleep();
    }
    else {
        m_sleepTimer = 0.0f;
    }
}

void RigidBody::Wake() {
    if (m_inverseMass <= 0.0f)
        return;

    m_isSleeping = false;
    m_sleepTimer = 0.0f;
}

void RigidBody::Sleep() {
    if (m_inverseMass <= 0.0f)
        return;

    m_isSleeping = true;
    m_sleepTimer = SleepTimeThreshold;

    m_linearVelocity =
        Vec3(0.0f, 0.0f, 0.0f);

    m_angularVelocity =
        Vec3(0.0f, 0.0f, 0.0f);

    ClearForces();
}

bool RigidBody::IsSleeping() const {
    return m_isSleeping;
}