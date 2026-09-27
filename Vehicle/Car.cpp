#include "Car.h"

#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Core/Debug/Logger.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace {
    size_t ToIndex(WheelIndex index) {
        return static_cast<size_t>(index);
    }
}

Car::Car()
    : m_chassis(nullptr),
    m_throttle(0.0f),
    m_brake(0.0f),
    m_steering(0.0f),
    m_energyTimer(0.0f),
    m_brakeTimer(0.0f) {}

void Car::SetChassis(RigidBody* chassis) {
    m_chassis = chassis;

}

void Car::SetInput(
    float throttle,
    float brake,
    float steering
) {
    m_throttle = std::clamp(throttle, 0.0f, 1.0f);
    m_brake = std::clamp(brake, 0.0f, 1.0f);
    m_steering = std::clamp(steering, -1.0f, 1.0f);
}

void Car::UpdatePhysics(
    PhysicsWorld& physicsWorld,
    float deltaTime
) {
    if (!m_chassis)
        return;

    constexpr float MaxSteeringAngle = 0.5f;
    constexpr float DriveTorque = 1500.0f;
    constexpr float BrakeTorque = 2500.0f;

    for (size_t i = 0; i < WheelCount; ++i) {
        const WheelIndex index =
            static_cast<WheelIndex>(i);

        const bool frontWheel =
            index == WheelIndex::FrontLeft ||
            index == WheelIndex::FrontRight;

        m_wheels[i].SetSteeringAngle(
            frontWheel
                ? m_steering * MaxSteeringAngle
                : 0.0f
        );

        m_wheels[i].SetDriveTorque(
            m_throttle * DriveTorque
        );

        m_wheels[i].SetBrakeTorque(
            m_brake * BrakeTorque
        );
    }

    for (size_t i = 0; i < WheelCount; ++i) {
        m_wheels[i].Update(
            *m_chassis,
            physicsWorld,
            m_suspensions[i],
            deltaTime
        );

        const Vec3 tireForce =
            m_tires[i].CalculateForce(
                *m_chassis,
                m_wheels[i],
                deltaTime
            );

        if (tireForce.LengthSquared() > 0.0f) {
            m_chassis->AddForceAtPoint(
                tireForce,
                m_wheels[i].GetContactPoint()
            );

            m_wheels[i].ApplyTireForce(
                *m_chassis,
                tireForce
            );
        }

        m_wheels[i].IntegrateRotation(
            deltaTime
        );
    }

    m_energyTimer += deltaTime;
    m_brakeTimer += deltaTime;

    if (m_brake > 0.0f &&
        m_brakeTimer >= 0.1f) {
        const char* names[WheelCount] = {
            "FL", "FR", "RL", "RR"
        };

        for (size_t i = 0; i < WheelCount; ++i) {
            const Tire& tire = m_tires[i];
            const Wheel& wheel = m_wheels[i];

            std::ostringstream log;
            log << std::fixed << std::setprecision(3)
                << "[Brake] " << names[i]
                << " input=" << m_brake
                << " torque=" << wheel.GetBrakeTorque()
                << " omega=" << wheel.GetAngularVelocity()
                << " wheelSpeed=" << tire.GetWheelSurfaceSpeed()
                << " longVel=" << tire.GetLongitudinalVelocity()
                << " slipVel=" << tire.GetLongitudinalSlipVelocity()
                << " normal=" << tire.GetNormalLoad()
                << " longForce=" << tire.GetLongitudinalForce();

            Logger::Debug(log.str());
        }

        m_brakeTimer = 0.0f;
    }

    if (m_energyTimer >= 0.5f) {
        const float mass =
            m_chassis->GetMass();

        const float height =
            m_chassis->GetPosition().y;

        const Vec3 position =
            m_chassis->GetPosition();

        const Vec3 velocity =
            m_chassis->GetLinearVelocity();

        const Vec3 angularVelocity =
            m_chassis->GetAngularVelocity();

        const Mat3 rotation =
            m_chassis->GetOrientation().ToMat3();

        const Mat3 inertiaWorld =
            rotation *
            m_chassis->GetInertiaTensor() *
            rotation.Transposed();

        const float potentialEnergy =
            mass *
            std::abs(physicsWorld.GetGravity().y) *
            height;

        const float linearEnergy =
            0.5f *
            mass *
            velocity.LengthSquared();

        const float angularEnergy =
            0.5f *
            angularVelocity.Dot(
                inertiaWorld *
                angularVelocity
            );

        float wheelEnergy = 0.0f;
        float suspensionEnergy = 0.0f;
        float suspensionPower = 0.0f;
        float suspensionResidual = 0.0f;

        for (size_t i = 0; i < WheelCount; ++i) {
            const float wheelVelocity =
                m_wheels[i].GetAngularVelocity();

            wheelEnergy +=
                0.5f *
                m_wheels[i].GetInertia() *
                wheelVelocity *
                wheelVelocity;

            const float compression =
                m_wheels[i].GetCompression();

            suspensionEnergy +=
                0.5f *
                m_suspensions[i].GetSpringRate() *
                compression *
                compression;

            suspensionPower +=
                m_wheels[i].GetSuspensionPower();

            suspensionResidual +=
                m_wheels[i].GetSuspensionResidual();
        }

        const float totalEnergy =
            potentialEnergy +
            linearEnergy +
            angularEnergy +
            wheelEnergy +
            suspensionEnergy;

        std::ostringstream log;
        log << std::fixed << std::setprecision(3)
            << "[Energy] x=" << position.x
            << " y=" << position.y
            << " z=" << position.z
            << " vx=" << velocity.x
            << " vy=" << velocity.y
            << " vz=" << velocity.z
            << " total=" << totalEnergy
            << " potential=" << potentialEnergy
            << " linear=" << linearEnergy
            << " angular=" << angularEnergy
            << " wheels=" << wheelEnergy
            << " suspension=" << suspensionEnergy
            << " suspensionPower=" << suspensionPower
            << " suspensionResidual=" << suspensionResidual;

        Logger::Debug(log.str());

        m_energyTimer = 0.0f;
    }
}

Wheel& Car::GetWheel(WheelIndex index) {
    return m_wheels[ToIndex(index)];
}

const Wheel& Car::GetWheel(WheelIndex index) const {
    return m_wheels[ToIndex(index)];
}

Suspension& Car::GetSuspension(WheelIndex index) {
    return m_suspensions[ToIndex(index)];
}

const Suspension& Car::GetSuspension(WheelIndex index) const {
    return m_suspensions[ToIndex(index)];
}

Tire& Car::GetTire(WheelIndex index) {
    return m_tires[ToIndex(index)];
}

const Tire& Car::GetTire(WheelIndex index) const {
    return m_tires[ToIndex(index)];
}

RigidBody* Car::GetChassis() {
    return m_chassis;
}

const RigidBody* Car::GetChassis() const {
    return m_chassis;
}
