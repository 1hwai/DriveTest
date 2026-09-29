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
    m_brakeTimer(0.0f),
    m_tireTimer(0.0f) {}

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
    constexpr float BrakeTorque = 2500.0f;
    constexpr float ClutchStiffness = 8.0f;
    constexpr float MaxClutchTorque = 250.0f;

    const float rearLoadTorque =
        m_differential.GetInputLoadTorque(
            m_wheels[ToIndex(WheelIndex::RearLeft)].GetTireReactionTorque(),
            m_wheels[ToIndex(WheelIndex::RearRight)].GetTireReactionTorque()
        );

    const float engineLoadTorque =
        m_transmission.GetInputLoadTorque(
            rearLoadTorque
        );

    float drivetrainTorque = 0.0f;

    const float gearRatio =
        m_transmission.GetGearRatio();

    if (std::abs(gearRatio) > 0.0001f) {
        const float rearWheelAngularVelocity =
            0.5f * (
                m_wheels[ToIndex(WheelIndex::RearLeft)].GetAngularVelocity() +
                m_wheels[ToIndex(WheelIndex::RearRight)].GetAngularVelocity()
            );

        const float targetEngineAngularVelocity =
            rearWheelAngularVelocity *
            gearRatio *
            m_transmission.GetFinalDriveRatio();

        const float idleAngularVelocity =
            m_engine.GetIdleRPM() *
            2.0f *
            3.14159265358979323846f /
            60.0f;

        const float clutchEngagement =
            std::clamp(
                std::abs(targetEngineAngularVelocity) /
                (idleAngularVelocity * 1.25f),
                0.0f,
                1.0f
            );

        const float angularVelocityError =
            targetEngineAngularVelocity -
            m_engine.GetAngularVelocity();

        drivetrainTorque =
            std::clamp(
                angularVelocityError *
                ClutchStiffness *
                clutchEngagement,
                -MaxClutchTorque,
                MaxClutchTorque
            );
    }

    m_engine.Update(
        m_throttle,
        engineLoadTorque,
        drivetrainTorque,
        deltaTime
    );

    const float transmissionTorque =
        m_transmission.GetOutputTorque(
            m_engine.GetTorque()
        );

    float leftDriveTorque = 0.0f;
    float rightDriveTorque = 0.0f;

    m_differential.DistributeTorque(
        transmissionTorque,
        leftDriveTorque,
        rightDriveTorque
    );

    const float clutchReactionTorque =
        -drivetrainTorque *
        gearRatio *
        m_transmission.GetFinalDriveRatio();

    m_differential.DistributeTorque(
        clutchReactionTorque,
        leftDriveTorque,
        rightDriveTorque
    );

    m_wheels[ToIndex(WheelIndex::FrontLeft)].SetSteeringAngle(
        m_steering * MaxSteeringAngle
    );
    m_wheels[ToIndex(WheelIndex::FrontRight)].SetSteeringAngle(
        m_steering * MaxSteeringAngle
    );

    m_wheels[ToIndex(WheelIndex::FrontLeft)].SetDriveTorque(0.0f);
    m_wheels[ToIndex(WheelIndex::FrontRight)].SetDriveTorque(0.0f);
    m_wheels[ToIndex(WheelIndex::RearLeft)].SetDriveTorque(
        leftDriveTorque
    );
    m_wheels[ToIndex(WheelIndex::RearRight)].SetDriveTorque(
        rightDriveTorque
    );

    for (size_t i = 0; i < WheelCount; ++i) {
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

        const TireState tireState =
            m_tires[i].CalculateState(
                *m_chassis,
                m_wheels[i]
            );

        const Vec3 tireForce =
            m_tires[i].CalculateForce(
                tireState
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
    m_tireTimer += deltaTime;

    if (m_tireTimer >= 0.1f) {
        const char* names[WheelCount] = {
            "FL", "FR", "RL", "RR"
        };

        for (size_t i = 0; i < WheelCount; ++i) {
            const Tire& tire = m_tires[i];
            const Wheel& wheel = m_wheels[i];

            std::ostringstream log;
            log << std::fixed << std::setprecision(3)
                << "[Tire] " << names[i]
                << " grounded=" << wheel.IsGrounded()
                << " omega=" << wheel.GetAngularVelocity()
                << " wheelSpeed=" << tire.GetWheelSurfaceSpeed()
                << " Vx=" << tire.GetLongitudinalVelocity()
                << " Vy=" << tire.GetLateralVelocity()
                << " slipVel=" << tire.GetLongitudinalSlipVelocity()
                << " kappa=" << tire.GetSlipRatio()
                << " alpha=" << tire.GetSlipAngle()
                << " Fz=" << tire.GetNormalLoad()
                << " Fx=" << tire.GetLongitudinalForce();

            Logger::Debug(log.str());
        }

        m_tireTimer = 0.0f;
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

Engine& Car::GetEngine() {
    return m_engine;
}

const Engine& Car::GetEngine() const {
    return m_engine;
}

Transmission& Car::GetTransmission() {
    return m_transmission;
}

const Transmission& Car::GetTransmission() const {
    return m_transmission;
}

float Car::GetSpeedKmh() const {
    if (!m_chassis)
        return 0.0f;

    const Vec3 velocity = m_chassis->GetLinearVelocity();

    return std::sqrt(
        velocity.x * velocity.x +
        velocity.z * velocity.z
    ) * 3.6f;
}
