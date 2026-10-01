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
    m_clutch(0.0f),
    m_energyTimer(0.0f),
    m_brakeTimer(0.0f),
    m_tireTimer(0.0f),
    m_brakeTorque(2500.0f),
    m_maxSteeringAngle(0.5f) {}

void Car::SetChassis(RigidBody* chassis) {
    m_chassis = chassis;
}

void Car::ApplyConfig(const VehicleConfig& config) {
    m_brakeTorque = config.brakeTorque;
    m_maxSteeringAngle = config.maxSteeringAngle;

    for (size_t i = 0; i < WheelCount; ++i) {
        m_wheels[i].SetLocalPosition(config.wheelPositions[i]);
        m_wheels[i].SetRadius(config.wheelRadius);
        m_wheels[i].SetInertia(config.wheelInertia);

        Suspension& suspension = m_suspensions[i];
        suspension.SetRestLength(config.suspensionRestLength);
        suspension.SetBumpTravel(config.suspensionBumpTravel);
        suspension.SetReboundTravel(config.suspensionReboundTravel);
        suspension.SetSpringRate(i < 2 ? config.frontSpringRate : config.rearSpringRate);
        suspension.SetCompressionDamperRate(i < 2 ? config.frontCompressionDamping : config.rearCompressionDamping);
        suspension.SetReboundDamperRate(i < 2 ? config.frontReboundDamping : config.rearReboundDamping);

        m_tires[i].SetStaticFriction(config.staticFriction);
        m_tires[i].SetDynamicFriction(config.dynamicFriction);
        m_tires[i].SetLongitudinalStiffness(config.longitudinalStiffness);
        m_tires[i].SetLateralStiffness(config.lateralStiffness);
        m_tires[i].SetRollingResistance(config.rollingResistance);
    }

    m_powertrain.GetEngine().Configure(
        config.idleRPM, config.stallRPM, config.redlineRPM,
        config.peakTorque, config.engineInertia
    );
    m_powertrain.GetTransmission().Configure(
        config.gearRatios, config.reverseRatio, config.finalDriveRatio
    );
}

void Car::SetInput(
    float throttle,
    float brake,
    float steering,
    float clutch
) {
    m_throttle = std::clamp(throttle, 0.0f, 1.0f);
    m_brake = std::clamp(brake, 0.0f, 1.0f);
    m_steering = std::clamp(steering, -1.0f, 1.0f);
    m_clutch = std::clamp(clutch, 0.0f, 1.0f);
}

void Car::UpdatePhysics(
    PhysicsWorld& physicsWorld,
    float deltaTime
) {
    if (!m_chassis)
        return;

    const float rearWheelAngularVelocity =
        0.5f * (
            m_wheels[ToIndex(WheelIndex::RearLeft)].GetAngularVelocity() +
            m_wheels[ToIndex(WheelIndex::RearRight)].GetAngularVelocity()
        );

    m_powertrain.Update(
        m_throttle,
        m_clutch,
        rearWheelAngularVelocity,
        deltaTime
    );

    const float leftDriveTorque =
        m_powertrain.GetLeftDriveTorque();
    const float rightDriveTorque =
        m_powertrain.GetRightDriveTorque();

    m_wheels[ToIndex(WheelIndex::FrontLeft)].SetSteeringAngle(
        m_steering * m_maxSteeringAngle
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
            m_brake * m_brakeTorque
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
                << " Fx=" << tire.GetLongitudinalForce()
                << " Fy=" << tire.GetLateralForce();

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
    return m_powertrain.GetEngine();
}

const Engine& Car::GetEngine() const {
    return m_powertrain.GetEngine();
}

Transmission& Car::GetTransmission() {
    return m_powertrain.GetTransmission();
}

const Transmission& Car::GetTransmission() const {
    return m_powertrain.GetTransmission();
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

float Car::GetThrottle() const {
    return m_throttle;
}

float Car::GetBrake() const {
    return m_brake;
}

float Car::GetSteering() const {
    return m_steering;
}

float Car::GetClutch() const {
    return m_clutch;
}
