#include "Car.h"

#include "RunningGear/RunningGear.h"
#include "RunningGear/IWheelContactProvider.h"
#include "RunningGear/ISuspensionGeometry.h"
#include "RunningGear/Wheel.h"
#include "RunningGear/Suspension.h"
#include "RunningGear/Tire.h"
#include "Powertrain/Powertrain.h"
#include "Powertrain/Engine.h"
#include "Powertrain/Transmission.h"
#include "VehicleConfig.h"

#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Core/Math/Vec3.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

Car::Car()
    : Car(std::unique_ptr<IWheelContactProvider>{}) {}

Car::Car(std::unique_ptr<IWheelContactProvider> contactProvider)
    : m_runningGear(std::make_unique<RunningGear>(std::move(contactProvider))),
      m_powertrain(std::make_unique<Powertrain>()) {}

Car::~Car() = default;

void Car::SetChassis(RigidBody* chassis) {
    m_runningGear->SetChassis(chassis);
}

void Car::ApplyConfig(const VehicleConfig& config) {
    m_runningGear->ApplyConfig(config);
    m_powertrain->GetEngine().Configure(
        config.idleRPM, config.stallRPM, config.redlineRPM,
        config.peakTorque, config.engineInertia
    );
    m_powertrain->GetTransmission().Configure(
        config.gearRatios, config.reverseRatio, config.finalDriveRatio
    );
}

void Car::SetInput(float throttle, float brake, float steerInput, float clutch) {
    m_throttle = std::clamp(throttle, 0.0f, 1.0f);
    m_brake = std::clamp(brake, 0.0f, 1.0f);
    m_steering = std::clamp(steerInput, -1.0f, 1.0f);
    m_clutch = std::clamp(clutch, 0.0f, 1.0f);
}

void Car::UpdatePhysics(PhysicsWorld& physicsWorld, float deltaTime) {
    if (!m_runningGear->GetChassis())
        return;

    m_powertrain->Update(
        m_throttle,
        m_clutch,
        m_runningGear->GetAverageWheelAngularVelocity(),
        deltaTime
    );

    m_runningGear->UpdatePhysics(
        physicsWorld,
        m_steering,
        m_brake,
        m_powertrain->GetLeftDriveTorque(),
        m_powertrain->GetRightDriveTorque(),
        deltaTime
    );
}

ISuspensionGeometry& Car::GetSuspensionGeometry(WheelIndex index) {
    return m_runningGear->GetSuspensionGeometry(index);
}
const ISuspensionGeometry& Car::GetSuspensionGeometry(WheelIndex index) const {
    return m_runningGear->GetSuspensionGeometry(index);
}
Wheel& Car::GetWheel(WheelIndex index) { return m_runningGear->GetWheel(index); }
const Wheel& Car::GetWheel(WheelIndex index) const { return m_runningGear->GetWheel(index); }
Suspension& Car::GetSuspension(WheelIndex index) { return m_runningGear->GetSuspension(index); }
const Suspension& Car::GetSuspension(WheelIndex index) const { return m_runningGear->GetSuspension(index); }
Tire& Car::GetTire(WheelIndex index) { return m_runningGear->GetTire(index); }
const Tire& Car::GetTire(WheelIndex index) const { return m_runningGear->GetTire(index); }
RigidBody* Car::GetChassis() { return m_runningGear->GetChassis(); }
const RigidBody* Car::GetChassis() const { return m_runningGear->GetChassis(); }
Engine& Car::GetEngine() { return m_powertrain->GetEngine(); }
const Engine& Car::GetEngine() const { return m_powertrain->GetEngine(); }
Transmission& Car::GetTransmission() { return m_powertrain->GetTransmission(); }
const Transmission& Car::GetTransmission() const { return m_powertrain->GetTransmission(); }

float Car::GetSpeedKmh() const {
    const RigidBody* chassis = GetChassis();
    if (!chassis)
        return 0.0f;
    const Vec3 velocity = chassis->GetLinearVelocity();
    return std::sqrt(velocity.x * velocity.x + velocity.z * velocity.z) * 3.6f;
}
float Car::GetThrottle() const { return m_throttle; }
float Car::GetBrake() const { return m_brake; }
float Car::GetSteering() const { return m_steering; }
float Car::GetClutch() const { return m_clutch; }
