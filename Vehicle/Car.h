#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include "RunningGear/Wheel.h"
#include "RunningGear/IWheelContactProvider.h"
#include "RunningGear/DoubleWishbone.h"
#include "RunningGear/Suspension.h"
#include "RunningGear/Tire.h"
#include "Powertrain/Engine.h"
#include "Powertrain/Transmission.h"
#include "Powertrain/Powertrain.h"
#include "VehicleConfig.h"

class RigidBody;
class PhysicsWorld;

enum class WheelIndex {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
    Count
};

constexpr size_t WheelCount =
    static_cast<size_t>(WheelIndex::Count);

class Car {
public:
    Car();
    explicit Car(std::unique_ptr<IWheelContactProvider> contactProvider);

    void SetChassis(RigidBody* chassis);
    void ApplyConfig(const VehicleConfig& config);

    void SetInput(
        float throttle,
        float brake,
        float steerInput,
        float clutch
    );

    void UpdatePhysics(
        PhysicsWorld& physicsWorld,
        float deltaTime
    );

    DoubleWishbone& GetSuspensionGeometry(WheelIndex index);
    const DoubleWishbone& GetSuspensionGeometry(WheelIndex index) const;

    Wheel& GetWheel(WheelIndex index);
    const Wheel& GetWheel(WheelIndex index) const;

    Suspension& GetSuspension(WheelIndex index);
    const Suspension& GetSuspension(WheelIndex index) const;

    Tire& GetTire(WheelIndex index);
    const Tire& GetTire(WheelIndex index) const;

    RigidBody* GetChassis();
    const RigidBody* GetChassis() const;

    Engine& GetEngine();
    const Engine& GetEngine() const;

    Transmission& GetTransmission();
    const Transmission& GetTransmission() const;

    float GetSpeedKmh() const;
    float GetThrottle() const;
    float GetBrake() const;
    float GetSteering() const;
    float GetClutch() const;

private:
    std::unique_ptr<IWheelContactProvider> m_contactProvider;
    RigidBody* m_chassis;
    float m_throttle;
    float m_brake;
    float m_steering;
    float m_clutch;
    float m_energyTimer;
    float m_brakeTimer;
    float m_tireTimer;
    float m_brakeTorque;
    float m_maxSteeringAngle;

    std::array<DoubleWishbone, WheelCount> m_suspensionGeometry;
    std::array<float, WheelCount> m_minimumSuspensionTravel{};
    std::array<float, WheelCount> m_maximumSuspensionTravel{};
    std::array<Wheel, WheelCount> m_wheels;
    std::array<Suspension, WheelCount> m_suspensions;
    std::array<Tire, WheelCount> m_tires;

    Engine m_engine;
    Transmission m_transmission;
    Powertrain m_powertrain;
};
