#pragma once

#include <memory>

#include "RunningGear/WheelIndex.h"

class RigidBody;
class PhysicsWorld;
class IWheelContactProvider;
class RunningGear;
class Powertrain;
class Wheel;
class Suspension;
class Tire;
class Engine;
class Transmission;
class DoubleWishbone;
struct VehicleConfig;

class Car {
public:
    Car();
    explicit Car(std::unique_ptr<IWheelContactProvider> contactProvider);
    ~Car();

    void SetChassis(RigidBody* chassis);
    void ApplyConfig(const VehicleConfig& config);
    void SetInput(float throttle, float brake, float steerInput, float clutch);
    void UpdatePhysics(PhysicsWorld& physicsWorld, float deltaTime);

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
    std::unique_ptr<RunningGear> m_runningGear;
    std::unique_ptr<Powertrain> m_powertrain;
    float m_throttle = 0.0f;
    float m_brake = 0.0f;
    float m_steering = 0.0f;
    float m_clutch = 0.0f;
};
