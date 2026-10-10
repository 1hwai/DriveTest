#pragma once

#include <memory>

#include "WheelContact.h"

class PhysicsWorld;
class RigidBody;

class IWheelContactProvider {
public:
    virtual ~IWheelContactProvider() = default;

    virtual WheelContactResult Query(
        const WheelContactInput& input,
        const PhysicsWorld& physicsWorld,
        const RigidBody* ignoredBody
    ) const = 0;
};

// The default is a composition choice, not a dependency of Wheel,
// Suspension, or Tire. Callers can inject any other provider into Car.
std::unique_ptr<IWheelContactProvider> CreateDefaultWheelContactProvider();
