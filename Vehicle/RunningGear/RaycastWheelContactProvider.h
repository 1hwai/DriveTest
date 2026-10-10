#pragma once

#include "IWheelContactProvider.h"

class RaycastWheelContactProvider final : public IWheelContactProvider {
public:
    WheelContactResult Query(
        const WheelContactInput& input,
        const PhysicsWorld& physicsWorld,
        const RigidBody* ignoredBody
    ) const override;
};
