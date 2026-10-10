#include "RaycastWheelContactProvider.h"

#include "../../Physics/PhysicsWorld.h"

std::unique_ptr<IWheelContactProvider> CreateDefaultWheelContactProvider() {
    return std::make_unique<RaycastWheelContactProvider>();
}

WheelContactResult RaycastWheelContactProvider::Query(
    const WheelContactInput& input,
    const PhysicsWorld& physicsWorld,
    const RigidBody* ignoredBody
) const {
    WheelContactResult contact;

    const Ray ray{
        input.hubPosition,
        Vec3(0.0f, -1.0f, 0.0f)
    };
    RaycastResult rayResult;

    if (!physicsWorld.Raycast(
        ray,
        rayResult,
        input.maxReach,
        ignoredBody
    )) {
        return contact;
    }

    WheelContactSample sample;
    sample.point = rayResult.point;
    sample.normal = rayResult.normal;
    sample.hasQueryDistance = true;
    sample.queryDistance = rayResult.distance;

    contact.state = WheelContactState::Contact;
    contact.samples.push_back(sample);
    return contact;
}
