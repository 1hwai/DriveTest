#pragma once

#include "../Core/Math/Quaternion.h"
#include "../Core/Math/Vec3.h"

// Consumer-facing kinematics contract shared by concrete suspension layouts.
// Configuration remains specific to each implementation.
class ISuspensionGeometry {
public:
    virtual ~ISuspensionGeometry() = default;

    virtual bool SolveAtTravel(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation,
        float travel
    ) = 0;

    virtual const Vec3& GetHubPosition() const = 0;
    virtual const Quaternion& GetHubOrientation() const = 0;
    virtual const Vec3& GetSpringMountA() const = 0;
    virtual const Vec3& GetSpringMountB() const = 0;
    virtual Vec3 GetSteeringAxisStart() const = 0;
    virtual Vec3 GetSteeringAxisEnd() const = 0;
    virtual bool ApplySteering(float angle) = 0;
};
