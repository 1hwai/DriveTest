#pragma once

#include <vector>

#include "../Core/Math/Quaternion.h"
#include "../Core/Math/Vec3.h"

enum class WheelContactState {
    NoContact,
    Contact
};

struct WheelContactInput {
    Vec3 hubPosition;
    Quaternion hubOrientation;
    float wheelRadius = 0.0f;
    float maxReach = 0.0f;
};

struct WheelContactSample {
    Vec3 point;
    Vec3 normal;

    bool hasQueryDistance = false;
    float queryDistance = 0.0f;

    bool hasSurfaceId = false;
    int surfaceId = -1;

    bool hasSeparation = false;
    float separation = 0.0f;

    bool hasPenetration = false;
    float penetration = 0.0f;
};

struct WheelContactResult {
    WheelContactState state = WheelContactState::NoContact;
    std::vector<WheelContactSample> samples;

    bool HasContact() const {
        return state == WheelContactState::Contact && !samples.empty();
    }
};
