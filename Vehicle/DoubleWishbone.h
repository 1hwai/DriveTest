#pragma once

#include "../Core/Math/Quaternion.h"
#include "../Core/Math/Vec3.h"

struct DoubleWishboneArmConfig {
    Vec3 innerPivotA;
    Vec3 innerPivotB;
    Vec3 outerJoint;
};

struct DoubleWishboneUprightConfig {
    Vec3 hubOffset;
};

struct DoubleWishboneConfig {
    DoubleWishboneArmConfig upperArm;
    DoubleWishboneArmConfig lowerArm;
    DoubleWishboneUprightConfig upright;
};

class DoubleWishbone {
public:
    DoubleWishbone();

    void Configure(const DoubleWishboneConfig& config);
    void Solve(const Vec3& chassisPosition, const Quaternion& chassisOrientation);

    const Vec3& GetUpperOuterJoint() const;
    const Vec3& GetLowerOuterJoint() const;
    const Vec3& GetHubPosition() const;
    const Quaternion& GetHubOrientation() const;

private:
    struct ArmState {
        Vec3 innerPivotA;
        Vec3 innerPivotB;
        Vec3 outerJoint;
        float innerToOuterLengthA;
        float innerToOuterLengthB;
    };

    struct UprightState {
        Vec3 upperJoint;
        Vec3 lowerJoint;
        Vec3 hubPosition;
        Quaternion hubOrientation;
    };

    DoubleWishboneArmConfig m_upperArmConfig;
    DoubleWishboneArmConfig m_lowerArmConfig;
    DoubleWishboneUprightConfig m_uprightConfig;

    ArmState m_upperArm;
    ArmState m_lowerArm;
    UprightState m_upright;

    float m_uprightJointDistance;

    static float Distance(const Vec3& a, const Vec3& b);
    static void ProjectDistance(Vec3& point, const Vec3& anchor, float length);
    static void SolveConstraints(
        ArmState& upperArm,
        ArmState& lowerArm,
        float uprightJointDistance,
        int iterations
    );

    void UpdateUpright(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation
    );
};
