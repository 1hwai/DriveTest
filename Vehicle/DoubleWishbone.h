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

struct DoubleWishboneSpringMountConfig {
    Vec3 chassisMount;
    Vec3 uprightMountOffset;
};

struct DoubleWishboneConfig {
    DoubleWishboneArmConfig upperArm;
    DoubleWishboneArmConfig lowerArm;
    DoubleWishboneUprightConfig upright;
    DoubleWishboneSpringMountConfig spring;
};

class DoubleWishbone {
public:
    DoubleWishbone();

    void Configure(const DoubleWishboneConfig& config);
    void Solve(const Vec3& chassisPosition, const Quaternion& chassisOrientation);
    bool SolveAtTravel(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation,
        float travel
    );

    const Vec3& GetUpperOuterJoint() const;
    const Vec3& GetLowerOuterJoint() const;
    const Vec3& GetHubPosition() const;
    const Quaternion& GetHubOrientation() const;
    const Vec3& GetSpringMountA() const;
    const Vec3& GetSpringMountB() const;

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
    Vec3 m_springMountA;
    Vec3 m_springMountB;
    Vec3 m_springMountALocal;
    Vec3 m_springMountBOffset;

    float m_uprightJointDistance;
    float m_restOuterAverageY;

    static float Distance(const Vec3& a, const Vec3& b);
    static void ProjectDistance(Vec3& point, const Vec3& anchor, float length);
    static bool SolveArmAtHeight(
        const ArmState& arm,
        float height,
        const Vec3& previousOuter,
        Vec3& outer
    );
    static bool SolveConstraints(
        ArmState& upperArm,
        ArmState& lowerArm,
        float uprightJointDistance,
        float targetAverageY
    );

    void UpdateUpright(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation
    );
};
