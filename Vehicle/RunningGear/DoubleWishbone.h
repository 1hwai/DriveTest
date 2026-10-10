#pragma once

#include "../../Core/Math/Quaternion.h"
#include "../../Core/Math/Vec3.h"
#include "ISuspensionGeometry.h"

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

class DoubleWishbone : public ISuspensionGeometry {
public:
    DoubleWishbone();
    std::unique_ptr<ISuspensionGeometry> Clone() const override;

    void Configure(const DoubleWishboneConfig& config);
    void Solve(const Vec3& chassisPosition, const Quaternion& chassisOrientation);
    bool SolveAtTravel(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation,
        float travel
    ) override;

    const Vec3& GetUpperOuterJoint() const;
    const Vec3& GetLowerOuterJoint() const;
    const Vec3& GetHubPosition() const override;
    const Quaternion& GetHubOrientation() const override;
    const Vec3& GetSpringMountA() const override;
    const Vec3& GetSpringMountB() const override;
    Vec3 GetSteeringAxisStart() const override;
    Vec3 GetSteeringAxisEnd() const override;
    bool ApplySteering(float angle) override;

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
    Vec3 m_steeringAxisStart;
    Vec3 m_steeringAxisEnd;

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
