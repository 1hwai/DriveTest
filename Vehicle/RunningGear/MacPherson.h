#pragma once

#include "../../Core/Math/Quaternion.h"
#include "../../Core/Math/Vec3.h"
#include "ISuspensionGeometry.h"

struct MacPhersonArmConfig {
    Vec3 innerPivotA;
    Vec3 innerPivotB;
    Vec3 outerJoint;
};

struct MacPhersonStrutConfig {
    Vec3 upperMount;
    Vec3 lowerMountOffset;
    float length = 0.0f;
};

struct MacPhersonUprightConfig {
    Vec3 hubOffset;
};

struct MacPhersonConfig {
    MacPhersonArmConfig lowerArm;
    MacPhersonStrutConfig strut;
    MacPhersonUprightConfig upright;
};

class MacPherson : public ISuspensionGeometry {
public:
    MacPherson();

    bool Configure(const MacPhersonConfig& config);
    bool Solve(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation
    );

    bool SolveAtTravel(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation,
        float travel
    ) override;

    const Vec3& GetLowerOuterJoint() const;
    const Vec3& GetStrutLowerMount() const;
    const Vec3& GetSpringMountA() const override;
    const Vec3& GetSpringMountB() const override;
    Vec3 GetSteeringAxisStart() const override;
    Vec3 GetSteeringAxisEnd() const override;
    bool ApplySteering(float angle) override;
    const Vec3& GetHubPosition() const override;
    const Quaternion& GetHubOrientation() const override;
    float GetLowerArmLengthA() const;
    float GetLowerArmLengthB() const;
    float GetStrutLength() const;

private:
    struct LowerArmState {
        Vec3 innerPivotA;
        Vec3 innerPivotB;
        Vec3 outerJoint;
        float innerToOuterLengthA;
        float innerToOuterLengthB;
    };

    struct StrutState {
        Vec3 upperMount;
        Vec3 lowerMount;
        float length;
    };

    struct UprightState {
        Vec3 lowerJoint;
        Vec3 strutLowerJoint;
        Vec3 hubPosition;
        Quaternion hubOrientation;
    };

    MacPhersonArmConfig m_lowerArmConfig;
    MacPhersonStrutConfig m_strutConfig;
    MacPhersonUprightConfig m_uprightConfig;

    LowerArmState m_lowerArm;
    StrutState m_strut;
    UprightState m_upright;

    float m_strutLowerOffsetLength;

    static float Distance(const Vec3& a, const Vec3& b);
    static Quaternion FromToRotation(const Vec3& from, const Vec3& to);
    static bool SolveConstraints(
        LowerArmState& lowerArm,
        const Vec3& strutUpperMount,
        float strutTargetDistance
    );

    void UpdateUpright(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation
    );
};
