#pragma once

#include "../Core/Math/Quaternion.h"
#include "../Core/Math/Vec3.h"

struct DoubleWishboneConfig {
    Vec3 upperInnerA;
    Vec3 upperInnerB;
    Vec3 upperOuter;
    Vec3 lowerInnerA;
    Vec3 lowerInnerB;
    Vec3 lowerOuter;
    Vec3 hubOffset;
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
    struct Arm {
        Vec3 innerA;
        Vec3 innerB;
        Vec3 outer;
        float lengthA;
        float lengthB;
    };

    Arm m_upperArm;
    Arm m_lowerArm;
    Vec3 m_hubOffset;
    Vec3 m_upperOuterJoint;
    Vec3 m_lowerOuterJoint;
    Vec3 m_hubPosition;
    Quaternion m_hubOrientation;

    static float Distance(const Vec3& a, const Vec3& b);
    static void ProjectDistance(Vec3& point, const Vec3& anchor, float length);
    static void SolveArm(Arm& arm, int iterations);
};
