#include "MacPherson.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr float Epsilon = 0.000001f;
}

MacPherson::MacPherson()
    : m_strutLowerOffsetLength(0.0f) {
    m_upright.hubPosition = Vec3(0.0f, 0.0f, 0.0f);
    m_upright.hubOrientation = Quaternion::Identity();
}

void MacPherson::Configure(const MacPhersonConfig& config) {
    m_lowerArmConfig = config.lowerArm;
    m_strutConfig = config.strut;
    m_uprightConfig = config.upright;

    m_lowerArm.innerPivotA = config.lowerArm.innerPivotA;
    m_lowerArm.innerPivotB = config.lowerArm.innerPivotB;
    m_lowerArm.outerJoint = config.lowerArm.outerJoint;
    m_lowerArm.innerToOuterLengthA =
        Distance(config.lowerArm.innerPivotA, config.lowerArm.outerJoint);
    m_lowerArm.innerToOuterLengthB =
        Distance(config.lowerArm.innerPivotB, config.lowerArm.outerJoint);

    m_strut.upperMount = config.strut.upperMount;
    m_strut.lowerMount =
        config.lowerArm.outerJoint +
        config.strut.lowerMountOffset;
    m_strutLowerOffsetLength =
        config.strut.lowerMountOffset.Length();
    m_strut.length =
        config.strut.length > 0.0f
            ? config.strut.length
            : Distance(m_strut.upperMount, m_strut.lowerMount);

    m_upright.lowerJoint = config.lowerArm.outerJoint;
    m_upright.strutLowerJoint = m_strut.lowerMount;
    m_upright.hubPosition =
        config.lowerArm.outerJoint +
        config.upright.hubOffset;
    m_upright.hubOrientation = Quaternion::Identity();
}

bool MacPherson::Solve(
    const Vec3& chassisPosition,
    const Quaternion& chassisOrientation
) {
    return SolveAtTravel(
        chassisPosition,
        chassisOrientation,
        0.0f
    );
}

bool MacPherson::SolveAtTravel(
    const Vec3& chassisPosition,
    const Quaternion& chassisOrientation,
    float travel
) {
    m_lowerArm.innerPivotA =
        chassisPosition +
        chassisOrientation * m_lowerArmConfig.innerPivotA;
    m_lowerArm.innerPivotB =
        chassisPosition +
        chassisOrientation * m_lowerArmConfig.innerPivotB;
    m_lowerArm.outerJoint =
        chassisPosition +
        chassisOrientation * m_lowerArmConfig.outerJoint;

    m_strut.upperMount =
        chassisPosition +
        chassisOrientation * m_strutConfig.upperMount;

    const float strutLength =
        m_strut.length - travel;
    if (strutLength <= Epsilon)
        return false;

    if (!SolveConstraints(
            m_lowerArm,
            m_strut.upperMount,
            strutLength + m_strutLowerOffsetLength
        )) {
        return false;
    }

    m_upright.lowerJoint = m_lowerArm.outerJoint;

    UpdateUpright(chassisPosition, chassisOrientation);
    return true;
}

void MacPherson::UpdateUpright(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation
) {
    const Vec3 localOffset =
        m_strutConfig.lowerMountOffset;

    Vec3 localAxis(0.0f, 1.0f, 0.0f);
    if (localOffset.LengthSquared() > Epsilon)
        localAxis = localOffset.Normalized();

    const Vec3 strutAxis =
        (m_strut.upperMount - m_upright.lowerJoint).Normalized();

    const Quaternion localOrientation =
        FromToRotation(localAxis, strutAxis);

    m_upright.strutLowerJoint =
        m_upright.lowerJoint +
        localOrientation * localOffset;

    m_strut.lowerMount = m_upright.strutLowerJoint;

    m_upright.hubOrientation =
        localOrientation;

    m_upright.hubPosition =
        m_upright.lowerJoint +
        localOrientation * m_uprightConfig.hubOffset;
}

float MacPherson::Distance(
    const Vec3& a,
    const Vec3& b
) {
    return (a - b).Length();
}

Quaternion MacPherson::FromToRotation(
    const Vec3& from,
    const Vec3& to
) {
    const Vec3 a = from.Normalized();
    const Vec3 b = to.Normalized();
    const float dot = std::clamp(a.Dot(b), -1.0f, 1.0f);

    if (dot > 1.0f - Epsilon)
        return Quaternion::Identity();

    if (dot < -1.0f + Epsilon) {
        Vec3 axis =
            Vec3(1.0f, 0.0f, 0.0f).Cross(a);

        if (axis.LengthSquared() < Epsilon)
            axis =
                Vec3(0.0f, 1.0f, 0.0f).Cross(a);

        return Quaternion::FromAxisAngle(
            axis.Normalized(),
            3.14159265359f
        );
    }

    const Vec3 cross = a.Cross(b);
    return Quaternion(
        1.0f + dot,
        cross.x,
        cross.y,
        cross.z
    ).Normalized();
}

bool MacPherson::SolveConstraints(
    LowerArmState& lowerArm,
    const Vec3& strutUpperMount,
    float strutTargetDistance
) {
    if (strutTargetDistance <= Epsilon)
        return false;

    const float radiusA =
        lowerArm.innerToOuterLengthA;
    const float radiusB =
        lowerArm.innerToOuterLengthB;

    const Vec3 centerA =
        lowerArm.innerPivotA;
    const Vec3 centerB =
        lowerArm.innerPivotB;

    const Vec3 armAxis =
        centerB - centerA;
    const float armAxisLength =
        armAxis.Length();

    if (armAxisLength < Epsilon ||
        armAxisLength > radiusA + radiusB + Epsilon ||
        armAxisLength < std::abs(radiusA - radiusB) - Epsilon) {
        return false;
    }

    const Vec3 ex =
        armAxis / armAxisLength;

    const float along =
        (
            radiusA * radiusA -
            radiusB * radiusB +
            armAxisLength * armAxisLength
        ) /
        (2.0f * armAxisLength);

    const float circleRadiusSquared =
        radiusA * radiusA -
        along * along;

    if (circleRadiusSquared < -Epsilon)
        return false;

    const float circleRadius =
        std::sqrt(std::max(0.0f, circleRadiusSquared));

    const Vec3 circleCenter =
        centerA + ex * along;

    const Vec3 toStrut =
        strutUpperMount - circleCenter;

    const Vec3 projected =
        toStrut -
        ex * toStrut.Dot(ex);

    const float projectedLength =
        projected.Length();

    if (projectedLength < Epsilon ||
        circleRadius < Epsilon)
        return false;

    const float cosine =
        (
            circleRadius * circleRadius +
            projectedLength * projectedLength -
            strutTargetDistance * strutTargetDistance
        ) /
        (2.0f * circleRadius * projectedLength);

    if (cosine < -1.0f - Epsilon ||
        cosine > 1.0f + Epsilon)
        return false;

    const float clampedCosine =
        std::clamp(cosine, -1.0f, 1.0f);

    const Vec3 projectedDirection =
        projected / projectedLength;

    const Vec3 perpendicular =
        ex.Cross(projectedDirection).Normalized();

    const float sine =
        std::sqrt(std::max(
            0.0f,
            1.0f - clampedCosine * clampedCosine
        ));

    const Vec3 candidateA =
        circleCenter +
        (
            projectedDirection * clampedCosine +
            perpendicular * sine
        ) * circleRadius;

    const Vec3 candidateB =
        circleCenter +
        (
            projectedDirection * clampedCosine -
            perpendicular * sine
        ) * circleRadius;

    const float distanceA =
        (candidateA - lowerArm.outerJoint).LengthSquared();

    const float distanceB =
        (candidateB - lowerArm.outerJoint).LengthSquared();

    lowerArm.outerJoint =
        distanceA <= distanceB
            ? candidateA
            : candidateB;

    return true;
}

const Vec3& MacPherson::GetLowerOuterJoint() const {
    return m_upright.lowerJoint;
}

const Vec3& MacPherson::GetStrutLowerMount() const {
    return m_upright.strutLowerJoint;
}

const Vec3& MacPherson::GetHubPosition() const {
    return m_upright.hubPosition;
}

const Quaternion& MacPherson::GetHubOrientation() const {
    return m_upright.hubOrientation;
}

float MacPherson::GetLowerArmLengthA() const {
    return m_lowerArm.innerToOuterLengthA;
}

float MacPherson::GetLowerArmLengthB() const {
    return m_lowerArm.innerToOuterLengthB;
}

float MacPherson::GetStrutLength() const {
    return m_strut.length;
}

const Vec3& MacPherson::GetSpringMountA() const { return m_strut.upperMount; }
const Vec3& MacPherson::GetSpringMountB() const { return m_strut.lowerMount; }
