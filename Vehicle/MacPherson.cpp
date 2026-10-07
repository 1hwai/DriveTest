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
    m_strut.length =
        config.strut.length > 0.0f
            ? config.strut.length
            : Distance(m_strut.upperMount, m_strut.lowerMount);

    m_strutLowerOffsetLength =
        config.strut.lowerMountOffset.Length();

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
    const float targetDistance =
        m_strut.length +
        m_strutLowerOffsetLength;

    if (!SolveConstraints(
            m_lowerArm,
            m_strut.upperMount,
            targetDistance
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
    const Vec3 strutAxis =
        (m_strut.upperMount - m_upright.lowerJoint).Normalized();

    const Vec3 localStrutAxis =
        m_strutConfig.lowerMountOffset.Normalized();

    Quaternion localOrientation =
        FromToRotation(localStrutAxis, strutAxis);

    Vec3 forward(0.0f, 0.0f, 1.0f);
    forward =
        forward -
        strutAxis * forward.Dot(strutAxis);

    if (forward.LengthSquared() < Epsilon) {
        forward = Vec3(1.0f, 0.0f, 0.0f);
        forward =
            forward -
            strutAxis * forward.Dot(strutAxis);
    }

    forward = forward.Normalized();

    const Vec3 right =
        forward.Cross(strutAxis).Normalized();
    const Vec3 correctedForward =
        strutAxis.Cross(right).Normalized();

    Mat3 basis = Mat3::Identity();
    basis.m[0][0] = right.x;
    basis.m[0][1] = right.y;
    basis.m[0][2] = right.z;
    basis.m[1][0] = strutAxis.x;
    basis.m[1][1] = strutAxis.y;
    basis.m[1][2] = strutAxis.z;
    basis.m[2][0] = correctedForward.x;
    basis.m[2][1] = correctedForward.y;
    basis.m[2][2] = correctedForward.z;

    const Quaternion basisOrientation =
        Quaternion::FromMat3(basis).Normalized();

    localOrientation =
        (basisOrientation * FromToRotation(
            basisOrientation * localStrutAxis,
            strutAxis
        )).Normalized();

    m_upright.strutLowerJoint =
        m_upright.lowerJoint +
        localOrientation * m_strutConfig.lowerMountOffset;

    m_strut.lowerMount = m_upright.strutLowerJoint;

    m_upright.hubOrientation =
        (chassisOrientation * localOrientation).Normalized();

    m_upright.hubPosition =
        chassisPosition +
        chassisOrientation * (
            m_upright.lowerJoint +
            localOrientation * m_uprightConfig.hubOffset
        );
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
        Vec3 axis = Vec3(1.0f, 0.0f, 0.0f).Cross(a);

        if (axis.LengthSquared() < Epsilon)
            axis = Vec3(0.0f, 1.0f, 0.0f).Cross(a);

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
    const float radiusA = lowerArm.innerToOuterLengthA;
    const float radiusB = lowerArm.innerToOuterLengthB;
    const Vec3 centerA = lowerArm.innerPivotA;
    const Vec3 centerB = lowerArm.innerPivotB;

    const Vec3 axis = centerB - centerA;
    const float distance = axis.Length();

    if (distance < Epsilon ||
        distance > radiusA + radiusB + Epsilon ||
        distance < std::abs(radiusA - radiusB) - Epsilon) {
        return false;
    }

    const Vec3 ex = axis / distance;
    const float along =
        (radiusA * radiusA -
         radiusB * radiusB +
         distance * distance) /
        (2.0f * distance);
    const float circleRadiusSquared =
        radiusA * radiusA -
        along * along;

    if (circleRadiusSquared < -Epsilon)
        return false;

    const float circleRadius =
        std::sqrt(std::max(0.0f, circleRadiusSquared));
    const Vec3 circleCenter =
        centerA + ex * along;

    Vec3 planeAxis =
        ex.Cross(Vec3(0.0f, 1.0f, 0.0f));

    if (planeAxis.LengthSquared() < Epsilon)
        planeAxis =
            ex.Cross(Vec3(0.0f, 0.0f, 1.0f));

    planeAxis = planeAxis.Normalized();
    const Vec3 planeAxisB =
        ex.Cross(planeAxis).Normalized();

    const Vec3 toStrut =
        strutUpperMount - circleCenter;
    const Vec3 projected =
        toStrut -
        ex * toStrut.Dot(ex);
    const float projectedLength =
        projected.Length();

    if (projectedLength < Epsilon)
        return false;

    const float cosine =
        (
            circleRadius * circleRadius +
            projectedLength * projectedLength -
            strutTargetDistance * strutTargetDistance
        ) /
        (2.0f * circleRadius * projectedLength);

    if (cosine < -1.0f - Epsilon ||
        cosine > 1.0f + Epsilon) {
        return false;
    }

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
