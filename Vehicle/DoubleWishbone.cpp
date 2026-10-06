#include "DoubleWishbone.h"

#include <cmath>

namespace {
    constexpr float Epsilon = 0.000001f;
}

DoubleWishbone::DoubleWishbone()
    : m_hubOffset(0.0f, 0.0f, 0.0f),
    m_upperOuterJoint(0.0f, 0.0f, 0.0f),
    m_lowerOuterJoint(0.0f, 0.0f, 0.0f),
    m_hubPosition(0.0f, 0.0f, 0.0f),
    m_hubOrientation(Quaternion::Identity()) {}

void DoubleWishbone::Configure(const DoubleWishboneConfig& config) {
    m_upperArm.innerA = config.upperInnerA;
    m_upperArm.innerB = config.upperInnerB;
    m_upperArm.outer = config.upperOuter;
    m_upperArm.lengthA = Distance(config.upperInnerA, config.upperOuter);
    m_upperArm.lengthB = Distance(config.upperInnerB, config.upperOuter);

    m_lowerArm.innerA = config.lowerInnerA;
    m_lowerArm.innerB = config.lowerInnerB;
    m_lowerArm.outer = config.lowerOuter;
    m_lowerArm.lengthA = Distance(config.lowerInnerA, config.lowerOuter);
    m_lowerArm.lengthB = Distance(config.lowerInnerB, config.lowerOuter);

    m_hubOffset = config.hubOffset;
    m_upperOuterJoint = config.upperOuter;
    m_lowerOuterJoint = config.lowerOuter;
    m_hubPosition = config.lowerOuter + config.hubOffset;
    m_hubOrientation = Quaternion::Identity();
}

void DoubleWishbone::Solve(
    const Vec3& chassisPosition,
    const Quaternion& chassisOrientation
) {
    m_upperArm.outer = m_upperOuterJoint;
    m_lowerArm.outer = m_lowerOuterJoint;

    SolveArm(m_upperArm, 8);
    SolveArm(m_lowerArm, 8);

    m_upperOuterJoint = m_upperArm.outer;
    m_lowerOuterJoint = m_lowerArm.outer;

    const Vec3 uprightUp = (
        m_upperOuterJoint - m_lowerOuterJoint
    ).Normalized();

    Vec3 forward(0.0f, 0.0f, 1.0f);
    forward = (
        forward -
        uprightUp * forward.Dot(uprightUp)
    ).Normalized();

    if (forward.LengthSquared() < Epsilon)
        forward = Vec3(0.0f, 0.0f, 1.0f);

    const Vec3 right =
        forward.Cross(uprightUp).Normalized();
    const Vec3 correctedForward =
        uprightUp.Cross(right).Normalized();

    Mat3 basis = Mat3::Identity();
    basis.m[0][0] = right.x;
    basis.m[0][1] = right.y;
    basis.m[0][2] = right.z;
    basis.m[1][0] = uprightUp.x;
    basis.m[1][1] = uprightUp.y;
    basis.m[1][2] = uprightUp.z;
    basis.m[2][0] = correctedForward.x;
    basis.m[2][1] = correctedForward.y;
    basis.m[2][2] = correctedForward.z;

    const Quaternion localOrientation =
        Quaternion::FromMat3(basis).Normalized();

    m_hubOrientation =
        (chassisOrientation * localOrientation).Normalized();

    m_hubPosition =
        chassisPosition +
        chassisOrientation * (
            m_lowerOuterJoint +
            localOrientation * m_hubOffset
        );
}

const Vec3& DoubleWishbone::GetUpperOuterJoint() const {
    return m_upperOuterJoint;
}

const Vec3& DoubleWishbone::GetLowerOuterJoint() const {
    return m_lowerOuterJoint;
}

const Vec3& DoubleWishbone::GetHubPosition() const {
    return m_hubPosition;
}

const Quaternion& DoubleWishbone::GetHubOrientation() const {
    return m_hubOrientation;
}

float DoubleWishbone::Distance(
    const Vec3& a,
    const Vec3& b
) {
    return (a - b).Length();
}

void DoubleWishbone::ProjectDistance(
    Vec3& point,
    const Vec3& anchor,
    float length
) {
    const Vec3 delta = point - anchor;
    const float distance = delta.Length();

    if (distance < Epsilon)
        return;

    point = anchor + delta * (length / distance);
}

void DoubleWishbone::SolveArm(
    Arm& arm,
    int iterations
) {
    for (int i = 0; i < iterations; ++i) {
        ProjectDistance(arm.outer, arm.innerA, arm.lengthA);
        ProjectDistance(arm.outer, arm.innerB, arm.lengthB);
    }
}
