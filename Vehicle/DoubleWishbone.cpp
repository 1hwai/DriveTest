#include "DoubleWishbone.h"

#include <cmath>

namespace {
    constexpr float Epsilon = 0.000001f;
}

DoubleWishbone::DoubleWishbone()
    : m_uprightJointDistance(0.0f) {
    m_upright.hubPosition = Vec3(0.0f, 0.0f, 0.0f);
    m_upright.hubOrientation = Quaternion::Identity();
}

void DoubleWishbone::Configure(const DoubleWishboneConfig& config) {
    m_upperArmConfig = config.upperArm;
    m_lowerArmConfig = config.lowerArm;
    m_uprightConfig = config.upright;

    m_upperArm.innerPivotA = config.upperArm.innerPivotA;
    m_upperArm.innerPivotB = config.upperArm.innerPivotB;
    m_upperArm.outerJoint = config.upperArm.outerJoint;
    m_upperArm.innerToOuterLengthA =
        Distance(config.upperArm.innerPivotA, config.upperArm.outerJoint);
    m_upperArm.innerToOuterLengthB =
        Distance(config.upperArm.innerPivotB, config.upperArm.outerJoint);

    m_lowerArm.innerPivotA = config.lowerArm.innerPivotA;
    m_lowerArm.innerPivotB = config.lowerArm.innerPivotB;
    m_lowerArm.outerJoint = config.lowerArm.outerJoint;
    m_lowerArm.innerToOuterLengthA =
        Distance(config.lowerArm.innerPivotA, config.lowerArm.outerJoint);
    m_lowerArm.innerToOuterLengthB =
        Distance(config.lowerArm.innerPivotB, config.lowerArm.outerJoint);

    m_uprightJointDistance =
        Distance(config.upperArm.outerJoint, config.lowerArm.outerJoint);

    m_upright.upperJoint = m_upperArm.outerJoint;
    m_upright.lowerJoint = m_lowerArm.outerJoint;
    m_upright.hubPosition =
        m_lowerArm.outerJoint + config.upright.hubOffset;
    m_upright.hubOrientation = Quaternion::Identity();
}

void DoubleWishbone::Solve(
    const Vec3& chassisPosition,
    const Quaternion& chassisOrientation
) {
    SolveConstraints(
        m_upperArm,
        m_lowerArm,
        m_uprightJointDistance,
        32
    );

    m_upright.upperJoint = m_upperArm.outerJoint;
    m_upright.lowerJoint = m_lowerArm.outerJoint;

    UpdateUpright(chassisPosition, chassisOrientation);
}

void DoubleWishbone::UpdateUpright(
    const Vec3& chassisPosition,
    const Quaternion& chassisOrientation
) {
    const Vec3 uprightUp =
        (m_upright.upperJoint - m_upright.lowerJoint).Normalized();

    Vec3 forward(0.0f, 0.0f, 1.0f);
    forward =
        forward -
        uprightUp * forward.Dot(uprightUp);

    if (forward.LengthSquared() < Epsilon) {
        forward = Vec3(1.0f, 0.0f, 0.0f);
    } else {
        forward = forward.Normalized();
    }

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

    m_upright.hubOrientation =
        (chassisOrientation * localOrientation).Normalized();

    m_upright.hubPosition =
        chassisPosition +
        chassisOrientation * (
            m_upright.lowerJoint +
            localOrientation * m_uprightConfig.hubOffset
        );
}

const Vec3& DoubleWishbone::GetUpperOuterJoint() const {
    return m_upright.upperJoint;
}

const Vec3& DoubleWishbone::GetLowerOuterJoint() const {
    return m_upright.lowerJoint;
}

const Vec3& DoubleWishbone::GetHubPosition() const {
    return m_upright.hubPosition;
}

const Quaternion& DoubleWishbone::GetHubOrientation() const {
    return m_upright.hubOrientation;
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

void DoubleWishbone::SolveConstraints(
    ArmState& upperArm,
    ArmState& lowerArm,
    float uprightJointDistance,
    int iterations
) {
    for (int i = 0; i < iterations; ++i) {
        ProjectDistance(
            upperArm.outerJoint,
            upperArm.innerPivotA,
            upperArm.innerToOuterLengthA
        );
        ProjectDistance(
            upperArm.outerJoint,
            upperArm.innerPivotB,
            upperArm.innerToOuterLengthB
        );

        ProjectDistance(
            lowerArm.outerJoint,
            lowerArm.innerPivotA,
            lowerArm.innerToOuterLengthA
        );
        ProjectDistance(
            lowerArm.outerJoint,
            lowerArm.innerPivotB,
            lowerArm.innerToOuterLengthB
        );

        const Vec3 delta =
            lowerArm.outerJoint - upperArm.outerJoint;
        const float distance = delta.Length();

        if (distance < Epsilon)
            continue;

        const Vec3 correction =
            delta * ((distance - uprightJointDistance) / distance);

        upperArm.outerJoint += correction * 0.5f;
        lowerArm.outerJoint -= correction * 0.5f;
    }
}
