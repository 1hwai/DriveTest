#include "DoubleWishbone.h"

#include <cmath>

namespace {
    constexpr float Epsilon = 0.000001f;
}

DoubleWishbone::DoubleWishbone()
    : m_uprightJointDistance(0.0f),
    m_restOuterAverageY(0.0f) {
    m_upright.hubPosition = Vec3(0.0f, 0.0f, 0.0f);
    m_upright.hubOrientation = Quaternion::Identity();
    m_springMountA = Vec3(0.0f, 0.0f, 0.0f);
    m_springMountB = Vec3(0.0f, 0.0f, 0.0f);
    m_springMountBOffset = Vec3(0.0f, 0.0f, 0.0f);
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
    m_restOuterAverageY =
        0.5f * (config.upperArm.outerJoint.y + config.lowerArm.outerJoint.y);

    m_upright.upperJoint = m_upperArm.outerJoint;
    m_upright.lowerJoint = m_lowerArm.outerJoint;
    m_upright.hubPosition =
        m_lowerArm.outerJoint + config.upright.hubOffset;
    m_upright.hubOrientation = Quaternion::Identity();
    m_springMountALocal = config.spring.chassisMount;
    m_springMountBOffset = config.spring.uprightMountOffset;
}

void DoubleWishbone::Solve(
    const Vec3& chassisPosition,
    const Quaternion& chassisOrientation
) {
    SolveAtTravel(
        chassisPosition,
        chassisOrientation,
        0.0f
    );
}

bool DoubleWishbone::SolveAtTravel(
    const Vec3& chassisPosition,
    const Quaternion& chassisOrientation,
    float travel
) {
    const float targetAverageY =
        m_restOuterAverageY + travel;

    if (!SolveConstraints(
            m_upperArm,
            m_lowerArm,
            m_uprightJointDistance,
            targetAverageY
        )) {
        return false;
    }

    m_upright.upperJoint = m_upperArm.outerJoint;
    m_upright.lowerJoint = m_lowerArm.outerJoint;

    UpdateUpright(chassisPosition, chassisOrientation);

    m_springMountA =
        chassisPosition +
        chassisOrientation * m_springMountALocal;
    m_springMountB =
        chassisPosition +
        chassisOrientation * m_upright.lowerJoint +
        m_upright.hubOrientation * m_springMountBOffset;


    return true;
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

const Vec3& DoubleWishbone::GetSpringMountA() const {
    return m_springMountA;
}

const Vec3& DoubleWishbone::GetSpringMountB() const {
    return m_springMountB;
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

bool DoubleWishbone::SolveArmAtHeight(
    const ArmState& arm,
    float height,
    const Vec3& previousOuter,
    Vec3& outer
) {
    const float radiusASquared =
        arm.innerToOuterLengthA * arm.innerToOuterLengthA -
        (height - arm.innerPivotA.y) *
        (height - arm.innerPivotA.y);
    const float radiusBSquared =
        arm.innerToOuterLengthB * arm.innerToOuterLengthB -
        (height - arm.innerPivotB.y) *
        (height - arm.innerPivotB.y);

    if (radiusASquared < -Epsilon ||
        radiusBSquared < -Epsilon)
        return false;

    const float radiusA =
        std::sqrt(std::max(0.0f, radiusASquared));
    const float radiusB =
        std::sqrt(std::max(0.0f, radiusBSquared));

    const Vec3 a(
        arm.innerPivotA.x,
        0.0f,
        arm.innerPivotA.z
    );
    const Vec3 b(
        arm.innerPivotB.x,
        0.0f,
        arm.innerPivotB.z
    );
    const Vec3 delta = b - a;
    const float distance = std::sqrt(
        delta.x * delta.x +
        delta.z * delta.z
    );

    if (distance < Epsilon ||
        distance > radiusA + radiusB + Epsilon ||
        distance < std::abs(radiusA - radiusB) - Epsilon)
        return false;

    const float along =
        (radiusA * radiusA -
         radiusB * radiusB +
         distance * distance) /
        (2.0f * distance);
    const float heightSquared =
        radiusA * radiusA -
        along * along;

    if (heightSquared < -Epsilon)
        return false;

    const float intersectionHeight =
        std::sqrt(std::max(0.0f, heightSquared));

    const Vec3 center =
        a + delta * (along / distance);
    const Vec3 perpendicular(
        -delta.z / distance,
        0.0f,
        delta.x / distance
    );

    const Vec3 candidateA =
        center + perpendicular * intersectionHeight;
    const Vec3 candidateB =
        center - perpendicular * intersectionHeight;

    const float distanceA =
        (candidateA - Vec3(
            previousOuter.x,
            0.0f,
            previousOuter.z
        )).LengthSquared();
    const float distanceB =
        (candidateB - Vec3(
            previousOuter.x,
            0.0f,
            previousOuter.z
        )).LengthSquared();

    const Vec3 selected =
        distanceA <= distanceB
            ? candidateA
            : candidateB;

    outer = Vec3(
        selected.x,
        height,
        selected.z
    );
    return true;
}

bool DoubleWishbone::SolveConstraints(
    ArmState& upperArm,
    ArmState& lowerArm,
    float uprightJointDistance,
    float targetAverageY
) {
    const float upperMaxDeltaA =
        upperArm.innerToOuterLengthA;
    const float upperMaxDeltaB =
        upperArm.innerToOuterLengthB;
    const float lowerMaxDeltaA =
        lowerArm.innerToOuterLengthA;
    const float lowerMaxDeltaB =
        lowerArm.innerToOuterLengthB;

    float minUpperY =
        std::max(
            upperArm.innerPivotA.y - upperMaxDeltaA,
            upperArm.innerPivotB.y - upperMaxDeltaB
        );
    float maxUpperY =
        std::min(
            upperArm.innerPivotA.y + upperMaxDeltaA,
            upperArm.innerPivotB.y + upperMaxDeltaB
        );

    float minLowerY =
        std::max(
            lowerArm.innerPivotA.y - lowerMaxDeltaA,
            lowerArm.innerPivotB.y - lowerMaxDeltaB
        );
    float maxLowerY =
        std::min(
            lowerArm.innerPivotA.y + lowerMaxDeltaA,
            lowerArm.innerPivotB.y + lowerMaxDeltaB
        );

    float minUpperFromLower =
        2.0f * targetAverageY - maxLowerY;
    float maxUpperFromLower =
        2.0f * targetAverageY - minLowerY;

    const float lowerBound =
        std::max(minUpperY, minUpperFromLower);
    const float upperBound =
        std::min(maxUpperY, maxUpperFromLower);

    if (lowerBound > upperBound)
        return false;

    auto Evaluate = [&](float upperY, Vec3& upper, Vec3& lower) {
        if (!SolveArmAtHeight(
                upperArm,
                upperY,
                upperArm.outerJoint,
                upper
            )) {
            return false;
        }

        const float lowerY =
            2.0f * targetAverageY - upperY;

        return SolveArmAtHeight(
            lowerArm,
            lowerY,
            lowerArm.outerJoint,
            lower
        );
    };

    float bestUpperY = 0.0f;
    float bestError = INFINITY;
    bool found = false;
    float previousY = 0.0f;
    float previousError = 0.0f;
    bool previousValid = false;

    constexpr int Samples = 96;
    for (int i = 0; i <= Samples; ++i) {
        const float upperY =
            lowerBound +
            (upperBound - lowerBound) *
            (static_cast<float>(i) / Samples);

        Vec3 upper;
        Vec3 lower;
        if (!Evaluate(upperY, upper, lower)) {
            previousValid = false;
            continue;
        }

        const float error =
            (upper - lower).Length() -
            uprightJointDistance;

        if (std::abs(error) < bestError) {
            bestError = std::abs(error);
            bestUpperY = upperY;
        }

        if (previousValid &&
            previousError * error <= 0.0f) {
            float a = previousY;
            float b = upperY;
            float fa = previousError;

            for (int iteration = 0; iteration < 32; ++iteration) {
                const float mid = 0.5f * (a + b);
                Vec3 midUpper;
                Vec3 midLower;

                if (!Evaluate(mid, midUpper, midLower))
                    break;

                const float fm =
                    (midUpper - midLower).Length() -
                    uprightJointDistance;

                if (std::abs(fm) < Epsilon) {
                    a = mid;
                    b = mid;
                    break;
                }

                if (fa * fm <= 0.0f) {
                    b = mid;
                } else {
                    a = mid;
                    fa = fm;
                }
            }

            bestUpperY = 0.5f * (a + b);
            found = true;
            break;
        }

        previousY = upperY;
        previousError = error;
        previousValid = true;
    }

    if (!found) {
        if (bestError > 0.0001f)
            return false;
    }

    Vec3 solvedUpper;
    Vec3 solvedLower;
    if (!Evaluate(bestUpperY, solvedUpper, solvedLower))
        return false;

    if (std::abs(
            (solvedUpper - solvedLower).Length() -
            uprightJointDistance
        ) > 0.0001f)
        return false;

    upperArm.outerJoint = solvedUpper;
    lowerArm.outerJoint = solvedLower;
    return true;
}
