#include "MacPherson.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr float Epsilon = 0.000001f;
    constexpr float UnitQuaternionTolerance = 0.001f;

    bool IsFinite(const Vec3& value) {
        return std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    bool IsValidOrientation(const Quaternion& value) {
        if (!std::isfinite(value.w) ||
            !std::isfinite(value.x) ||
            !std::isfinite(value.y) ||
            !std::isfinite(value.z)) {
            return false;
        }

        const float lengthSquared =
            value.w * value.w +
            value.x * value.x +
            value.y * value.y +
            value.z * value.z;

        return std::isfinite(lengthSquared) &&
            std::abs(lengthSquared - 1.0f) <= UnitQuaternionTolerance;
    }
}

std::unique_ptr<ISuspensionGeometry> MacPherson::Clone() const {
    return std::make_unique<MacPherson>(*this);
}

MacPherson::MacPherson()
    : m_strutLowerOffsetLength(0.0f) {
    m_upright.hubPosition = Vec3(0.0f, 0.0f, 0.0f);
    m_upright.hubOrientation = Quaternion::Identity();
}

bool MacPherson::Configure(const MacPhersonConfig& config) {
    // Validate the complete configuration before mutating the current setup.
    if (!IsFinite(config.lowerArm.innerPivotA) ||
        !IsFinite(config.lowerArm.innerPivotB) ||
        !IsFinite(config.lowerArm.outerJoint) ||
        !IsFinite(config.strut.upperMount) ||
        !IsFinite(config.strut.lowerMountOffset) ||
        !IsFinite(config.upright.hubOffset) ||
        !std::isfinite(config.strut.length) ||
        config.strut.length < 0.0f) {
        return false;
    }

    const float armLengthA =
        Distance(config.lowerArm.innerPivotA, config.lowerArm.outerJoint);
    const float armLengthB =
        Distance(config.lowerArm.innerPivotB, config.lowerArm.outerJoint);
    const float pivotSeparation =
        Distance(config.lowerArm.innerPivotA, config.lowerArm.innerPivotB);
    const float lowerMountOffsetLength =
        config.strut.lowerMountOffset.Length();
    const Vec3 configuredLowerMount =
        config.lowerArm.outerJoint + config.strut.lowerMountOffset;
    const float derivedStrutLength =
        Distance(config.strut.upperMount, configuredLowerMount);
    const float effectiveStrutLength =
        config.strut.length > 0.0f
            ? config.strut.length
            : derivedStrutLength;

    if (!std::isfinite(armLengthA) ||
        !std::isfinite(armLengthB) ||
        !std::isfinite(pivotSeparation) ||
        !std::isfinite(lowerMountOffsetLength) ||
        !std::isfinite(derivedStrutLength) ||
        !std::isfinite(effectiveStrutLength) ||
        armLengthA <= Epsilon ||
        armLengthB <= Epsilon ||
        pivotSeparation <= Epsilon ||
        effectiveStrutLength <= Epsilon) {
        return false;
    }

    m_lowerArmConfig = config.lowerArm;
    m_strutConfig = config.strut;
    m_uprightConfig = config.upright;

    m_lowerArm.innerPivotA = config.lowerArm.innerPivotA;
    m_lowerArm.innerPivotB = config.lowerArm.innerPivotB;
    m_lowerArm.outerJoint = config.lowerArm.outerJoint;
    m_lowerArm.innerToOuterLengthA = armLengthA;
    m_lowerArm.innerToOuterLengthB = armLengthB;

    m_strut.upperMount = config.strut.upperMount;
    m_strut.lowerMount = configuredLowerMount;
    m_strutLowerOffsetLength = lowerMountOffsetLength;
    m_strut.length = effectiveStrutLength;

    m_upright.lowerJoint = config.lowerArm.outerJoint;
    m_upright.strutLowerJoint = m_strut.lowerMount;
    m_upright.hubPosition =
        config.lowerArm.outerJoint +
        config.upright.hubOffset;
    m_upright.hubOrientation = Quaternion::Identity();

    return true;
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
    // Reject invalid pose inputs before any geometry state is mutated.
    if (!IsFinite(chassisPosition) ||
        !IsValidOrientation(chassisOrientation) ||
        !std::isfinite(travel)) {
        return false;
    }

    // Keep the last valid geometry if this solve cannot be completed.
    const LowerArmState previousLowerArm = m_lowerArm;
    const StrutState previousStrut = m_strut;
    const UprightState previousUpright = m_upright;

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
    if (strutLength <= Epsilon) {
        m_lowerArm = previousLowerArm;
        m_strut = previousStrut;
        m_upright = previousUpright;
        return false;
    }

    if (!SolveConstraints(
            m_lowerArm,
            m_strut.upperMount,
            strutLength + m_strutLowerOffsetLength
        )) {
        m_lowerArm = previousLowerArm;
        m_strut = previousStrut;
        m_upright = previousUpright;
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

    const Vec3 worldLocalAxis =
        chassisOrientation * localAxis;

    const Quaternion strutRotation =
        FromToRotation(worldLocalAxis, strutAxis);

    const Quaternion worldUprightOrientation =
        (strutRotation * chassisOrientation).Normalized();

    m_upright.strutLowerJoint =
        m_upright.lowerJoint +
        strutRotation * (chassisOrientation * localOffset);

    m_strut.lowerMount = m_upright.strutLowerJoint;

    m_upright.hubOrientation =
        worldUprightOrientation;

    m_upright.hubPosition =
        m_upright.lowerJoint +
        strutRotation *
            (chassisOrientation * m_uprightConfig.hubOffset);
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

Vec3 MacPherson::GetSteeringAxisStart() const { return m_strut.upperMount; }
Vec3 MacPherson::GetSteeringAxisEnd() const { return m_upright.lowerJoint; }

bool MacPherson::ApplySteering(float angle) {
    const Vec3 axisDelta = m_upright.lowerJoint - m_strut.upperMount;
    const float axisLengthSquared = axisDelta.LengthSquared();
    if (axisLengthSquared < Epsilon)
        return false;
    const Vec3 axis = axisDelta / std::sqrt(axisLengthSquared);
    const Quaternion rotation = Quaternion::FromAxisAngle(axis, angle);
    const Vec3 axisPoint = m_upright.lowerJoint;
    m_upright.hubPosition = axisPoint + rotation * (m_upright.hubPosition - axisPoint);
    m_upright.strutLowerJoint = axisPoint + rotation * (m_upright.strutLowerJoint - axisPoint);
    m_strut.lowerMount = m_upright.strutLowerJoint;
    m_upright.hubOrientation = (rotation * m_upright.hubOrientation).Normalized();
    return true;
}
