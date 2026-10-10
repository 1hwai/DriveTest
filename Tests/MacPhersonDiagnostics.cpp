#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "../Vehicle/MacPherson.h"

namespace {
    constexpr float Tolerance = 0.0001f;

    bool Near(float a, float b) {
        return std::abs(a - b) <= Tolerance;
    }

    bool NearVec(const Vec3& a, const Vec3& b) {
        return Near(a.x, b.x) &&
            Near(a.y, b.y) &&
            Near(a.z, b.z);
    }

    bool Finite(const Vec3& value) {
        return std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    bool Finite(const Quaternion& value) {
        return std::isfinite(value.w) &&
            std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    bool UnitQuaternion(const Quaternion& value) {
        const float lengthSquared =
            value.w * value.w +
            value.x * value.x +
            value.y * value.y +
            value.z * value.z;

        return Near(lengthSquared, 1.0f);
    }

    MacPhersonConfig MakeConfig(float side) {
        return {
            {
                Vec3(side * 0.52f, -0.22f, 1.03f),
                Vec3(side * 0.52f, -0.22f, 1.47f),
                Vec3(side * 0.76f, -0.18f, 1.25f)
            },
            {
                Vec3(side * 0.52f, 0.68f, 1.25f),
                Vec3(0.0f, 0.26f, 0.0f),
                0.64f
            },
            {
                Vec3(0.0f, 0.10f, 0.0f)
            }
        };
    }

    Vec3 ToWorld(
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation,
        const Vec3& local
    ) {
        return chassisPosition +
            chassisOrientation * local;
    }

    bool CheckArm(
        const Vec3& outer,
        const MacPhersonArmConfig& arm,
        float lengthA,
        float lengthB
    ) {
        return Near(
            (outer - arm.innerPivotA).Length(),
            lengthA
        ) &&
        Near(
            (outer - arm.innerPivotB).Length(),
            lengthB
        );
    }
}

    bool CheckAxis(const Vec3& start, const Vec3& end) {
        const Vec3 delta = end - start;
        return Finite(start) && Finite(end) &&
            delta.Length() > 0.0001f &&
            Near(delta.Normalized().Length(), 1.0f);
    }

    bool CheckAxisRotation(const Vec3& axisStart, const Vec3& axisEnd, const Vec3& before, const Vec3& after, float angle) {
        const Vec3 axis = (axisEnd - axisStart).Normalized();
        const Vec3 radial = before - axisStart;
        const Vec3 rotated = axisStart + Quaternion::FromAxisAngle(axis, angle) * radial;
        return NearVec(after, rotated) &&
            Near(radial.Dot(axis), (after - axisStart).Dot(axis));
    }

    bool CheckSteeringState(
        MacPherson& suspension,
        const Vec3& chassisPosition,
        const Quaternion& chassisOrientation,
        float travel,
        float angle
    ) {
        if (!suspension.SolveAtTravel(
                chassisPosition,
                chassisOrientation,
                travel
            )) {
            return false;
        }

        const Vec3 axisStart = suspension.GetSteeringAxisStart();
        const Vec3 axisEnd = suspension.GetSteeringAxisEnd();
        const Vec3 before = suspension.GetHubPosition();
        const float radius = (before - axisStart).Length();

        if (!suspension.ApplySteering(angle))
            return false;

        const Vec3 after = suspension.GetHubPosition();
        const Vec3 axisStartAfter = suspension.GetSteeringAxisStart();
        const Vec3 axisEndAfter = suspension.GetSteeringAxisEnd();

        return CheckAxisRotation(
                axisStart,
                axisEnd,
                before,
                after,
                angle
            ) &&
            NearVec(axisStart, axisStartAfter) &&
            NearVec(axisEnd, axisEndAfter) &&
            Near((after - axisStart).Length(), radius) &&
            Finite(after) &&
            Finite(suspension.GetHubOrientation()) &&
            UnitQuaternion(suspension.GetHubOrientation());
    }

int main() {
    const MacPhersonConfig leftConfig = MakeConfig(1.0f);
    const MacPhersonConfig rightConfig = MakeConfig(-1.0f);

    MacPherson left;
    MacPherson right;

    left.Configure(leftConfig);
    right.Configure(rightConfig);

    const Vec3 chassisPosition(0.0f, 0.0f, 0.0f);
    const Quaternion identity = Quaternion::Identity();

    const bool solvedLeft =
        left.Solve(chassisPosition, identity);
    const bool solvedRight =
        right.Solve(chassisPosition, identity);

    const Vec3 leftJoint = left.GetLowerOuterJoint();
    const Vec3 rightJoint = right.GetLowerOuterJoint();
    const Vec3 leftMount = left.GetStrutLowerMount();
    const Vec3 rightMount = right.GetStrutLowerMount();

    const bool armLengths =
        CheckArm(
            leftJoint,
            leftConfig.lowerArm,
            left.GetLowerArmLengthA(),
            left.GetLowerArmLengthB()
        ) &&
        CheckArm(
            rightJoint,
            rightConfig.lowerArm,
            right.GetLowerArmLengthA(),
            right.GetLowerArmLengthB()
        );

    const float leftStrutLength =
        (leftConfig.strut.upperMount - leftMount).Length();
    const float rightStrutLength =
        (rightConfig.strut.upperMount - rightMount).Length();

    const bool strutConstraint =
        Near(leftStrutLength, left.GetStrutLength()) &&
        Near(rightStrutLength, right.GetStrutLength());

    const bool symmetry =
        Near(leftJoint.x, -rightJoint.x) &&
        Near(leftJoint.y, rightJoint.y) &&
        Near(leftJoint.z, rightJoint.z) &&
        Near(leftMount.x, -rightMount.x) &&
        Near(leftMount.y, rightMount.y) &&
        Near(leftMount.z, rightMount.z);

    const Vec3 expectedLeftHub =
        leftJoint +
        left.GetHubOrientation() * leftConfig.upright.hubOffset;
    const Vec3 expectedRightHub =
        rightJoint +
        right.GetHubOrientation() * rightConfig.upright.hubOffset;

    const bool hubPosition =
        NearVec(left.GetHubPosition(), expectedLeftHub) &&
        NearVec(right.GetHubPosition(), expectedRightHub);

    const Vec3 leftAxisStart = left.GetSteeringAxisStart();
    const Vec3 leftAxisEnd = left.GetSteeringAxisEnd();
    const Vec3 rightAxisStart = right.GetSteeringAxisStart();
    const Vec3 rightAxisEnd = right.GetSteeringAxisEnd();

    const bool steeringAxis =
        CheckAxis(leftAxisStart, leftAxisEnd) &&
        CheckAxis(rightAxisStart, rightAxisEnd) &&
        Near(leftAxisStart.x, -rightAxisStart.x) &&
        Near(leftAxisStart.y, rightAxisStart.y) &&
        Near(leftAxisStart.z, rightAxisStart.z) &&
        Near(leftAxisEnd.x, -rightAxisEnd.x) &&
        Near(leftAxisEnd.y, rightAxisEnd.y) &&
        Near(leftAxisEnd.z, rightAxisEnd.z);

    const Vec3 zeroBefore = left.GetHubPosition();
    const Quaternion zeroOrientation = left.GetHubOrientation();
    const bool zeroSteering =
        left.ApplySteering(0.0f) &&
        NearVec(left.GetHubPosition(), zeroBefore) &&
        Near(left.GetHubOrientation().w, zeroOrientation.w) &&
        Near(left.GetHubOrientation().x, zeroOrientation.x) &&
        Near(left.GetHubOrientation().y, zeroOrientation.y) &&
        Near(left.GetHubOrientation().z, zeroOrientation.z);

    left.Solve(chassisPosition, identity);
    const Vec3 steeringBefore = left.GetHubPosition();
    const Vec3 steeringAxisStart = left.GetSteeringAxisStart();
    const Vec3 steeringAxisEnd = left.GetSteeringAxisEnd();
    const bool steeringRotation =
        left.ApplySteering(0.35f) &&
        CheckAxisRotation(
            steeringAxisStart,
            steeringAxisEnd,
            steeringBefore,
            left.GetHubPosition(),
            0.35f
        ) &&
        NearVec(steeringAxisStart, left.GetSteeringAxisStart()) &&
        NearVec(steeringAxisEnd, left.GetSteeringAxisEnd()) &&
        Near(
            (left.GetHubPosition() - steeringAxisStart).Length(),
            (steeringBefore - steeringAxisStart).Length()
        ) &&
        UnitQuaternion(left.GetHubOrientation());

    left.Solve(chassisPosition, identity);
    right.Solve(chassisPosition, identity);
    left.ApplySteering(0.35f);
    right.ApplySteering(-0.35f);

    const bool steeringSymmetry =
        Near(left.GetHubPosition().x, -right.GetHubPosition().x) &&
        Near(left.GetHubPosition().y, right.GetHubPosition().y) &&
        Near(left.GetHubPosition().z, right.GetHubPosition().z);

    const bool steeringSign =
        CheckSteeringState(
            left,
            chassisPosition,
            identity,
            0.0f,
            0.35f
        ) &&
        CheckSteeringState(
            left,
            chassisPosition,
            identity,
            0.0f,
            -0.35f
        );

    const Vec3 leftSpringA = left.GetSpringMountA();
    const Vec3 leftSpringB = left.GetSpringMountB();
    const Vec3 rightSpringA = right.GetSpringMountA();
    const Vec3 rightSpringB = right.GetSpringMountB();

    const bool springMounts =
        Finite(leftSpringA) && Finite(leftSpringB) &&
        Finite(rightSpringA) && Finite(rightSpringB) &&
        Near(leftSpringA.x, -rightSpringA.x) &&
        Near(leftSpringA.y, rightSpringA.y) &&
        Near(leftSpringA.z, rightSpringA.z) &&
        Near(leftSpringB.x, -rightSpringB.x) &&
        Near(leftSpringB.y, rightSpringB.y) &&
        Near(leftSpringB.z, rightSpringB.z) &&
        (leftSpringB - leftSpringA).Length() > 0.0001f &&
        (rightSpringB - rightSpringA).Length() > 0.0001f;

    const Vec3 hubBeforeInvalidTravel = left.GetHubPosition();
    const Vec3 lowerJointBeforeInvalidTravel = left.GetLowerOuterJoint();
    const Vec3 springMountABeforeInvalidTravel = left.GetSpringMountA();
    const Vec3 springMountBBeforeInvalidTravel = left.GetSpringMountB();
    const Quaternion orientationBeforeInvalidTravel = left.GetHubOrientation();
    const auto geometryUnchangedAfterRejectedTravel = [&]() {
        const Quaternion orientation = left.GetHubOrientation();
        return NearVec(left.GetHubPosition(), hubBeforeInvalidTravel) &&
            NearVec(left.GetLowerOuterJoint(), lowerJointBeforeInvalidTravel) &&
            NearVec(left.GetSpringMountA(), springMountABeforeInvalidTravel) &&
            NearVec(left.GetSpringMountB(), springMountBBeforeInvalidTravel) &&
            Near(orientation.w, orientationBeforeInvalidTravel.w) &&
            Near(orientation.x, orientationBeforeInvalidTravel.x) &&
            Near(orientation.y, orientationBeforeInvalidTravel.y) &&
            Near(orientation.z, orientationBeforeInvalidTravel.z);
    };
    const bool nanTravelRejected =
        !left.SolveAtTravel(
            chassisPosition,
            identity,
            std::numeric_limits<float>::quiet_NaN()
        ) &&
        geometryUnchangedAfterRejectedTravel();
    const bool infiniteTravelRejected =
        !left.SolveAtTravel(
            chassisPosition,
            identity,
            std::numeric_limits<float>::infinity()
        ) &&
        geometryUnchangedAfterRejectedTravel();
    const bool impossibleFiniteTravelRejected =
        !left.SolveAtTravel(
            chassisPosition,
            identity,
            left.GetStrutLength()
        ) &&
        geometryUnchangedAfterRejectedTravel();
    const bool invalidTravelRejected =
        nanTravelRejected &&
        infiniteTravelRejected &&
        impossibleFiniteTravelRejected;

    // Exercise the geometric-constraint failure path, not only the
    // early invalid-strut-length guard. The requested distance is outside
    // the arm/strut intersection range for this diagnostic configuration.
    const bool unreachableConstraintRejected =
        !left.SolveAtTravel(
            chassisPosition,
            identity,
            0.30f
        ) &&
        geometryUnchangedAfterRejectedTravel();

    const auto rejectedPosePreservesGeometry = [&](
        const Vec3& position,
        const Quaternion& orientation
    ) {
        return !left.SolveAtTravel(position, orientation, 0.0f) &&
            geometryUnchangedAfterRejectedTravel();
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const bool nonFinitePositionRejected =
        rejectedPosePreservesGeometry(
            Vec3(nan, 0.0f, 0.0f), identity
        ) &&
        rejectedPosePreservesGeometry(
            Vec3(0.0f, infinity, 0.0f), identity
        );
    const bool invalidOrientationRejected =
        rejectedPosePreservesGeometry(
            chassisPosition,
            Quaternion(nan, 0.0f, 0.0f, 1.0f)
        ) &&
        rejectedPosePreservesGeometry(
            chassisPosition,
            Quaternion(1.0f, 0.0f, infinity, 0.0f)
        ) &&
        rejectedPosePreservesGeometry(
            chassisPosition,
            Quaternion(0.0f, 0.0f, 0.0f, 0.0f)
        ) &&
        rejectedPosePreservesGeometry(
            chassisPosition,
            Quaternion(2.0f, 0.0f, 0.0f, 0.0f)
        );
    const bool invalidChassisPoseRejected =
        nonFinitePositionRejected && invalidOrientationRejected;

    const bool finite =
        solvedLeft &&
        solvedRight &&
        Finite(left.GetHubPosition()) &&
        Finite(right.GetHubPosition()) &&
        Finite(left.GetStrutLowerMount()) &&
        Finite(right.GetStrutLowerMount()) &&
        Finite(left.GetHubOrientation()) &&
        Finite(right.GetHubOrientation());

    bool travel = true;
    float maxHubStep = 0.0f;
    float maxHorizontalDrift = 0.0f;

    // Establish an unsteered reference at zero travel before sweeping.
    if (!left.SolveAtTravel(chassisPosition, identity, 0.0f))
        travel = false;
    const Vec3 staticLeftHub = left.GetHubPosition();

    Vec3 previousLeftHub;
    bool hasPrevious = false;

    // Save the forward sweep so the reverse sweep can verify that branch
    // selection depends on the configured reference, not solve call order.
    constexpr int TravelSteps = 30;
    constexpr float TravelStep = 0.005f;
    Vec3 forwardLeftJoints[TravelSteps + 1];
    Vec3 forwardRightJoints[TravelSteps + 1];
    Vec3 forwardLeftHubs[TravelSteps + 1];
    Vec3 forwardRightHubs[TravelSteps + 1];

    for (int i = 0; i <= TravelSteps; ++i) {
        const float value =
            -0.075f + static_cast<float>(i) * TravelStep;
        const bool solvedAtTravelLeft =
            left.SolveAtTravel(
                chassisPosition,
                identity,
                value
            );
        const bool solvedAtTravelRight =
            right.SolveAtTravel(
                chassisPosition,
                identity,
                value
            );

        if (!solvedAtTravelLeft || !solvedAtTravelRight) {
            travel = false;
            continue;
        }

        const Vec3 leftAtTravel =
            left.GetLowerOuterJoint();
        const Vec3 rightAtTravel =
            right.GetLowerOuterJoint();
        const Vec3 leftHubAtTravel =
            left.GetHubPosition();
        const Vec3 rightHubAtTravel =
            right.GetHubPosition();

        forwardLeftJoints[i] = leftAtTravel;
        forwardRightJoints[i] = rightAtTravel;
        forwardLeftHubs[i] = leftHubAtTravel;
        forwardRightHubs[i] = rightHubAtTravel;

        const float expectedStrutLength =
            left.GetStrutLength() - value;

        const float actualLeftStrut =
            (leftConfig.strut.upperMount -
             left.GetStrutLowerMount()).Length();
        const float actualRightStrut =
            (rightConfig.strut.upperMount -
             right.GetStrutLowerMount()).Length();

        travel =
            CheckArm(
                leftAtTravel,
                leftConfig.lowerArm,
                left.GetLowerArmLengthA(),
                left.GetLowerArmLengthB()
            ) &&
            CheckArm(
                rightAtTravel,
                rightConfig.lowerArm,
                right.GetLowerArmLengthA(),
                right.GetLowerArmLengthB()
            ) &&
            Near(actualLeftStrut, expectedStrutLength) &&
            Near(actualRightStrut, expectedStrutLength) &&
            Near(leftAtTravel.x, -rightAtTravel.x) &&
            Near(leftAtTravel.y, rightAtTravel.y) &&
            Near(leftAtTravel.z, rightAtTravel.z) &&
            Near(leftHubAtTravel.x, -rightHubAtTravel.x) &&
            Near(leftHubAtTravel.y, rightHubAtTravel.y) &&
            Near(leftHubAtTravel.z, rightHubAtTravel.z) &&
            NearVec(
                leftHubAtTravel,
                leftAtTravel +
                left.GetHubOrientation() *
                    leftConfig.upright.hubOffset
            ) &&
            Finite(leftAtTravel) &&
            Finite(rightAtTravel) &&
            Finite(leftHubAtTravel) &&
            Finite(rightHubAtTravel) &&
            Finite(left.GetHubOrientation()) &&
            Finite(right.GetHubOrientation()) &&
            UnitQuaternion(left.GetHubOrientation()) &&
            UnitQuaternion(right.GetHubOrientation()) &&
            travel;

        maxHorizontalDrift = std::max(
            maxHorizontalDrift,
            std::abs(leftHubAtTravel.x - staticLeftHub.x)
        );
        maxHorizontalDrift = std::max(
            maxHorizontalDrift,
            std::abs(leftHubAtTravel.z - staticLeftHub.z)
        );

        if (hasPrevious) {
            maxHubStep = std::max(
                maxHubStep,
                (leftHubAtTravel - previousLeftHub).Length()
            );
        }

        previousLeftHub = leftHubAtTravel;
        hasPrevious = true;
    }

    bool branchSelectionStable = true;
    for (int i = TravelSteps; i >= 0; --i) {
        const float value =
            -0.075f + static_cast<float>(i) * TravelStep;
        const bool solvedLeftReverse =
            left.SolveAtTravel(chassisPosition, identity, value);
        const bool solvedRightReverse =
            right.SolveAtTravel(chassisPosition, identity, value);

        branchSelectionStable =
            solvedLeftReverse &&
            solvedRightReverse &&
            NearVec(left.GetLowerOuterJoint(), forwardLeftJoints[i]) &&
            NearVec(right.GetLowerOuterJoint(), forwardRightJoints[i]) &&
            NearVec(left.GetHubPosition(), forwardLeftHubs[i]) &&
            NearVec(right.GetHubPosition(), forwardRightHubs[i]) &&
            branchSelectionStable;
    }

    const bool endpoints =
        left.SolveAtTravel(
            chassisPosition,
            identity,
            -0.075f
        ) &&
        left.SolveAtTravel(
            chassisPosition,
            identity,
            0.075f
        ) &&
        right.SolveAtTravel(
            chassisPosition,
            identity,
            -0.075f
        ) &&
        right.SolveAtTravel(
            chassisPosition,
            identity,
            0.075f
        );

    bool poseSweep = true;
    bool poseConstraints = true;
    const float rollAngles[] = {
        -0.35f, -0.175f, 0.0f, 0.175f, 0.35f
    };
    const float pitchAngles[] = {
        -0.20f, 0.0f, 0.20f
    };

    const Vec3 poseChassisPosition(3.0f, -2.0f, 4.0f);
    const float poseTravels[] = {
        -0.05f, 0.0f, 0.05f
    };

    for (float roll : rollAngles) {
        for (float pitch : pitchAngles) {
            const Quaternion orientation =
                (
                    Quaternion::FromAxisAngle(
                        Vec3(1.0f, 0.0f, 0.0f),
                        pitch
                    ) *
                    Quaternion::FromAxisAngle(
                        Vec3(0.0f, 0.0f, 1.0f),
                        roll
                    )
                ).Normalized();

            for (float poseTravel : poseTravels) {
                const bool solvedPoseLeft =
                    left.SolveAtTravel(
                        poseChassisPosition,
                        orientation,
                        poseTravel
                    );
                const bool solvedPoseRight =
                    right.SolveAtTravel(
                        poseChassisPosition,
                        orientation,
                        poseTravel
                    );

                if (!solvedPoseLeft || !solvedPoseRight) {
                    poseSweep = false;
                    continue;
                }

                const Vec3 leftPivotA =
                    ToWorld(
                        poseChassisPosition,
                        orientation,
                        leftConfig.lowerArm.innerPivotA
                    );
                const Vec3 leftPivotB =
                    ToWorld(
                        poseChassisPosition,
                        orientation,
                        leftConfig.lowerArm.innerPivotB
                    );
                const Vec3 rightPivotA =
                    ToWorld(
                        poseChassisPosition,
                        orientation,
                        rightConfig.lowerArm.innerPivotA
                    );
                const Vec3 rightPivotB =
                    ToWorld(
                        poseChassisPosition,
                        orientation,
                        rightConfig.lowerArm.innerPivotB
                    );
                const Vec3 leftUpperMount =
                    ToWorld(
                        poseChassisPosition,
                        orientation,
                        leftConfig.strut.upperMount
                    );
                const Vec3 rightUpperMount =
                    ToWorld(
                        poseChassisPosition,
                        orientation,
                        rightConfig.strut.upperMount
                    );

                const float expectedStrutLength =
                    left.GetStrutLength() - poseTravel;
                const bool currentPoseConstraints =
                    Near(
                        (left.GetLowerOuterJoint() - leftPivotA).Length(),
                        left.GetLowerArmLengthA()
                    ) &&
                    Near(
                        (left.GetLowerOuterJoint() - leftPivotB).Length(),
                        left.GetLowerArmLengthB()
                    ) &&
                    Near(
                        (right.GetLowerOuterJoint() - rightPivotA).Length(),
                        right.GetLowerArmLengthA()
                    ) &&
                    Near(
                        (right.GetLowerOuterJoint() - rightPivotB).Length(),
                        right.GetLowerArmLengthB()
                    ) &&
                    Near(
                        (leftUpperMount - left.GetStrutLowerMount()).Length(),
                        expectedStrutLength
                    ) &&
                    Near(
                        (rightUpperMount - right.GetStrutLowerMount()).Length(),
                        expectedStrutLength
                    );
                poseConstraints =
                    currentPoseConstraints && poseConstraints;

                const bool hubOffsetInvariant =
                    Near(
                        (left.GetHubPosition() - left.GetLowerOuterJoint()).Length(),
                        leftConfig.upright.hubOffset.Length()
                    ) &&
                    Near(
                        (right.GetHubPosition() - right.GetLowerOuterJoint()).Length(),
                        rightConfig.upright.hubOffset.Length()
                    ) &&
                    Near(
                        (left.GetStrutLowerMount() - left.GetLowerOuterJoint()).Length(),
                        leftConfig.strut.lowerMountOffset.Length()
                    ) &&
                    Near(
                        (right.GetStrutLowerMount() - right.GetLowerOuterJoint()).Length(),
                        rightConfig.strut.lowerMountOffset.Length()
                    );

                const Vec3 leftLocalHub =
                    orientation.Conjugate() *
                    (left.GetHubPosition() - poseChassisPosition);
                const Vec3 rightLocalHub =
                    orientation.Conjugate() *
                    (right.GetHubPosition() - poseChassisPosition);

                poseSweep =
                    hubOffsetInvariant &&
                    currentPoseConstraints &&
                    Finite(leftLocalHub) &&
                    Finite(rightLocalHub) &&
                    Finite(left.GetHubOrientation()) &&
                    Finite(right.GetHubOrientation()) &&
                    UnitQuaternion(left.GetHubOrientation()) &&
                    UnitQuaternion(right.GetHubOrientation()) &&
                    Near(leftLocalHub.x, -rightLocalHub.x) &&
                    Near(leftLocalHub.y, rightLocalHub.y) &&
                    Near(leftLocalHub.z, rightLocalHub.z) &&
                    poseSweep;
            }
        }
    }

    const bool continuity =
        maxHubStep < 0.01f &&
        maxHorizontalDrift < 0.20f;

    const bool orientation =
        UnitQuaternion(left.GetHubOrientation()) &&
        UnitQuaternion(right.GetHubOrientation());

    std::cout
        << "[MacPherson]"
        << " solved=" << (solvedLeft && solvedRight)
        << " armLengths=" << armLengths
        << " strutConstraint=" << strutConstraint
        << " symmetry=" << symmetry
        << " hubPosition=" << hubPosition
        << " steeringAxis=" << steeringAxis
        << " zeroSteering=" << zeroSteering
        << " steeringRotation=" << steeringRotation
        << " finite=" << finite
        << " invalidTravelRejected=" << invalidTravelRejected
        << " unreachableConstraintRejected=" << unreachableConstraintRejected
        << " nonFinitePositionRejected=" << nonFinitePositionRejected
        << " invalidOrientationRejected=" << invalidOrientationRejected
        << " invalidChassisPoseRejected=" << invalidChassisPoseRejected
        << " travel=" << travel
        << " endpoints=" << endpoints
        << " poseSweep=" << poseSweep
        << " poseConstraints=" << poseConstraints
        << " continuity=" << continuity
        << " branchSelectionStable=" << branchSelectionStable
        << " maxHubStep=" << maxHubStep
        << " maxHorizontalDrift=" << maxHorizontalDrift
        << '\n';

    if (!finite ||
        !invalidTravelRejected ||
        !unreachableConstraintRejected ||
        !invalidChassisPoseRejected ||
        !armLengths ||
        !strutConstraint ||
        !symmetry ||
        !hubPosition ||
        !steeringAxis ||
        !zeroSteering ||
        !steeringRotation ||
        !travel ||
        !endpoints ||
        !poseSweep ||
        !poseConstraints ||
        !orientation ||
        !continuity ||
        !branchSelectionStable) {
        std::cerr
            << "[FAIL] MacPherson travel diagnostics";
        return 1;
    }

    std::cout
        << "[PASS] MacPherson travel diagnostics";
    return 0;
}
