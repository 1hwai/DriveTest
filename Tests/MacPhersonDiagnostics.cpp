#include <algorithm>
#include <cmath>
#include <iostream>

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
        CheckAxisRotation(steeringAxisStart, steeringAxisEnd, steeringBefore, left.GetHubPosition(), 0.35f) &&
        UnitQuaternion(left.GetHubOrientation());

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
    const Vec3 staticLeftHub = left.GetHubPosition();
    Vec3 previousLeftHub;
    bool hasPrevious = false;

    const float travels[] = {
        -0.075f, -0.05f, -0.025f,
        0.0f,
        0.025f, 0.05f, 0.075f
    };

    for (float value : travels) {
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
    const float rollAngles[] = {
        -0.35f, -0.175f, 0.0f, 0.175f, 0.35f
    };
    const float pitchAngles[] = {
        -0.20f, 0.0f, 0.20f
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

            const bool solvedPoseLeft =
                left.SolveAtTravel(
                    chassisPosition,
                    orientation,
                    0.0f
                );
            const bool solvedPoseRight =
                right.SolveAtTravel(
                    chassisPosition,
                    orientation,
                    0.0f
                );

            if (!solvedPoseLeft || !solvedPoseRight) {
                poseSweep = false;
                continue;
            }

            const Vec3 leftLocalHub =
                orientation.Conjugate() *
                (left.GetHubPosition() - chassisPosition);
            const Vec3 rightLocalHub =
                orientation.Conjugate() *
                (right.GetHubPosition() - chassisPosition);

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

            poseSweep =
                hubOffsetInvariant &&
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

    const bool continuity =
        maxHubStep < 0.05f &&
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
        << " travel=" << travel
        << " endpoints=" << endpoints
        << " poseSweep=" << poseSweep
        << " continuity=" << continuity
        << " maxHubStep=" << maxHubStep
        << " maxHorizontalDrift=" << maxHorizontalDrift
        << '\n';

    if (!finite ||
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
        !orientation ||
        !continuity) {
        std::cerr
            << "[FAIL] MacPherson travel diagnostics";
        return 1;
    }

    std::cout
        << "[PASS] MacPherson travel diagnostics";
    return 0;
}
