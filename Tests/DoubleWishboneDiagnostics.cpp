#include <cmath>
#include <iostream>
#include <algorithm>

#include "../Vehicle/DoubleWishbone.h"

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

    DoubleWishboneConfig MakeConfig(float side) {
        const float z = 1.25f;

        return {
            {
                Vec3(side * 0.55f, 0.15f, z - 0.18f),
                Vec3(side * 0.55f, 0.15f, z + 0.18f),
                Vec3(side * 0.72f, 0.10f, z)
            },
            {
                Vec3(side * 0.55f, -0.25f, z - 0.18f),
                Vec3(side * 0.55f, -0.25f, z + 0.18f),
                Vec3(side * 0.76f, -0.20f, z)
            },
            {
                Vec3(0.0f, 0.10f, 0.0f)
            }
        };
    }


    bool CheckArm(
        const Vec3& outer,
        const DoubleWishboneArmConfig& arm
    ) {
        const float lengthA =
            (outer - arm.innerPivotA).Length();
        const float lengthB =
            (outer - arm.innerPivotB).Length();
        const float expectedA =
            (arm.outerJoint - arm.innerPivotA).Length();
        const float expectedB =
            (arm.outerJoint - arm.innerPivotB).Length();

        return Near(lengthA, expectedA) &&
            Near(lengthB, expectedB);
    }

    bool CheckGeometry(
        const DoubleWishbone& suspension,
        const DoubleWishboneConfig& config
    ) {
        const Vec3 upper = suspension.GetUpperOuterJoint();
        const Vec3 lower = suspension.GetLowerOuterJoint();

        return CheckArm(upper, config.upperArm) &&
            CheckArm(lower, config.lowerArm) &&
            Near(
                (upper - lower).Length(),
                (config.upperArm.outerJoint -
                 config.lowerArm.outerJoint).Length()
            );
    }


}

int main() {
    const DoubleWishboneConfig leftConfig =
        MakeConfig(1.0f);
    const DoubleWishboneConfig rightConfig =
        MakeConfig(-1.0f);

    DoubleWishbone left;
    DoubleWishbone right;

    left.Configure(leftConfig);
    right.Configure(rightConfig);

    const Vec3 chassisPosition(0.0f, 0.78f, 0.0f);
    const Quaternion chassisOrientation =
        Quaternion::Identity();

    left.Solve(chassisPosition, chassisOrientation);
    right.Solve(chassisPosition, chassisOrientation);

    const Vec3 leftUpper = left.GetUpperOuterJoint();
    const Vec3 leftLower = left.GetLowerOuterJoint();
    const Vec3 rightUpper = right.GetUpperOuterJoint();
    const Vec3 rightLower = right.GetLowerOuterJoint();

    const bool armLengths =
        CheckArm(leftUpper, leftConfig.upperArm) &&
        CheckArm(leftLower, leftConfig.lowerArm) &&
        CheckArm(rightUpper, rightConfig.upperArm) &&
        CheckArm(rightLower, rightConfig.lowerArm);

    const float expectedUprightDistance =
        (leftConfig.upperArm.outerJoint -
         leftConfig.lowerArm.outerJoint).Length();

    const bool uprightConstraint =
        Near(
            (leftUpper - leftLower).Length(),
            expectedUprightDistance
        ) &&
        Near(
            (rightUpper - rightLower).Length(),
            expectedUprightDistance
        );

    const bool symmetry =
        Near(leftUpper.x, -rightUpper.x) &&
        Near(leftUpper.y, rightUpper.y) &&
        Near(leftUpper.z, rightUpper.z) &&
        Near(leftLower.x, -rightLower.x) &&
        Near(leftLower.y, rightLower.y) &&
        Near(leftLower.z, rightLower.z);

    const Vec3 leftHub = left.GetHubPosition();
    const Vec3 rightHub = right.GetHubPosition();

    const bool hubSymmetry =
        Near(leftHub.x, -rightHub.x) &&
        Near(leftHub.y, rightHub.y) &&
        Near(leftHub.z, rightHub.z);

    const Vec3 expectedLeftHub =
        chassisPosition +
        chassisOrientation * (
            leftLower + leftConfig.upright.hubOffset
        );
    const Vec3 expectedRightHub =
        chassisPosition +
        chassisOrientation * (
            rightLower + rightConfig.upright.hubOffset
        );

    const bool hubPosition =
        NearVec(leftHub, expectedLeftHub) &&
        NearVec(rightHub, expectedRightHub);

    const Vec3 leftSpringA = left.GetSpringMountA();
    const Vec3 leftSpringB = left.GetSpringMountB();
    const Vec3 rightSpringA = right.GetSpringMountA();
    const Vec3 rightSpringB = right.GetSpringMountB();

    const bool springMounts =
        Finite(leftSpringA) && Finite(leftSpringB) &&
        Finite(rightSpringA) && Finite(rightSpringB) &&
        NearVec(leftSpringA, Vec3(0.0f, 0.0f, 0.0f)) == false &&
        NearVec(rightSpringA, Vec3(0.0f, 0.0f, 0.0f)) == false &&
        Near(leftSpringA.x, -rightSpringA.x) &&
        Near(leftSpringA.y, rightSpringA.y) &&
        Near(leftSpringA.z, rightSpringA.z) &&
        Near(leftSpringB.x, -rightSpringB.x) &&
        Near(leftSpringB.y, rightSpringB.y) &&
        Near(leftSpringB.z, rightSpringB.z) &&
        (leftSpringB - leftSpringA).Length() > 0.0001f &&
        (rightSpringB - rightSpringA).Length() > 0.0001f;

    const bool finite =
        Finite(leftHub) &&
        Finite(rightHub) &&
        Finite(left.GetHubOrientation()) &&
        Finite(right.GetHubOrientation());

    bool travel = true;
    const float travels[] = {
        -0.075f, -0.05f, -0.025f,
        0.0f,
        0.025f, 0.05f, 0.075f
    };

    float previousHubY = 0.0f;
    bool hasPrevious = false;
    float maxHubStep = 0.0f;
    float maxHorizontalDrift = 0.0f;

    for (float value : travels) {
        const bool solvedLeft =
            left.SolveAtTravel(
                chassisPosition,
                chassisOrientation,
                value
            );
        const bool solvedRight =
            right.SolveAtTravel(
                chassisPosition,
                chassisOrientation,
                value
            );

        if (!solvedLeft || !solvedRight) {
            travel = false;
            continue;
        }

        travel =
            CheckGeometry(left, leftConfig) &&
            CheckGeometry(right, rightConfig) &&
            travel;

        const Vec3 leftHubAtTravel =
            left.GetHubPosition();
        const Vec3 rightHubAtTravel =
            right.GetHubPosition();

        travel =
            Near(leftHubAtTravel.x, -rightHubAtTravel.x) &&
            Near(leftHubAtTravel.y, rightHubAtTravel.y) &&
            Near(leftHubAtTravel.z, rightHubAtTravel.z) &&
            travel;

        maxHorizontalDrift = std::max(
            maxHorizontalDrift,
            std::abs(
                leftHubAtTravel.x -
                leftHub.x
            )
        );
        maxHorizontalDrift = std::max(
            maxHorizontalDrift,
            std::abs(
                leftHubAtTravel.z -
                leftHub.z
            )
        );

        if (hasPrevious) {
            maxHubStep = std::max(
                maxHubStep,
                std::abs(
                    leftHubAtTravel.y -
                    previousHubY
                )
            );
        }

        previousHubY = leftHubAtTravel.y;
        hasPrevious = true;
    }

    const bool endpoints =
        left.SolveAtTravel(
            chassisPosition,
            chassisOrientation,
            -0.075f
        ) &&
        left.SolveAtTravel(
            chassisPosition,
            chassisOrientation,
            0.075f
        ) &&
        right.SolveAtTravel(
            chassisPosition,
            chassisOrientation,
            -0.075f
        ) &&
        right.SolveAtTravel(
            chassisPosition,
            chassisOrientation,
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

            const bool solvedLeft =
                left.SolveAtTravel(
                    chassisPosition,
                    orientation,
                    0.0f
                );
            const bool solvedRight =
                right.SolveAtTravel(
                    chassisPosition,
                    orientation,
                    0.0f
                );

            if (!solvedLeft || !solvedRight) {
                poseSweep = false;
                continue;
            }

            const Vec3 leftPoseHub =
                orientation.Conjugate() *
                (left.GetHubPosition() - chassisPosition);
            const Vec3 rightPoseHub =
                orientation.Conjugate() *
                (right.GetHubPosition() - chassisPosition);

            poseSweep =
                Finite(leftPoseHub) &&
                Finite(rightPoseHub) &&
                Finite(left.GetHubOrientation()) &&
                Finite(right.GetHubOrientation()) &&
                Near(leftPoseHub.x, -rightPoseHub.x) &&
                Near(leftPoseHub.y, rightPoseHub.y) &&
                Near(leftPoseHub.z, rightPoseHub.z) &&
                poseSweep;
        }
    }

    const bool continuity =
        maxHubStep < 0.05f &&
        maxHorizontalDrift < 0.20f;

    std::cout
        << "[DoubleWishbone]"
        << " armLengths=" << armLengths
        << " uprightConstraint=" << uprightConstraint
        << " symmetry=" << symmetry
        << " hubSymmetry=" << hubSymmetry
        << " hubPosition=" << hubPosition
        << " finite=" << finite
        << " travel=" << travel
        << " endpoints=" << endpoints
        << " continuity=" << continuity
        << " maxHubStep=" << maxHubStep
        << " maxHorizontalDrift=" << maxHorizontalDrift
        << '\n';

    if (!armLengths ||
        !uprightConstraint ||
        !symmetry ||
        !hubSymmetry ||
        !hubPosition ||
        !finite ||
        !travel ||
        !endpoints ||
        !poseSweep ||
        !continuity) {
        std::cerr
            << "[FAIL] Double wishbone travel diagnostics\n";
        return 1;
    }

    std::cout
        << "[PASS] Double wishbone travel diagnostics\n";
    return 0;
}
