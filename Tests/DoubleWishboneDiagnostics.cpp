#include <cmath>
#include <iostream>

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

    const bool finite =
        Finite(leftHub) &&
        Finite(rightHub) &&
        Finite(left.GetHubOrientation()) &&
        Finite(right.GetHubOrientation());

    std::cout
        << "[DoubleWishbone]"
        << " armLengths=" << armLengths
        << " uprightConstraint=" << uprightConstraint
        << " symmetry=" << symmetry
        << " hubSymmetry=" << hubSymmetry
        << " hubPosition=" << hubPosition
        << " finite=" << finite
        << '\n';

    if (!armLengths ||
        !uprightConstraint ||
        !symmetry ||
        !hubSymmetry ||
        !hubPosition ||
        !finite) {
        std::cerr
            << "[FAIL] Double wishbone static geometry diagnostics\n";
        return 1;
    }

    std::cout
        << "[PASS] Double wishbone static geometry diagnostics\n";
    return 0;
}
