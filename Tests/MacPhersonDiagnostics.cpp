#include <cmath>
#include <iostream>

#include "../Vehicle/MacPherson.h"

namespace {
    constexpr float Tolerance = 0.0001f;

    bool Near(float a, float b) {
        return std::abs(a - b) <= Tolerance;
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
}

int main() {
    const MacPhersonConfig leftConfig = MakeConfig(1.0f);
    const MacPhersonConfig rightConfig = MakeConfig(-1.0f);

    MacPherson left;
    MacPherson right;

    left.Configure(leftConfig);
    right.Configure(rightConfig);

    const Vec3 chassisPosition(0.0f, 0.0f, 0.0f);
    const Quaternion chassisOrientation =
        Quaternion::Identity();

    const bool solvedLeft =
        left.Solve(chassisPosition, chassisOrientation);
    const bool solvedRight =
        right.Solve(chassisPosition, chassisOrientation);

    const Vec3 leftJoint = left.GetLowerOuterJoint();
    const Vec3 rightJoint = right.GetLowerOuterJoint();
    const Vec3 leftMount = left.GetStrutLowerMount();
    const Vec3 rightMount = right.GetStrutLowerMount();

    const bool armLengths =
        Near(
            (leftJoint - leftConfig.lowerArm.innerPivotA).Length(),
            left.GetLowerArmLengthA()
        ) &&
        Near(
            (leftJoint - leftConfig.lowerArm.innerPivotB).Length(),
            left.GetLowerArmLengthB()
        ) &&
        Near(
            (rightJoint - rightConfig.lowerArm.innerPivotA).Length(),
            right.GetLowerArmLengthA()
        ) &&
        Near(
            (rightJoint - rightConfig.lowerArm.innerPivotB).Length(),
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
        Near(left.GetHubPosition().x, expectedLeftHub.x) &&
        Near(left.GetHubPosition().y, expectedLeftHub.y) &&
        Near(left.GetHubPosition().z, expectedLeftHub.z) &&
        Near(right.GetHubPosition().x, expectedRightHub.x) &&
        Near(right.GetHubPosition().y, expectedRightHub.y) &&
        Near(right.GetHubPosition().z, expectedRightHub.z);

    const bool finite =
        solvedLeft &&
        solvedRight &&
        Finite(left.GetHubPosition()) &&
        Finite(right.GetHubPosition()) &&
        Finite(left.GetStrutLowerMount()) &&
        Finite(right.GetStrutLowerMount()) &&
        Finite(left.GetHubOrientation()) &&
        Finite(right.GetHubOrientation());

    std::cout
        << "[MacPherson]"
        << " solved=" << (solvedLeft && solvedRight)
        << " armLengths=" << armLengths
        << " strutConstraint=" << strutConstraint
        << " symmetry=" << symmetry
        << " hubPosition=" << hubPosition
        << " finite=" << finite
        << '\n';

    if (!finite ||
        !armLengths ||
        !strutConstraint ||
        !symmetry ||
        !hubPosition) {
        std::cerr
            << "[FAIL] MacPherson diagnostics\n";
        return 1;
    }

    std::cout
        << "[PASS] MacPherson diagnostics\n";
    return 0;
}
