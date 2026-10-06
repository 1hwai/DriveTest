#include <cmath>
#include <iostream>

#include "../Vehicle/DoubleWishbone.h"

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

    DoubleWishboneConfig MakeConfig(float side) {
        const float z = 1.25f;

        return {
            Vec3(side * 0.55f, 0.15f, z - 0.18f),
            Vec3(side * 0.55f, 0.15f, z + 0.18f),
            Vec3(side * 0.72f, 0.10f, z),
            Vec3(side * 0.55f, -0.25f, z - 0.18f),
            Vec3(side * 0.55f, -0.25f, z + 0.18f),
            Vec3(side * 0.76f, -0.20f, z),
            Vec3(0.0f, 0.10f, 0.0f)
        };
    }
}

int main() {
    DoubleWishbone left;
    DoubleWishbone right;

    left.Configure(MakeConfig(1.0f));
    right.Configure(MakeConfig(-1.0f));

    const Vec3 chassisPosition(0.0f, 0.78f, 0.0f);
    const Quaternion chassisOrientation =
        Quaternion::Identity();

    left.Solve(chassisPosition, chassisOrientation);
    right.Solve(chassisPosition, chassisOrientation);

    const Vec3 leftUpper = left.GetUpperOuterJoint();
    const Vec3 leftLower = left.GetLowerOuterJoint();
    const Vec3 rightUpper = right.GetUpperOuterJoint();
    const Vec3 rightLower = right.GetLowerOuterJoint();

    const float leftUpperLengthA =
        (leftUpper - Vec3(0.55f, 0.15f, 1.07f)).Length();
    const float leftUpperLengthB =
        (leftUpper - Vec3(0.55f, 0.15f, 1.43f)).Length();
    const float leftLowerLengthA =
        (leftLower - Vec3(0.55f, -0.25f, 1.07f)).Length();
    const float leftLowerLengthB =
        (leftLower - Vec3(0.55f, -0.25f, 1.43f)).Length();

    const bool armLengths =
        Near(leftUpperLengthA, (MakeConfig(1.0f).upperOuter - MakeConfig(1.0f).upperInnerA).Length()) &&
        Near(leftUpperLengthB, (MakeConfig(1.0f).upperOuter - MakeConfig(1.0f).upperInnerB).Length()) &&
        Near(leftLowerLengthA, (MakeConfig(1.0f).lowerOuter - MakeConfig(1.0f).lowerInnerA).Length()) &&
        Near(leftLowerLengthB, (MakeConfig(1.0f).lowerOuter - MakeConfig(1.0f).lowerInnerB).Length());

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

    const bool finite =
        Finite(leftHub) &&
        Finite(rightHub) &&
        std::isfinite(left.GetHubOrientation().w) &&
        std::isfinite(left.GetHubOrientation().x) &&
        std::isfinite(left.GetHubOrientation().y) &&
        std::isfinite(left.GetHubOrientation().z) &&
        std::isfinite(right.GetHubOrientation().w) &&
        std::isfinite(right.GetHubOrientation().x) &&
        std::isfinite(right.GetHubOrientation().y) &&
        std::isfinite(right.GetHubOrientation().z);

    std::cout
        << "[DoubleWishbone] armLengths=" << armLengths
        << " symmetry=" << symmetry
        << " hubSymmetry=" << hubSymmetry
        << " finite=" << finite
        << '\n';

    if (!armLengths || !symmetry || !hubSymmetry || !finite) {
        std::cerr << "[FAIL] Double wishbone static geometry diagnostics\n";
        return 1;
    }

    std::cout << "[PASS] Double wishbone static geometry diagnostics\n";
    return 0;
}
