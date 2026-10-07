#include <cmath>
#include <iostream>

#include "../Vehicle/Suspension.h"

namespace {
bool Near(float actual, float expected, float tolerance = 0.0001f) {
    return std::abs(actual - expected) <= tolerance;
}

bool NearVec(const Vec3& a, const Vec3& b, float tolerance = 0.0001f) {
    return Near(a.x, b.x, tolerance) &&
        Near(a.y, b.y, tolerance) &&
        Near(a.z, b.z, tolerance);
}

bool Finite(const Vec3& value) {
    return std::isfinite(value.x) &&
        std::isfinite(value.y) &&
        std::isfinite(value.z);
}

bool TestMechanicalLimits() {
    Suspension suspension;
    suspension.SetRestLength(0.8f);
    suspension.SetBumpTravel(0.2f);
    suspension.SetReboundTravel(0.3f);

    const bool passed =
        Near(suspension.ClampLength(0.4f), 0.6f) &&
        Near(suspension.ClampLength(0.7f), 0.7f) &&
        Near(suspension.ClampLength(1.3f), 1.1f);

    std::cout << "[SuspensionLimits] "
              << "min=" << suspension.ClampLength(0.4f)
              << " rest=" << suspension.ClampLength(0.8f)
              << " max=" << suspension.ClampLength(1.3f)
              << "\n";
    return passed;
}

bool TestCompressionState() {
    Suspension suspension;
    suspension.SetRestLength(0.8f);
    suspension.SetBumpTravel(0.2f);
    suspension.SetReboundTravel(0.3f);

    suspension.UpdateLength(0.7f, 0.1f);

    const bool passed =
        Near(suspension.GetLength(), 0.7f) &&
        Near(suspension.GetCompression(), 0.1f) &&
        Near(suspension.GetCompressionVelocity(), 1.0f);

    std::cout << "[SuspensionCompression] "
              << "length=" << suspension.GetLength()
              << " compression=" << suspension.GetCompression()
              << " velocity=" << suspension.GetCompressionVelocity()
              << "\n";
    return passed;
}

bool TestDampingDirection() {
    Suspension suspension;
    suspension.SetRestLength(1.0f);
    suspension.SetBumpTravel(0.5f);
    suspension.SetReboundTravel(0.5f);
    suspension.SetSpringRate(1000.0f);
    suspension.SetCompressionDamperRate(100.0f);
    suspension.SetReboundDamperRate(300.0f);

    suspension.UpdateLength(0.8f, 0.1f);
    const float compressionForce = suspension.CalculateForce();

    suspension.UpdateLength(0.9f, 0.1f);
    const float reboundForce = suspension.CalculateForce();

    const float expectedCompressionForce = 1000.0f * 0.2f + 100.0f * 2.0f;
    const float expectedReboundForce = 0.0f;

    std::cout << "[SuspensionDamping] "
              << "compressionForce=" << compressionForce
              << " reboundForce=" << reboundForce
              << "\n";

    return Near(compressionForce, expectedCompressionForce) &&
        Near(reboundForce, expectedReboundForce);
}

bool TestReboundForceCannotBecomeTensile() {
    Suspension suspension;
    suspension.SetRestLength(1.0f);
    suspension.SetSpringRate(1000.0f);
    suspension.SetReboundDamperRate(3000.0f);

    suspension.UpdateLength(0.9f, 0.01f);
    suspension.UpdateLength(0.99f, 0.01f);

    const float force = suspension.CalculateForce();

    std::cout << "[SuspensionNonTensile] force=" << force << "\n";
    return Near(force, 0.0f);
}

bool TestMountLengthAndForceDirection() {
    Suspension suspension;
    suspension.SetRestLength(1.0f);
    suspension.SetBumpTravel(0.5f);
    suspension.SetReboundTravel(0.5f);
    suspension.SetSpringRate(1000.0f);
    suspension.SetCompressionDamperRate(0.0f);
    suspension.SetReboundDamperRate(0.0f);

    const Vec3 mountA(1.0f, 2.0f, 3.0f);
    const Vec3 mountB(1.3f, 2.4f, 3.0f);
    const Vec3 delta = mountB - mountA;

    suspension.UpdateFromMounts(mountA, mountB, 0.1f);
    const Vec3 force = suspension.CalculateForceVector(mountA, mountB);
    const Vec3 expectedDirection = delta.Normalized();
    const float expectedForce = 1000.0f * (1.0f - delta.Length());

    const bool passed =
        Near(suspension.GetLength(), delta.Length()) &&
        Near(suspension.GetCompression(), 1.0f - delta.Length()) &&
        Near(force.Length(), expectedForce) &&
        NearVec(force.Normalized(), expectedDirection);

    std::cout << "[SuspensionMounts] "
              << "length=" << suspension.GetLength()
              << " compression=" << suspension.GetCompression()
              << " force=" << force.Length()
              << " directionMatches=" << NearVec(force.Normalized(), expectedDirection)
              << " finite=" << Finite(force)
              << "\n";
    return passed && Finite(force);
}

bool TestZeroLengthMounts() {
    Suspension suspension;
    const Vec3 mount(0.0f, 0.0f, 0.0f);

    suspension.UpdateFromMounts(mount, mount, 0.016f);
    const Vec3 force = suspension.CalculateForceVector(mount, mount);

    const bool passed =
        Near(suspension.GetLength(), 0.0f) &&
        Near(force.Length(), 0.0f) &&
        Finite(force);

    std::cout << "[SuspensionDegenerateMounts] "
              << "length=" << suspension.GetLength()
              << " force=" << force.Length()
              << "\n";
    return passed;
}
}

int main() {
    bool passed = true;

    passed = TestMechanicalLimits() && passed;
    passed = TestCompressionState() && passed;
    passed = TestDampingDirection() && passed;
    passed = TestReboundForceCannotBecomeTensile() && passed;
    passed = TestMountLengthAndForceDirection() && passed;
    passed = TestZeroLengthMounts() && passed;

    if (passed)
        std::cout << "[PASS] Suspension diagnostics\n";
    else
        std::cerr << "[FAIL] Suspension diagnostics\n";

    return passed ? 0 : 1;
}
