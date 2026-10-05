#pragma once
#include <array>
#include <string>
#include <vector>
#include "../Core/Math/Vec3.h"

struct VehicleConfig {
    float mass = 1190.0f;
    float spawnClearance = 0.78f;
    Vec3 colliderHalfExtents = Vec3(0.90f, 0.34f, 2.15f);
    // Vehicle-local coordinates: front = +Z, right = +X, up = +Y.
    // Left wheels have negative X; right wheels have positive X.
    std::array<Vec3, 4> wheelPositions = {
        Vec3(0.76f, -0.10f, 1.25f), Vec3(-0.76f, -0.10f, 1.25f),
        Vec3(0.76f, -0.10f, -1.25f), Vec3(-0.76f, -0.10f, -1.25f)
    };
    float wheelRadius = 0.32f, wheelInertia = 1.8f;
    float suspensionRestLength = 0.32f, suspensionBumpTravel = 0.16f, suspensionReboundTravel = 0.42f;
    float frontSpringRate = 39000.0f, rearSpringRate = 39000.0f;
    float frontCompressionDamping = 2500.0f, frontReboundDamping = 3000.0f;
    float rearCompressionDamping = 2500.0f, rearReboundDamping = 3000.0f;
    float staticFriction = 1.10f, dynamicFriction = 0.95f;
    float longitudinalStiffness = 36000.0f, lateralStiffness = 42000.0f, rollingResistance = 0.015f;
    float brakeTorque = 2500.0f, maxSteeringAngle = 0.5f;
    float idleRPM = 900.0f, stallRPM = 550.0f, redlineRPM = 7000.0f, peakTorque = 280.0f, engineInertia = 0.25f;
    std::vector<float> gearRatios = {3.50f, 2.10f, 1.50f, 1.10f, 0.85f};
    float reverseRatio = 3.20f, finalDriveRatio = 4.10f;
    bool Load(const std::string& path, std::string& error);
    bool Validate(std::string& error) const;
};