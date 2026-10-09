#pragma once
#include <array>
#include <string>
#include <vector>
#include "../Core/Math/Vec3.h"
#include "VehicleCoordinates.h"

struct VehicleConfig {
    float mass = 1190.0f;
    float spawnClearance = 0.42f;
    Vec3 colliderHalfExtents = Vec3(0.90f, 0.34f, 2.15f);
    // Vehicle-local coordinates: forward = +Z, right = -X, up = +Y.
    // Therefore left wheels use +X and right wheels use -X.
    std::array<Vec3, 4> wheelPositions = {
        VehicleCoordinates::LeftWheelPosition(0.76f, -0.10f, 1.25f),
        VehicleCoordinates::RightWheelPosition(0.76f, -0.10f, 1.25f),
        VehicleCoordinates::LeftWheelPosition(0.76f, -0.10f, -1.25f),
        VehicleCoordinates::RightWheelPosition(0.76f, -0.10f, -1.25f)
    };
    float wheelRadius = 0.32f, wheelInertia = 1.8f;
    float upperArmInnerX = 0.55f, upperArmInnerY = 0.15f, upperArmInnerZ = 0.18f;
    float upperArmOuterX = 0.72f, upperArmOuterY = 0.10f;
    float lowerArmInnerX = 0.55f, lowerArmInnerY = -0.25f, lowerArmInnerZ = 0.18f;
    float lowerArmOuterX = 0.76f, lowerArmOuterY = -0.20f;
    float hubOffsetY = 0.10f;
    // Spring mounts are chassis-local and upright-local respectively.
    float springChassisMountX = 0.55f, springChassisMountY = 0.10f, springChassisMountZ = -0.18f;
    float springUprightMountOffsetX = 0.0f, springUprightMountOffsetY = 0.10f, springUprightMountOffsetZ = 0.0f;
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