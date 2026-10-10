#include "VehicleConfig.h"
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <sstream>
#include <unordered_map>

namespace {
std::string Trim(const std::string& value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}
bool ParseFloat(const std::string& text, float& value) {
    char* end = nullptr;
    value = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || !std::isfinite(value)) return false;
    while (*end && std::isspace(static_cast<unsigned char>(*end))) ++end;
    return *end == '\0';
}
}

bool VehicleConfig::Load(const std::string& path, std::string& error) {
    std::ifstream file(path);
    if (!file) { error = "Unable to open vehicle config: " + path; return false; }

    std::unordered_map<std::string, float> values;
    std::vector<float> gears;
    std::string suspensionLayoutValue;
    bool suspensionLayoutSeen = false;
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        line = Trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#') continue;
        if (line.front() == '[' && line.back() == ']') continue;
        const size_t separator = line.find('=');
        if (separator == std::string::npos) {
            error = "Invalid config line " + std::to_string(lineNumber);
            return false;
        }
        const std::string key = Trim(line.substr(0, separator));
        const std::string value = Trim(line.substr(separator + 1));
        if (key.empty() || value.empty()) {
            error = "Empty key or value on line " + std::to_string(lineNumber);
            return false;
        }
        if (key == "SuspensionLayout") {
            if (suspensionLayoutSeen) {
                error = "Duplicate config key: " + key;
                return false;
            }
            suspensionLayoutSeen = true;
            suspensionLayoutValue = value;
            continue;
        }
        if (key.rfind("SuspensionLayout", 0) == 0) {
            error = "Invalid suspension layout key on line " + std::to_string(lineNumber);
            return false;
        }
        if (key == "GearRatios") {
            std::stringstream stream(value);
            std::string token;
            gears.clear();
            while (std::getline(stream, token, ',')) {
                float ratio = 0.0f;
                if (!ParseFloat(Trim(token), ratio)) {
                    error = "Invalid GearRatios on line " + std::to_string(lineNumber);
                    return false;
                }
                gears.push_back(ratio);
            }
            continue;
        }
        float number = 0.0f;
        if (!ParseFloat(value, number)) {
            error = "Invalid number for " + key + " on line " + std::to_string(lineNumber);
            return false;
        }
        if (!values.emplace(key, number).second) {
            error = "Duplicate config key: " + key;
            return false;
        }
    }

#define READ(key, target) do { const auto it = values.find(key); if (it != values.end()) target = it->second; } while (false)
    READ("mass", mass);
    READ("spawnClearance", spawnClearance);
    READ("ColliderHalfExtentsX", colliderHalfExtents.x);
    READ("ColliderHalfExtentsY", colliderHalfExtents.y);
    READ("ColliderHalfExtentsZ", colliderHalfExtents.z);
    READ("wheelRadius", wheelRadius);
    READ("wheelInertia", wheelInertia);
    READ("upperArmInnerX", upperArmInnerX);
    READ("upperArmInnerY", upperArmInnerY);
    READ("upperArmInnerZ", upperArmInnerZ);
    READ("upperArmOuterX", upperArmOuterX);
    READ("upperArmOuterY", upperArmOuterY);
    READ("lowerArmInnerX", lowerArmInnerX);
    READ("lowerArmInnerY", lowerArmInnerY);
    READ("lowerArmInnerZ", lowerArmInnerZ);
    READ("lowerArmOuterX", lowerArmOuterX);
    READ("lowerArmOuterY", lowerArmOuterY);
    READ("hubOffsetY", hubOffsetY);
    READ("macPhersonLowerArmInnerX", macPhersonLowerArmInnerX);
    READ("macPhersonLowerArmInnerY", macPhersonLowerArmInnerY);
    READ("macPhersonLowerArmInnerZ", macPhersonLowerArmInnerZ);
    READ("macPhersonLowerArmOuterX", macPhersonLowerArmOuterX);
    READ("macPhersonLowerArmOuterY", macPhersonLowerArmOuterY);
    READ("macPhersonStrutUpperMountX", macPhersonStrutUpperMountX);
    READ("macPhersonStrutUpperMountY", macPhersonStrutUpperMountY);
    READ("macPhersonStrutLowerMountOffsetY", macPhersonStrutLowerMountOffsetY);
    READ("macPhersonStrutLength", macPhersonStrutLength);
    READ("springChassisMountX", springChassisMountX);
    READ("springChassisMountY", springChassisMountY);
    READ("springChassisMountZ", springChassisMountZ);
    READ("springUprightMountOffsetX", springUprightMountOffsetX);
    READ("springUprightMountOffsetY", springUprightMountOffsetY);
    READ("springUprightMountOffsetZ", springUprightMountOffsetZ);
    READ("suspensionRestLength", suspensionRestLength);
    READ("suspensionBumpTravel", suspensionBumpTravel);
    READ("suspensionReboundTravel", suspensionReboundTravel);
    READ("frontSpringRate", frontSpringRate);
    READ("rearSpringRate", rearSpringRate);
    READ("frontCompressionDamping", frontCompressionDamping);
    READ("frontReboundDamping", frontReboundDamping);
    READ("rearCompressionDamping", rearCompressionDamping);
    READ("rearReboundDamping", rearReboundDamping);
    READ("staticFriction", staticFriction);
    READ("dynamicFriction", dynamicFriction);
    READ("longitudinalStiffness", longitudinalStiffness);
    READ("lateralStiffness", lateralStiffness);
    READ("rollingResistance", rollingResistance);
    READ("brakeTorque", brakeTorque);
    READ("maxSteeringAngle", maxSteeringAngle);
    READ("idleRPM", idleRPM);
    READ("stallRPM", stallRPM);
    READ("redlineRPM", redlineRPM);
    READ("peakTorque", peakTorque);
    READ("engineInertia", engineInertia);
    READ("reverseRatio", reverseRatio);
    READ("finalDriveRatio", finalDriveRatio);
    READ("FrontLeftX", wheelPositions[0].x); READ("FrontLeftY", wheelPositions[0].y); READ("FrontLeftZ", wheelPositions[0].z);
    READ("FrontRightX", wheelPositions[1].x); READ("FrontRightY", wheelPositions[1].y); READ("FrontRightZ", wheelPositions[1].z);
    READ("RearLeftX", wheelPositions[2].x); READ("RearLeftY", wheelPositions[2].y); READ("RearLeftZ", wheelPositions[2].z);
    READ("RearRightX", wheelPositions[3].x); READ("RearRightY", wheelPositions[3].y); READ("RearRightZ", wheelPositions[3].z);
#undef READ

    if (suspensionLayoutSeen) {
        if (suspensionLayoutValue == "MacPherson") {
            suspensionLayout = SuspensionLayout::MacPherson;
        } else if (suspensionLayoutValue == "DoubleWishbone") {
            suspensionLayout = SuspensionLayout::DoubleWishbone;
        } else {
            error = "Invalid SuspensionLayout: " + suspensionLayoutValue;
            return false;
        }
    }

    if (!gears.empty()) gearRatios = gears;
    const char* known[] = {
        "mass","spawnClearance","ColliderHalfExtentsX","ColliderHalfExtentsY","ColliderHalfExtentsZ",
        "wheelRadius","wheelInertia","upperArmInnerX","upperArmInnerY","upperArmInnerZ","upperArmOuterX","upperArmOuterY",
        "lowerArmInnerX","lowerArmInnerY","lowerArmInnerZ","lowerArmOuterX","lowerArmOuterY","hubOffsetY",
        "macPhersonLowerArmInnerX","macPhersonLowerArmInnerY","macPhersonLowerArmInnerZ","macPhersonLowerArmOuterX","macPhersonLowerArmOuterY",
        "macPhersonStrutUpperMountX","macPhersonStrutUpperMountY","macPhersonStrutLowerMountOffsetY","macPhersonStrutLength",
        "springChassisMountX","springChassisMountY","springChassisMountZ",
        "springUprightMountOffsetX","springUprightMountOffsetY","springUprightMountOffsetZ",
        "suspensionRestLength","suspensionBumpTravel","suspensionReboundTravel",
        "frontSpringRate","rearSpringRate","frontCompressionDamping","frontReboundDamping","rearCompressionDamping","rearReboundDamping",
        "staticFriction","dynamicFriction","longitudinalStiffness","lateralStiffness","rollingResistance","brakeTorque","maxSteeringAngle",
        "idleRPM","stallRPM","redlineRPM","peakTorque","engineInertia","reverseRatio","finalDriveRatio",
        "FrontLeftX","FrontLeftY","FrontLeftZ","FrontRightX","FrontRightY","FrontRightZ",
        "RearLeftX","RearLeftY","RearLeftZ","RearRightX","RearRightY","RearRightZ"
    };
    for (const auto& entry : values) {
        bool found = false;
        for (const char* key : known) if (entry.first == key) { found = true; break; }
        if (!found) { error = "Unknown vehicle config key: " + entry.first; return false; }
    }
    return Validate(error);
}

bool VehicleConfig::Validate(std::string& error) const {
    if (suspensionLayout != SuspensionLayout::MacPherson &&
        suspensionLayout != SuspensionLayout::DoubleWishbone) {
        error = "Invalid suspension layout";
        return false;
    }
    if (mass <= 0.0f || spawnClearance <= 0.0f ||
        colliderHalfExtents.x <= 0.0f || colliderHalfExtents.y <= 0.0f || colliderHalfExtents.z <= 0.0f ||
        wheelRadius <= 0.0f || wheelInertia <= 0.0f) {
        error = "Mass, height, collider dimensions and wheel values must be positive";
        return false;
    }
    if (suspensionRestLength <= 0.0f || suspensionBumpTravel < 0.0f || suspensionReboundTravel < 0.0f ||
        frontSpringRate < 0.0f || rearSpringRate < 0.0f ||
        frontCompressionDamping < 0.0f || frontReboundDamping < 0.0f ||
        rearCompressionDamping < 0.0f || rearReboundDamping < 0.0f) {
        error = "Invalid suspension lengths, spring rates or damping";
        return false;
    }
    if (macPhersonLowerArmInnerX <= 0.0f ||
        macPhersonLowerArmInnerZ <= 0.0f ||
        macPhersonLowerArmOuterX <= 0.0f ||
        macPhersonStrutLength <= 0.0f) {
        error = "Invalid MacPherson geometry dimensions";
        return false;
    }
    if (staticFriction < 0.0f || dynamicFriction < 0.0f ||
        longitudinalStiffness < 0.0f || lateralStiffness < 0.0f || rollingResistance < 0.0f ||
        brakeTorque < 0.0f || maxSteeringAngle < 0.0f ||
        idleRPM <= stallRPM || redlineRPM <= idleRPM || peakTorque < 0.0f || engineInertia <= 0.0f ||
        reverseRatio <= 0.0f || finalDriveRatio <= 0.0f || gearRatios.size() < 4) {
        error = "Invalid tire, control, engine or transmission values";
        return false;
    }
    for (float ratio : gearRatios) if (ratio <= 0.0f) {
        error = "Forward gear ratios must be positive";
        return false;
    }
    return true;
}