#include "StageSerializer.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <utility>

namespace {
    constexpr const char* StageHeader = "DRIVETEST_STAGE 1";
    constexpr size_t MaxProfilePoints = 10000;
    constexpr size_t MaxBumps = 100000;
    constexpr size_t MaxRoadPoints = 10000;

    std::filesystem::path ResolveStagePath(const std::string& path) {
        std::filesystem::path filePath(path);
#ifdef DRIVETEST_PROJECT_ROOT
        if (filePath.is_relative())
            filePath = std::filesystem::path(DRIVETEST_PROJECT_ROOT) / filePath;
#endif
        return filePath;
    }

    bool IsFinite(float value) {
        return std::isfinite(value);
    }

    bool Validate(const StageDefinition& stage, std::string& error) {
        if (stage.name.empty()) {
            error = "Stage name cannot be empty.";
            return false;
        }
        if (stage.terrainResolution < 2 || stage.terrainSize <= 0.0f ||
            stage.terrainHeightScale < 0.0f || stage.terrainNoiseScale <= 0.0f ||
            stage.terrainOctaves < 1 || stage.roadWidth <= 0.0f ||
            !IsFinite(stage.terrainSize) || !IsFinite(stage.terrainHeightScale) ||
            !IsFinite(stage.terrainNoiseScale) || !IsFinite(stage.roadWidth) ||
            !IsFinite(stage.roadSurfaceOffset)) {
            error = "Stage contains invalid terrain or road settings.";
            return false;
        }
        if (stage.elevationProfile.size() > MaxProfilePoints ||
            stage.bumps.size() > MaxBumps ||
            stage.roadControlPoints.size() < 2 ||
            stage.roadControlPoints.size() > MaxRoadPoints) {
            error = "Stage contains an invalid number of profile points, bumps, or road points.";
            return false;
        }
        for (const auto& point : stage.elevationProfile) {
            if (!IsFinite(point.first) || !IsFinite(point.second)) {
                error = "Elevation profile contains a non-finite value.";
                return false;
            }
        }
        for (const TerrainBump& bump : stage.bumps) {
            if (!IsFinite(bump.x) || !IsFinite(bump.z) ||
                !IsFinite(bump.radius) || !IsFinite(bump.height) || bump.radius < 0.0f) {
                error = "Terrain bump contains an invalid value.";
                return false;
            }
        }
        for (const Vec3& point : stage.roadControlPoints) {
            if (!IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z)) {
                error = "Road control point contains a non-finite value.";
                return false;
            }
        }
        return true;
    }

    bool ReadCount(std::istream& file, const char* expected, size_t maximum, size_t& count) {
        std::string token;
        if (!(file >> token >> count) || token != expected || count > maximum)
            return false;
        return true;
    }
}

bool StageSerializer::Load(const std::string& path, StageDefinition& stage, std::string& error) {
    const std::filesystem::path filePath = ResolveStagePath(path);
    std::ifstream file(filePath);
    if (!file) {
        error = "Could not open stage file: " + filePath.string();
        return false;
    }

    std::string header;
    std::getline(file, header);
    if (header != StageHeader) {
        error = "Unsupported stage file header or version: " + filePath.string();
        return false;
    }

    StageDefinition candidate;
    std::string token;
    if (!(file >> token) || token != "NAME" || !(file >> std::quoted(candidate.name))) {
        error = "Expected NAME in stage file.";
        return false;
    }
    if (!(file >> token) || token != "TERRAIN" ||
        !(file >> candidate.terrainResolution >> candidate.terrainSize >>
            candidate.terrainHeightScale >> candidate.terrainNoiseScale >>
            candidate.terrainOctaves >> candidate.terrainSeed)) {
        error = "Expected valid TERRAIN settings in stage file.";
        return false;
    }
    if (!(file >> token) || token != "ROAD" ||
        !(file >> candidate.roadWidth >> candidate.roadSurfaceOffset)) {
        error = "Expected valid ROAD settings in stage file.";
        return false;
    }

    size_t count = 0;
    if (!ReadCount(file, "ELEVATION_PROFILE", MaxProfilePoints, count)) {
        error = "Expected ELEVATION_PROFILE count in stage file.";
        return false;
    }
    candidate.elevationProfile.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        float z = 0.0f, height = 0.0f;
        if (!(file >> z >> height)) {
            error = "Invalid elevation profile entry.";
            return false;
        }
        candidate.elevationProfile.emplace_back(z, height);
    }

    if (!ReadCount(file, "BUMPS", MaxBumps, count)) {
        error = "Expected BUMPS count in stage file.";
        return false;
    }
    candidate.bumps.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        TerrainBump bump{};
        if (!(file >> bump.x >> bump.z >> bump.radius >> bump.height)) {
            error = "Invalid terrain bump entry.";
            return false;
        }
        candidate.bumps.push_back(bump);
    }

    if (!ReadCount(file, "ROAD_POINTS", MaxRoadPoints, count)) {
        error = "Expected ROAD_POINTS count in stage file.";
        return false;
    }
    candidate.roadControlPoints.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        Vec3 point;
        if (!(file >> point.x >> point.y >> point.z)) {
            error = "Invalid road control point.";
            return false;
        }
        candidate.roadControlPoints.push_back(point);
    }
    if (!(file >> token) || token != "END_STAGE") {
        error = "Expected END_STAGE in stage file.";
        return false;
    }
    if (!Validate(candidate, error))
        return false;

    stage = std::move(candidate);
    error.clear();
    return true;
}

bool StageSerializer::Save(const std::string& path, const StageDefinition& stage, std::string& error) {
    if (!Validate(stage, error))
        return false;

    const std::filesystem::path filePath = ResolveStagePath(path);
    if (filePath.has_parent_path()) {
        std::error_code filesystemError;
        std::filesystem::create_directories(filePath.parent_path(), filesystemError);
        if (filesystemError) {
            error = "Could not create stage directory: " + filesystemError.message();
            return false;
        }
    }
    std::ofstream file(filePath);
    if (!file) {
        error = "Could not write stage file: " + filePath.string();
        return false;
    }

    file << std::setprecision(9);
    file << StageHeader << '\n';
    file << "NAME " << std::quoted(stage.name) << '\n';
    file << "TERRAIN " << stage.terrainResolution << ' ' << stage.terrainSize << ' '
        << stage.terrainHeightScale << ' ' << stage.terrainNoiseScale << ' '
        << stage.terrainOctaves << ' ' << stage.terrainSeed << '\n';
    file << "ROAD " << stage.roadWidth << ' ' << stage.roadSurfaceOffset << '\n';
    file << "ELEVATION_PROFILE " << stage.elevationProfile.size() << '\n';
    for (const auto& point : stage.elevationProfile)
        file << point.first << ' ' << point.second << '\n';
    file << "BUMPS " << stage.bumps.size() << '\n';
    for (const TerrainBump& bump : stage.bumps)
        file << bump.x << ' ' << bump.z << ' ' << bump.radius << ' ' << bump.height << '\n';
    file << "ROAD_POINTS " << stage.roadControlPoints.size() << '\n';
    for (const Vec3& point : stage.roadControlPoints)
        file << point.x << ' ' << point.y << ' ' << point.z << '\n';
    file << "END_STAGE\n";
    file.flush();
    if (!file) {
        error = "Failed while writing stage file: " + filePath.string();
        return false;
    }
    error.clear();
    return true;
}
