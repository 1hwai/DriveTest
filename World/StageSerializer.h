#pragma once

#include <string>
#include <utility>
#include <vector>

#include "../Core/Math/Vec3.h"
#include "../Physics/Terrain.h"

struct StageDefinition {
    std::string name = "Untitled";
    int terrainResolution = 65;
    float terrainSize = 100.0f;
    float terrainHeightScale = 4.0f;
    float terrainNoiseScale = 0.035f;
    int terrainOctaves = 5;
    unsigned int terrainSeed = 1337;
    std::vector<std::pair<float, float>> elevationProfile;
    std::vector<TerrainBump> bumps;
    float roadWidth = 8.0f;
    float roadSurfaceOffset = 0.25f;
    std::vector<Vec3> roadControlPoints;
};

class StageSerializer {
public:
    static bool Load(const std::string& path, StageDefinition& stage, std::string& error);
    static bool Save(const std::string& path, const StageDefinition& stage, std::string& error);
};
