#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

class Mesh;
class Terrain;
class Road;

class MeshManager {
public:
    MeshManager() = default;
    ~MeshManager();

    bool CreateCube(const std::string& name);
    bool CreateTriangle(const std::string& name);
    bool CreateSphere(const std::string& name);
    bool CreateCylinder(const std::string& name);
    bool CreateDisc(const std::string& name);
    bool CreateFoliageBillboard(const std::string& name, bool shrub = false);
    bool CreateWheel(const std::string& name);
    bool LoadFromFile(const std::string& name, const std::string& path, bool wheelOnly = false);
    bool LoadTexture(const std::string& name, const std::string& path, bool removeBackground = false, bool secondary = false);
    bool CreateTerrain(const std::string& name, const Terrain& terrain);
    bool CreateTerrainChunk(const std::string& name, const Terrain& terrain, int firstCellX, int firstCellZ, int cellWidth, int cellHeight);
    bool CreateRoad(const std::string& name, const Road& road);
    bool CreateRoadSection(const std::string& name, const Road& road, std::size_t firstSegment, std::size_t endSegment);
    bool CreateRoadsideWall(const std::string& name, const Terrain& terrain, const Road& road, bool leftSide, std::size_t firstSegment, std::size_t endSegment, float offset, float wallHeight);

    Mesh* Get(const std::string& name);
    const Mesh* Get(const std::string& name) const;

    void Clear();

private:
    std::unordered_map<
        std::string,
        std::unique_ptr<Mesh>
    > m_meshes;
};