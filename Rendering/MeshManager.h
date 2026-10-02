#pragma once

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
    bool CreateFoliageBillboard(const std::string& name, bool shrub = false);
    bool CreateWheel(const std::string& name);
    bool LoadFromFile(const std::string& name, const std::string& path, bool wheelOnly = false);
    bool CreateTerrain(const std::string& name, const Terrain& terrain);
    bool CreateRoad(const std::string& name, const Road& road);

    Mesh* Get(const std::string& name);
    const Mesh* Get(const std::string& name) const;

    void Clear();

private:
    std::unordered_map<
        std::string,
        std::unique_ptr<Mesh>
    > m_meshes;
};