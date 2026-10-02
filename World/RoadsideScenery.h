#pragma once

#include <memory>
#include <vector>

class MeshManager;
class Terrain;
class Road;
class Object;

class RoadsideScenery {
public:
    static std::vector<std::unique_ptr<Object>> Create(
        MeshManager& meshManager,
        const Terrain& terrain,
        const Road& road
    );
};
