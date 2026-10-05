#pragma once

#include <memory>
#include <string>
#include <vector>

class MeshManager;
class SceneFactory;
class Object;
class Terrain;
class Road;
struct StageDefinition;

class TestTrack {
public:
    TestTrack();
    ~TestTrack();
    bool Initialize(MeshManager& meshManager, SceneFactory& sceneFactory, const StageDefinition& stage);

    const Terrain& GetTerrain() const;
    std::vector<std::unique_ptr<Object>> TakeObjects();

private:
    std::unique_ptr<Terrain> m_terrain;
    std::unique_ptr<Road> m_road;
    std::vector<std::unique_ptr<Object>> m_objects;
};
