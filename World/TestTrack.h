#pragma once

#include <memory>
#include <vector>

class MeshManager;
class SceneFactory;
class Object;
class Terrain;

class TestTrack {
public:
    bool Initialize(MeshManager& meshManager, SceneFactory& sceneFactory);

    const Terrain& GetTerrain() const;
    std::vector<std::unique_ptr<Object>> TakeObjects();

private:
    std::unique_ptr<Terrain> m_terrain;
    std::unique_ptr<class Road> m_road;
    std::vector<std::unique_ptr<Object>> m_objects;
};
