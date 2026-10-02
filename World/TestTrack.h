#pragma once

#include <memory>
#include <vector>

class MeshManager;
class SceneFactory;
class Object;
class Terrain;
class Road;

class TestTrack {
public:
    ~TestTrack();
    bool Initialize(MeshManager& meshManager, SceneFactory& sceneFactory);

    const Terrain& GetTerrain() const;
    std::vector<std::unique_ptr<Object>> TakeObjects();

private:
    std::unique_ptr<Terrain> m_terrain;
    std::unique_ptr<Road> m_road;
    std::vector<std::unique_ptr<Object>> m_objects;
};
