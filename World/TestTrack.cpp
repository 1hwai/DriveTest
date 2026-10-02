#include "TestTrack.h"

#include "RoadsideScenery.h"
#include "SceneFactory.h"
#include "../Core/Object.h"
#include "../Physics/Terrain.h"
#include "../Physics/Road.h"
#include "../Rendering/MeshManager.h"

#include <memory>
#include <utility>

bool TestTrack::Initialize(MeshManager& meshManager, SceneFactory& sceneFactory) {
    if (m_terrain || m_road)
        return false;

    m_terrain = std::make_unique<Terrain>(1025, 10000.0f, 35.0f, 0.0015f, 5, 1337);
    if (!meshManager.CreateTerrain("terrain", *m_terrain))
        return false;

    m_road = std::make_unique<Road>();
    if (!m_road->GenerateTestCourse(*m_terrain, 12.0f, 0.25f) ||
        !meshManager.CreateRoad("road", *m_road))
        return false;

    auto ground = sceneFactory.CreateTerrain({"Terrain", 0.0f, 0.5f}, *m_terrain);
    if (!ground)
        return false;
    ground->SetScenePersistent(false);
    ground->SetColor(Vec3(0.24f, 0.48f, 0.19f));
    m_objects.push_back(std::move(ground));

    auto road = sceneFactory.CreateRoad({"TestRoad", 0.0f, 0.85f}, *m_road);
    if (!road)
        return false;
    road->SetScenePersistent(false);
    road->SetColor(Vec3(0.16f, 0.17f, 0.18f));
    m_objects.push_back(std::move(road));

    auto scenery = RoadsideScenery::Create(meshManager, *m_terrain, *m_road);
    for (auto& object : scenery)
        m_objects.push_back(std::move(object));

    return true;
}

const Terrain& TestTrack::GetTerrain() const {
    return *m_terrain;
}

std::vector<std::unique_ptr<Object>> TestTrack::TakeObjects() {
    return std::move(m_objects);
}
