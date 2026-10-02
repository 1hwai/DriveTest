#include "TestTrack.h"

#include "RoadsideScenery.h"
#include "SceneFactory.h"
#include "../Core/Object.h"
#include "../Physics/Terrain.h"
#include "../Physics/Road.h"
#include "../Rendering/MeshManager.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

TestTrack::TestTrack() = default;
TestTrack::~TestTrack() = default;

bool TestTrack::Initialize(MeshManager& meshManager, SceneFactory& sceneFactory) {
    if (m_terrain || m_road)
        return false;

    const std::vector<std::pair<float, float>> elevationProfile = {
        {-500.0f, 0.0f}, {-300.0f, 0.0f}, {0.0f, 0.0f}, {180.0f, 12.0f},
        {360.0f, 38.0f}, {560.0f, 8.0f}, {760.0f, 0.0f}, {980.0f, 3.0f},
        {1200.0f, 28.0f}, {1430.0f, 48.0f}, {1650.0f, 10.0f},
        {1870.0f, 0.0f}, {2100.0f, 18.0f}, {2350.0f, 35.0f},
        {2600.0f, 5.0f}, {2850.0f, 0.0f}, {3100.0f, 22.0f},
        {3350.0f, 42.0f}, {3600.0f, 4.0f}, {4000.0f, 0.0f}
    };

    const std::vector<TerrainBump> bumps = {
        {52.0f, 835.0f, 26.0f, 1.8f},
        {74.0f, 905.0f, 22.0f, -0.8f},
        {-108.0f, 1735.0f, 24.0f, 1.4f},
        {-18.0f, 2690.0f, 25.0f, 1.6f},
        {22.0f, 2970.0f, 21.0f, -0.7f},
        {42.0f, 3030.0f, 24.0f, 1.2f}
    };

    m_terrain = std::make_unique<Terrain>(1025, 10000.0f, 35.0f, 0.0015f, 5, 1337, elevationProfile, bumps);
    if (!meshManager.CreateTerrain("terrain", *m_terrain))
        return false;

    m_road = std::make_unique<Road>();
    if (!m_road->GenerateTestCourse(*m_terrain, 12.0f, 0.25f))
        return false;

    auto ground = sceneFactory.CreateTerrain({"Terrain", 0.0f, 0.5f}, *m_terrain);
    if (!ground)
        return false;
    ground->SetScenePersistent(false);
    ground->SetColor(Vec3(1.0f, 1.0f, 1.0f));
    ground->SetRenderSurface(RenderSurface::Terrain);
    m_objects.push_back(std::move(ground));

    auto roadCollider = sceneFactory.CreateRoad({"TestRoadCollision", 0.0f, 0.85f}, *m_road);
    if (!roadCollider)
        return false;
    roadCollider->SetScenePersistent(false);
    roadCollider->SetMesh(nullptr);
    m_objects.push_back(std::move(roadCollider));

    const size_t segmentCount = m_road->GetSegmentCount();
    const size_t tarmacEnd = std::max<size_t>(1, segmentCount * 18 / 100);
    const size_t transitionEnd = std::max(tarmacEnd + 1, segmentCount * 24 / 100);
    const size_t transitionSteps = 12;

    const auto addRoadVisual = [&](const std::string& name, size_t first, size_t end, RenderSurface surface, float blend) {
        if (first >= end || !meshManager.CreateRoadSection(name, *m_road, first, end))
            return false;
        auto object = std::make_unique<Object>();
        object->SetName(name);
        object->SetScenePersistent(false);
        object->SetMesh(meshManager.Get(name));
        object->SetColor(Vec3(1.0f, 1.0f, 1.0f));
        object->SetRenderSurface(surface);
        object->SetSurfaceBlend(blend);
        m_objects.push_back(std::move(object));
        return true;
    };

    if (!addRoadVisual("RoadTarmac", 0, tarmacEnd, RenderSurface::Tarmac, 0.0f))
        return false;

    for (size_t i = 0; i < transitionSteps; ++i) {
        const size_t first = tarmacEnd + (transitionEnd - tarmacEnd) * i / transitionSteps;
        const size_t end = tarmacEnd + (transitionEnd - tarmacEnd) * (i + 1) / transitionSteps;
        if (!addRoadVisual("RoadTransition" + std::to_string(i), first, end, RenderSurface::Transition,
            static_cast<float>(i + 1) / static_cast<float>(transitionSteps)))
            return false;
    }

    if (!addRoadVisual("RoadGravel", transitionEnd, segmentCount, RenderSurface::Gravel, 1.0f))
        return false;

    auto scenery = RoadsideScenery::Create(meshManager, *m_terrain, *m_road);
    if (scenery.empty())
        return false;
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
