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
        {360.0f, 38.0f}, {560.0f, 8.0f}, {760.0f, 0.0f},
        {800.0f, 0.5f}, {830.0f, 2.0f}, {860.0f, 0.1f},
        {900.0f, 1.8f}, {930.0f, 0.2f}, {980.0f, 3.0f},
        {1200.0f, 28.0f}, {1430.0f, 48.0f}, {1650.0f, 10.0f},
        {1710.0f, 6.5f}, {1740.0f, 8.0f}, {1770.0f, 5.5f}, {1810.0f, 2.0f},
        {1870.0f, 0.0f}, {2100.0f, 18.0f}, {2350.0f, 35.0f},
        {2600.0f, 5.0f}, {2660.0f, 3.0f}, {2690.0f, 4.5f}, {2720.0f, 2.0f},
        {2760.0f, 1.0f}, {2850.0f, 0.0f}, {2920.0f, 2.0f},
        {2950.0f, 3.5f}, {2980.0f, 1.0f}, {3020.0f, 4.0f},
        {3050.0f, 6.5f}, {3100.0f, 22.0f}, {3350.0f, 42.0f},
        {3600.0f, 4.0f}, {4000.0f, 0.0f}
    };

    m_terrain = std::make_unique<Terrain>(1025, 10000.0f, 35.0f, 0.0015f, 5, 1337, elevationProfile);
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
