#include "TestTrack.h"

#include "RoadsideScenery.h"
#include "SceneFactory.h"
#include "StageSerializer.h"
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

bool TestTrack::Initialize(
    MeshManager& meshManager,
    SceneFactory& sceneFactory,
    const StageDefinition& stage
) {
    if (m_terrain || m_road)
        return false;

    std::vector<TerrainBump> terrainBumps;
    terrainBumps.reserve(stage.bumps.size());
    for (const StageBump& bump : stage.bumps)
        terrainBumps.push_back({ bump.x, bump.z, bump.radius, bump.height });

    m_terrain = std::make_unique<Terrain>(
        stage.terrainResolution,
        stage.terrainSize,
        stage.terrainHeightScale,
        stage.terrainNoiseScale,
        stage.terrainOctaves,
        stage.terrainSeed,
        stage.elevationProfile,
        terrainBumps
    );
    if (!meshManager.CreateTerrain("terrain", *m_terrain) ||
        !meshManager.LoadTexture("terrain", "Assets/road/dirt.jpg"))
        return false;

    m_road = std::make_unique<Road>();
    if (!m_road->GenerateCourse(*m_terrain, stage.roadControlPoints, stage.roadWidth, stage.roadSurfaceOffset))
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
    const size_t transitionSteps = std::min<size_t>(12, transitionEnd - tarmacEnd);

    const auto addRoadVisual = [&](const std::string& name, size_t first, size_t end, RenderSurface surface, float blend) {
        if (first >= end || !meshManager.CreateRoadSection(name, *m_road, first, end))
            return false;

        bool textureLoaded = false;
        if (surface == RenderSurface::Tarmac) {
            textureLoaded = meshManager.LoadTexture(name, "Assets/road/asphalt.jpg");
        } else if (surface == RenderSurface::Transition) {
            textureLoaded = meshManager.LoadTexture(name, "Assets/road/asphalt.jpg") &&
                meshManager.LoadTexture(name, "Assets/road/dirt.jpg", false, true);
        } else if (surface == RenderSurface::Gravel) {
            textureLoaded = meshManager.LoadTexture(name, "Assets/road/dirt.jpg");
        }
        if (!textureLoaded)
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
