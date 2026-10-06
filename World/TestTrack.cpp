#include "TestTrack.h"

#include "RoadsideScenery.h"
#include "SceneFactory.h"
#include "StageSerializer.h"
#include "../Core/Object.h"
#include "../Physics/Terrain.h"
#include "../Physics/Road.h"
#include "../Rendering/Mesh.h"
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
    constexpr int TerrainChunkCells = 8;
    Mesh* asphaltTextureSource = nullptr;
    Mesh* dirtTextureSource = nullptr;

    for (int firstZ = 0;
        firstZ < stage.terrainResolution - 1;
        firstZ += TerrainChunkCells) {
        for (int firstX = 0;
            firstX < stage.terrainResolution - 1;
            firstX += TerrainChunkCells) {
            const std::string name =
                "TerrainChunk_" +
                std::to_string(firstX) + "_" +
                std::to_string(firstZ);

            if (!meshManager.CreateTerrainChunk(
                name,
                *m_terrain,
                firstX,
                firstZ,
                TerrainChunkCells,
                TerrainChunkCells
            ))
                return false;

            Mesh* terrainMesh = meshManager.Get(name);
            if (!terrainMesh)
                return false;

            if (!dirtTextureSource) {
                if (!terrainMesh->LoadTexture("Assets/road/dirt.jpg"))
                    return false;
                dirtTextureSource = terrainMesh;
            } else if (!terrainMesh->ShareTextureFrom(*dirtTextureSource)) {
                return false;
            }

            auto object = std::make_unique<Object>();
            object->SetName(name);
            object->SetScenePersistent(false);
            object->SetMesh(meshManager.Get(name));
            object->SetColor(Vec3(1.0f, 1.0f, 1.0f));
            object->SetRenderSurface(RenderSurface::Terrain);
            m_objects.push_back(std::move(object));
        }
    }

    auto terrainCollider =
        sceneFactory.CreateTerrainCollider(
            {"TestTerrainCollision", 0.0f, 0.85f},
            *m_terrain
        );
    if (!terrainCollider)
        return false;
    terrainCollider->SetScenePersistent(false);
    terrainCollider->SetMesh(nullptr);
    m_objects.push_back(std::move(terrainCollider));

    m_road = std::make_unique<Road>();
    if (!m_road->GenerateCourse(*m_terrain, stage.roadControlPoints, stage.roadWidth, stage.roadSurfaceOffset))
        return false;

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

    constexpr size_t RoadChunkSegments = 32;

    const auto addRoadVisual = [&](
        const std::string& prefix,
        size_t first,
        size_t end,
        RenderSurface surface,
        float blend
    ) {
        for (size_t chunkFirst = first;
            chunkFirst < end;
            chunkFirst += RoadChunkSegments) {
            const size_t chunkEnd =
                std::min(chunkFirst + RoadChunkSegments, end);

            const std::string name =
                prefix + "_" + std::to_string(chunkFirst);

            if (!meshManager.CreateRoadSection(
                name,
                *m_road,
                chunkFirst,
                chunkEnd
            ))
                return false;

            Mesh* roadMesh = meshManager.Get(name);
            if (!roadMesh)
                return false;

            if (surface == RenderSurface::Tarmac) {
                if (!asphaltTextureSource) {
                    if (!roadMesh->LoadTexture("Assets/road/asphalt.jpg"))
                        return false;
                    asphaltTextureSource = roadMesh;
                } else if (!roadMesh->ShareTextureFrom(*asphaltTextureSource)) {
                    return false;
                }
            } else if (surface == RenderSurface::Transition) {
                if (!asphaltTextureSource) {
                    if (!roadMesh->LoadTexture("Assets/road/asphalt.jpg"))
                        return false;
                    asphaltTextureSource = roadMesh;
                } else if (!roadMesh->ShareTextureFrom(*asphaltTextureSource)) {
                    return false;
                }

                if (!dirtTextureSource) {
                    if (!roadMesh->LoadTexture("Assets/road/dirt.jpg", false, true))
                        return false;
                    dirtTextureSource = roadMesh;
                } else if (!roadMesh->ShareTextureFrom(*dirtTextureSource, true)) {
                    return false;
                }
            } else if (surface == RenderSurface::Gravel) {
                if (!dirtTextureSource) {
                    if (!roadMesh->LoadTexture("Assets/road/dirt.jpg"))
                        return false;
                    dirtTextureSource = roadMesh;
                } else if (!roadMesh->ShareTextureFrom(*dirtTextureSource)) {
                    return false;
                }
            }

            auto object = std::make_unique<Object>();
            object->SetName(name);
            object->SetScenePersistent(false);
            object->SetMesh(meshManager.Get(name));
            object->SetColor(Vec3(1.0f, 1.0f, 1.0f));
            object->SetRenderSurface(surface);
            object->SetSurfaceBlend(blend);
            m_objects.push_back(std::move(object));
        }

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
