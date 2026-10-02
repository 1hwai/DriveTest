#include "MeshManager.h"

#include "Mesh.h"
#include "../Physics/Terrain.h"
#include "../Physics/Road.h"

MeshManager::~MeshManager() = default;

bool MeshManager::CreateCube(const std::string& name)
{
    if (m_meshes.find(name) != m_meshes.end())
        return false;

    auto mesh = std::make_unique<Mesh>();

    if (!mesh->CreateCube())
        return false;

    m_meshes.emplace(
        name,
        std::move(mesh)
    );

    return true;
}

bool MeshManager::CreateTriangle(const std::string& name)
{
    if (m_meshes.find(name) != m_meshes.end())
        return false;

    auto mesh = std::make_unique<Mesh>();

    if (!mesh->CreateTriangle())
        return false;

    m_meshes.emplace(
        name,
        std::move(mesh)
    );

    return true;
}

bool MeshManager::CreateSphere(const std::string& name)
{
    if (m_meshes.find(name) != m_meshes.end())
        return false;

    auto mesh = std::make_unique<Mesh>();

    if (!mesh->CreateSphere())
        return false;

    m_meshes.emplace(
        name,
        std::move(mesh)
    );

    return true;
}

bool MeshManager::CreateCylinder(const std::string& name)
{
    if (m_meshes.find(name) != m_meshes.end()) return false;
    auto mesh = std::make_unique<Mesh>();
    if (!mesh->CreateCylinder()) return false;
    m_meshes.emplace(name, std::move(mesh));
    return true;
}

bool MeshManager::CreateDisc(const std::string& name)
{
    if (m_meshes.find(name) != m_meshes.end()) return false;
    auto mesh = std::make_unique<Mesh>();
    if (!mesh->CreateDisc()) return false;
    m_meshes.emplace(name, std::move(mesh));
    return true;
}

bool MeshManager::CreateFoliageBillboard(const std::string& name, bool shrub)
{
    if (m_meshes.find(name) != m_meshes.end()) return false;
    auto mesh = std::make_unique<Mesh>();
    if (!mesh->CreateFoliageBillboard(shrub)) return false;
    m_meshes.emplace(name, std::move(mesh));
    return true;
}

bool MeshManager::CreateWheel(const std::string& name)
{
    if (m_meshes.find(name) != m_meshes.end())
        return false;

    auto mesh = std::make_unique<Mesh>();

    if (!mesh->CreateWheel())
        return false;

    m_meshes.emplace(
        name,
        std::move(mesh)
    );

    return true;
}



bool MeshManager::LoadFromFile(const std::string& name, const std::string& path, bool wheelOnly) {
    if (m_meshes.find(name) != m_meshes.end())
        return false;

    auto mesh = std::make_unique<Mesh>();
    if (!mesh->LoadFromFile(path, wheelOnly))
        return false;

    m_meshes.emplace(name, std::move(mesh));
    return true;
}

bool MeshManager::LoadTexture(const std::string& name, const std::string& path, bool removeBackground, bool secondary) {
    Mesh* mesh = Get(name);
    return mesh != nullptr && mesh->LoadTexture(path, removeBackground, secondary);
}

bool MeshManager::CreateTerrain(const std::string& name, const Terrain& terrain) {
    if (m_meshes.find(name) != m_meshes.end())
        return false;

    auto mesh = std::make_unique<Mesh>();

    if (!mesh->CreateTerrain(terrain))
        return false;

    m_meshes.emplace(
        name,
        std::move(mesh)
    );

    return true;
}


bool MeshManager::CreateRoad(const std::string& name, const Road& road) {
    if (m_meshes.find(name) != m_meshes.end())
        return false;

    auto mesh = std::make_unique<Mesh>();

    if (!mesh->CreateRoad(road))
        return false;

    m_meshes.emplace(name, std::move(mesh));
    return true;
}

bool MeshManager::CreateRoadSection(const std::string& name, const Road& road, size_t firstSegment, size_t endSegment) {
    if (m_meshes.find(name) != m_meshes.end())
        return false;
    auto mesh = std::make_unique<Mesh>();
    if (!mesh->CreateRoadSection(road, firstSegment, endSegment))
        return false;
    m_meshes.emplace(name, std::move(mesh));
    return true;
}

bool MeshManager::CreateRoadsideWall(const std::string& name, const Terrain& terrain, const Road& road, bool leftSide, size_t firstSegment, size_t endSegment, float offset, float wallHeight) {
    if (m_meshes.find(name) != m_meshes.end())
        return false;
    auto mesh = std::make_unique<Mesh>();
    if (!mesh->CreateRoadsideWall(terrain, road, leftSide, firstSegment, endSegment, offset, wallHeight))
        return false;
    m_meshes.emplace(name, std::move(mesh));
    return true;
}

Mesh* MeshManager::Get(const std::string& name)
{
    auto it = m_meshes.find(name);

    if (it == m_meshes.end())
        return nullptr;

    return it->second.get();
}

const Mesh* MeshManager::Get(const std::string& name) const
{
    auto it = m_meshes.find(name);

    if (it == m_meshes.end())
        return nullptr;

    return it->second.get();
}

void MeshManager::Clear()
{
    m_meshes.clear();
}