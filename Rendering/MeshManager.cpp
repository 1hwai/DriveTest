#include "MeshManager.h"

#include "Mesh.h"

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