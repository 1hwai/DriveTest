#pragma once

#include <memory>
#include <string>
#include <unordered_map>

class Mesh;

class MeshManager {
public:
    MeshManager() = default;
    ~MeshManager();

    bool CreateCube(const std::string& name);
    bool CreateTriangle(const std::string& name);
    bool CreateSphere(const std::string& name);
    bool CreateWheel(const std::string& name);

    Mesh* Get(const std::string& name);
    const Mesh* Get(const std::string& name) const;

    void Clear();

private:
    std::unordered_map<
        std::string,
        std::unique_ptr<Mesh>
    > m_meshes;
};