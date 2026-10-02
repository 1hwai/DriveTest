#pragma once

#include <cstddef>
#include <string>
#include <vector>
#include "../Core/Math/Vec3.h"

class Shader;
class Terrain;
class Road;

class Mesh {
public:
    Mesh();
    ~Mesh();

    bool CreateTriangle();
    bool CreateCube();
    bool CreateSphere(int segments = 16, int rings = 12);
    bool CreateCylinder(int segments = 12);
    bool CreateDisc(int segments = 32);
    bool CreateFoliageBillboard(bool shrub = false);
    bool CreateWheel(int segments = 32, int widthSegments = 8);
    bool LoadFromFile(const std::string& path, bool wheelOnly = false);
    bool LoadTexture(const std::string& path, bool removeBackground = false, bool secondary = false);
    bool CreateTerrain(const Terrain& terrain);
    bool CreateRoad(const Road& road);
    bool CreateRoadSection(const Road& road, std::size_t firstSegment, std::size_t endSegment);
    bool CreateRoadsideWall(const Terrain& terrain, const Road& road, bool leftSide, std::size_t firstSegment, std::size_t endSegment, float offset, float wallHeight);

    void Draw() const;
    void Draw(Shader& shader, const Vec3& color) const;
    void Destroy();

private:
    struct Submesh {
        unsigned int vao = 0;
        unsigned int vbo = 0;
        unsigned int ebo = 0;
        unsigned int indexCount = 0;
        unsigned int texture = 0;
        Vec3 baseColor = Vec3(1.0f, 1.0f, 1.0f);
    };

    unsigned int m_vao;
    unsigned int m_vbo;
    unsigned int m_ebo;
    unsigned int m_vertexCount;
    unsigned int m_indexCount;
    bool m_indexed;
    unsigned int m_materialTexture;
    unsigned int m_secondaryTexture;
    std::vector<Submesh> m_submeshes;
    std::vector<unsigned int> m_textures;
};
