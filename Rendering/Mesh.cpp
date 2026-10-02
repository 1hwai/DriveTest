#include "Mesh.h"

#include "../Physics/Terrain.h"
#include "../Physics/Road.h"

#include <glad/gl.h>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <cmath>
#include <cfloat>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace {
    struct Vertex {
        float position[3];
        float normal[3];
    };
}

Mesh::Mesh()
    : m_vao(0),
    m_vbo(0),
    m_ebo(0),
    m_vertexCount(0),
    m_indexCount(0),
    m_indexed(false) {}

Mesh::~Mesh() {
    Destroy();
}

bool Mesh::CreateTriangle() {
    Destroy();

    const Vertex vertices[] = {
        {{ 0.0f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{-0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{ 0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}}
    };

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    if (m_vao == 0 || m_vbo == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);

    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(vertices),
        vertices,
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, position))
    );
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, normal))
    );
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    m_vertexCount = 3;
    m_indexCount = 0;
    m_indexed = false;

    return true;
}

bool Mesh::CreateCube() {
    Destroy();

    const Vertex vertices[] = {
        // Front
        {{-0.5f, -0.5f,  0.5f}, { 0.0f,  0.0f,  1.0f}},
        {{ 0.5f, -0.5f,  0.5f}, { 0.0f,  0.0f,  1.0f}},
        {{ 0.5f,  0.5f,  0.5f}, { 0.0f,  0.0f,  1.0f}},
        {{-0.5f,  0.5f,  0.5f}, { 0.0f,  0.0f,  1.0f}},

        // Back
        {{ 0.5f, -0.5f, -0.5f}, { 0.0f,  0.0f, -1.0f}},
        {{-0.5f, -0.5f, -0.5f}, { 0.0f,  0.0f, -1.0f}},
        {{-0.5f,  0.5f, -0.5f}, { 0.0f,  0.0f, -1.0f}},
        {{ 0.5f,  0.5f, -0.5f}, { 0.0f,  0.0f, -1.0f}},

        // Left
        {{-0.5f, -0.5f, -0.5f}, {-1.0f,  0.0f,  0.0f}},
        {{-0.5f, -0.5f,  0.5f}, {-1.0f,  0.0f,  0.0f}},
        {{-0.5f,  0.5f,  0.5f}, {-1.0f,  0.0f,  0.0f}},
        {{-0.5f,  0.5f, -0.5f}, {-1.0f,  0.0f,  0.0f}},

        // Right
        {{ 0.5f, -0.5f,  0.5f}, { 1.0f,  0.0f,  0.0f}},
        {{ 0.5f, -0.5f, -0.5f}, { 1.0f,  0.0f,  0.0f}},
        {{ 0.5f,  0.5f, -0.5f}, { 1.0f,  0.0f, 0.0f}},
        {{ 0.5f,  0.5f,  0.5f}, { 1.0f,  0.0f, 0.0f}},

        // Top
        {{-0.5f,  0.5f,  0.5f}, { 0.0f,  1.0f,  0.0f}},
        {{ 0.5f,  0.5f,  0.5f}, { 0.0f,  1.0f, 0.0f}},
        {{ 0.5f,  0.5f, -0.5f}, { 0.0f, 1.0f, 0.0f}},
        {{-0.5f,  0.5f, -0.5f}, { 0.0f, 1.0f, 0.0f}},

        // Bottom
        {{-0.5f, -0.5f, -0.5f}, { 0.0f, -1.0f,  0.0f}},
        {{ 0.5f, -0.5f, -0.5f}, { 0.0f, -1.0f,  0.0f}},
        {{ 0.5f, -0.5f,  0.5f}, { 0.0f, -1.0f,  0.0f}},
        {{-0.5f, -0.5f,  0.5f}, { 0.0f, -1.0f,  0.0f}}
    };

    const unsigned int indices[] = {
         0,  1,  2,   2,  3,  0,
         4,  5,  6,   6,  7,  4,
         8,  9, 10,  10, 11,  8,
        12, 13, 14,  14, 15, 12,
        16, 17, 18,  18, 19, 16,
        20, 21, 22,  22, 23, 20
    };

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    if (m_vao == 0 || m_vbo == 0 || m_ebo == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(vertices),
        vertices,
        GL_STATIC_DRAW
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        sizeof(indices),
        indices,
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, position))
    );
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, normal))
    );
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    m_vertexCount = 0;
    m_indexCount = 36;
    m_indexed = true;

    return true;
}

bool Mesh::CreateSphere(int segments, int rings) {
    Destroy();

    if (segments < 3) segments = 3;
    if (rings < 2) rings = 2;

    const float radius = 0.5f;
    const float pi = 3.14159265358979f;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    vertices.reserve((rings + 1) * (segments + 1));
    indices.reserve(rings * segments * 6);

    for (int ring = 0; ring <= rings; ++ring) {
        const float v = static_cast<float>(ring) / static_cast<float>(rings);
        const float phi = v * pi;

        const float y = std::cos(phi);
        const float ringRadius = std::sin(phi);

        for (int segment = 0; segment <= segments; ++segment) {
            const float u = static_cast<float>(segment) / static_cast<float>(segments);
            const float theta = u * 2.0f * pi;

            const float x = ringRadius * std::cos(theta);
            const float z = ringRadius * std::sin(theta);

            vertices.push_back({
                {x * radius, y * radius, z * radius},
                {x, y, z}
            });
        }
    }

    const int columns = segments + 1;

    for (int ring = 0; ring < rings; ++ring) {
        for (int segment = 0; segment < segments; ++segment) {
            const unsigned int a = ring * columns + segment;
            const unsigned int b = a + columns;
            const unsigned int c = a + 1;
            const unsigned int d = b + 1;

            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);

            indices.push_back(c);
            indices.push_back(b);
            indices.push_back(d);
        }
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    if (m_vao == 0 || m_vbo == 0 || m_ebo == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<long>(vertices.size() * sizeof(Vertex)),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        static_cast<long>(indices.size() * sizeof(unsigned int)),
        indices.data(),
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, position))
    );
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, normal))
    );
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    m_vertexCount = 0;
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;

    return true;
}


bool Mesh::CreateWheel(int segments, int widthSegments) {
    Destroy();

    if (segments < 8) segments = 8;
    if (widthSegments < 4) widthSegments = 4;

    const float pi = 3.14159265358979f;
    const float majorRadius = 0.34f;
    const float tubeRadius = 0.16f;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    vertices.reserve((widthSegments + 1) * (segments + 1));
    indices.reserve(widthSegments * segments * 6);

    for (int width = 0; width <= widthSegments; ++width) {
        const float v = static_cast<float>(width) / static_cast<float>(widthSegments);
        const float phi = v * 2.0f * pi;

        const float cosPhi = std::cos(phi);
        const float sinPhi = std::sin(phi);

        for (int segment = 0; segment <= segments; ++segment) {
            const float u = static_cast<float>(segment) / static_cast<float>(segments);
            const float theta = u * 2.0f * pi;

            const float cosTheta = std::cos(theta);
            const float sinTheta = std::sin(theta);

            const float radial = majorRadius + tubeRadius * sinPhi;

            vertices.push_back({
                {
                    tubeRadius * cosPhi,
                    radial * cosTheta,
                    radial * sinTheta
                },
                {
                    cosPhi,
                    sinPhi * cosTheta,
                    sinPhi * sinTheta
                }
            });
        }
    }

    const int columns = segments + 1;

    for (int width = 0; width < widthSegments; ++width) {
        for (int segment = 0; segment < segments; ++segment) {
            const unsigned int a = width * columns + segment;
            const unsigned int b = a + columns;
            const unsigned int c = a + 1;
            const unsigned int d = b + 1;

            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);

            indices.push_back(c);
            indices.push_back(b);
            indices.push_back(d);
        }
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    if (m_vao == 0 || m_vbo == 0 || m_ebo == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<long>(vertices.size() * sizeof(Vertex)),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        static_cast<long>(indices.size() * sizeof(unsigned int)),
        indices.data(),
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, position))
    );
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, normal))
    );
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    m_vertexCount = 0;
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;

    return true;
}




bool Mesh::LoadFromFile(const std::string& path) {
    Destroy();

    Assimp::Importer importer;
    const aiScene* importedScene = importer.ReadFile(
        path,
        aiProcess_Triangulate |
        aiProcess_GenSmoothNormals |
        aiProcess_JoinIdenticalVertices |
        aiProcess_PreTransformVertices |
        aiProcess_ImproveCacheLocality
    );

    if (importedScene == nullptr || importedScene->mNumMeshes == 0)
        return false;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    aiVector3D minimum(FLT_MAX, FLT_MAX, FLT_MAX);
    aiVector3D maximum(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (unsigned int meshIndex = 0; meshIndex < importedScene->mNumMeshes; ++meshIndex) {
        const aiMesh* source = importedScene->mMeshes[meshIndex];
        const unsigned int baseVertex = static_cast<unsigned int>(vertices.size());

        for (unsigned int i = 0; i < source->mNumVertices; ++i) {
            const aiVector3D& p = source->mVertices[i];
            const aiVector3D n = source->HasNormals() ? source->mNormals[i] : aiVector3D(0.0f, 1.0f, 0.0f);
            vertices.push_back({{p.x, p.y, p.z}, {n.x, n.y, n.z}});
            minimum.x = std::min(minimum.x, p.x);
            minimum.y = std::min(minimum.y, p.y);
            minimum.z = std::min(minimum.z, p.z);
            maximum.x = std::max(maximum.x, p.x);
            maximum.y = std::max(maximum.y, p.y);
            maximum.z = std::max(maximum.z, p.z);
        }

        for (unsigned int faceIndex = 0; faceIndex < source->mNumFaces; ++faceIndex) {
            const aiFace& face = source->mFaces[faceIndex];
            for (unsigned int i = 0; i < face.mNumIndices; ++i)
                indices.push_back(baseVertex + face.mIndices[i]);
        }
    }

    const float width = maximum.x - minimum.x;
    const float height = maximum.y - minimum.y;
    const float depth = maximum.z - minimum.z;
    const float horizontalLength = std::max(width, depth);
    if (vertices.empty() || indices.empty() || horizontalLength <= 0.001f || height <= 0.001f)
        return false;

    // Normalize unknown source units to a roughly 4.4 m car and align the tire bottoms with the simulated wheels.
    const float scale = 4.4f / horizontalLength;
    for (Vertex& vertex : vertices) {
        vertex.position[0] = (vertex.position[0] - (minimum.x + maximum.x) * 0.5f) * scale;
        vertex.position[1] = (vertex.position[1] - minimum.y) * scale - 0.74f;
        vertex.position[2] = (vertex.position[2] - (minimum.z + maximum.z) * 0.5f) * scale;
        Vec3 normal(vertex.normal[0], vertex.normal[1], vertex.normal[2]);
        normal = normal.Normalized();
        vertex.normal[0] = normal.x;
        vertex.normal[1] = normal.y;
        vertex.normal[2] = normal.z;
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
    if (m_vao == 0 || m_vbo == 0 || m_ebo == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<long>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<long>(indices.size() * sizeof(unsigned int)), indices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    m_vertexCount = 0;
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;
    return true;
}

bool Mesh::CreateTerrain(const Terrain& terrain) {
    Destroy();

    const int resolution = terrain.GetResolution();
    const float size = terrain.GetSize();
    const std::vector<float>& heights = terrain.GetHeights();

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    vertices.reserve(static_cast<size_t>(resolution) * resolution);
    indices.reserve(static_cast<size_t>(resolution - 1) * (resolution - 1) * 6);

    const float halfSize = size * 0.5f;

    for (int z = 0; z < resolution; ++z) {
        for (int x = 0; x < resolution; ++x) {
            const float worldX =
                -halfSize +
                size * static_cast<float>(x) /
                static_cast<float>(resolution - 1);

            const float worldZ =
                -halfSize +
                size * static_cast<float>(z) /
                static_cast<float>(resolution - 1);

            const Vec3 normal =
                terrain.GetNormal(worldX, worldZ);

            vertices.push_back({
                {worldX, heights[static_cast<size_t>(z) * resolution + x], worldZ},
                {normal.x, normal.y, normal.z}
            });
        }
    }

    for (int z = 0; z < resolution - 1; ++z) {
        for (int x = 0; x < resolution - 1; ++x) {
            const unsigned int a = static_cast<unsigned int>(z * resolution + x);
            const unsigned int b = a + 1;
            const unsigned int c = a + static_cast<unsigned int>(resolution);
            const unsigned int d = c + 1;

            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(b);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(d);
        }
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    if (m_vao == 0 || m_vbo == 0 || m_ebo == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<long>(vertices.size() * sizeof(Vertex)),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        static_cast<long>(indices.size() * sizeof(unsigned int)),
        indices.data(),
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    m_vertexCount = 0;
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;
    return true;
}



bool Mesh::CreateImprezaGlass() {
    Destroy();
    std::vector<Vertex> vertices;
    const auto tri=[&](const Vec3& a,const Vec3& b,const Vec3& c) {
        Vec3 n=(b-a).Cross(c-a).Normalized();
        vertices.push_back({{a.x,a.y,a.z},{n.x,n.y,n.z}});
        vertices.push_back({{b.x,b.y,b.z},{n.x,n.y,n.z}});
        vertices.push_back({{c.x,c.y,c.z},{n.x,n.y,n.z}});
    };
    const auto quad=[&](const Vec3& a,const Vec3& b,const Vec3& c,const Vec3& d){tri(a,b,c);tri(a,c,d);};
    quad({-.59f,.43f,.80f},{.59f,.43f,.80f},{.47f,.73f,.36f},{-.47f,.73f,.36f});
    quad({-.47f,.73f,-.63f},{.47f,.73f,-.63f},{.60f,.43f,-.91f},{-.60f,.43f,-.91f});
    quad({-.825f,.25f,.73f},{-.635f,.69f,.34f},{-.635f,.69f,-.57f},{-.825f,.25f,-.82f});
    quad({.825f,.25f,.73f},{.825f,.25f,-.82f},{.635f,.69f,-.57f},{.635f,.69f,.34f});
    glGenVertexArrays(1,&m_vao); glGenBuffers(1,&m_vbo);
    if(!m_vao||!m_vbo){Destroy();return false;}
    glBindVertexArray(m_vao); glBindBuffer(GL_ARRAY_BUFFER,m_vbo);
    glBufferData(GL_ARRAY_BUFFER,static_cast<long>(vertices.size()*sizeof(Vertex)),vertices.data(),GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),reinterpret_cast<void*>(offsetof(Vertex,position))); glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),reinterpret_cast<void*>(offsetof(Vertex,normal))); glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER,0); glBindVertexArray(0);
    m_vertexCount=static_cast<unsigned int>(vertices.size()); m_indexCount=0; m_indexed=false;
    return true;
}

bool Mesh::CreateRoad(const Road& road) {
    Destroy();

    const std::vector<Vec3>& leftEdge = road.GetLeftEdge();
    const std::vector<Vec3>& rightEdge = road.GetRightEdge();

    if (leftEdge.size() < 2 || leftEdge.size() != rightEdge.size())
        return false;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(leftEdge.size() * 2);
    indices.reserve(road.GetSegmentCount() * 6);

    for (size_t i = 0; i < leftEdge.size(); ++i) {
        const size_t previousIndex = i == 0 ? i : i - 1;
        const size_t nextIndex = std::min(i + 1, leftEdge.size() - 1);
        const Vec3 tangent =
            (leftEdge[nextIndex] + rightEdge[nextIndex]) -
            (leftEdge[previousIndex] + rightEdge[previousIndex]);
        const Vec3 width = rightEdge[i] - leftEdge[i];
        Vec3 normal = tangent.Cross(width).Normalized();

        if (normal.y < 0.0f)
            normal = -normal;

        vertices.push_back({
            {leftEdge[i].x, leftEdge[i].y, leftEdge[i].z},
            {normal.x, normal.y, normal.z}
        });
        vertices.push_back({
            {rightEdge[i].x, rightEdge[i].y, rightEdge[i].z},
            {normal.x, normal.y, normal.z}
        });
    }

    for (size_t i = 0; i < road.GetSegmentCount(); ++i) {
        const unsigned int left = static_cast<unsigned int>(i * 2);
        const unsigned int right = left + 1;
        const unsigned int nextLeft = left + 2;
        const unsigned int nextRight = left + 3;

        indices.push_back(left);
        indices.push_back(nextLeft);
        indices.push_back(right);
        indices.push_back(right);
        indices.push_back(nextLeft);
        indices.push_back(nextRight);
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    if (m_vao == 0 || m_vbo == 0 || m_ebo == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<long>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<long>(indices.size() * sizeof(unsigned int)), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    m_vertexCount = 0;
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;
    return true;
}

void Mesh::Draw() const {
    if (m_vao == 0)
        return;

    glBindVertexArray(m_vao);

    if (m_indexed) {
        glDrawElements(
            GL_TRIANGLES,
            m_indexCount,
            GL_UNSIGNED_INT,
            nullptr
        );
    }
    else {
        glDrawArrays(
            GL_TRIANGLES,
            0,
            m_vertexCount
        );
    }

    glBindVertexArray(0);
}

void Mesh::Destroy() {
    if (m_ebo != 0) {
        glDeleteBuffers(1, &m_ebo);
        m_ebo = 0;
    }

    if (m_vbo != 0) {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }

    if (m_vao != 0) {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }

    m_vertexCount = 0;
    m_indexCount = 0;
    m_indexed = false;
}