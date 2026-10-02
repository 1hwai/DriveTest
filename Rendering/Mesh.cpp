#include "Mesh.h"
#include "Shader.h"

#include "../Physics/Terrain.h"
#include "../Physics/Road.h"

#include <glad/gl.h>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <cmath>
#include <cfloat>
#include <string>
#include <unordered_map>
#include <assimp/material.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace {
    struct Vertex {
        float position[3];
        float normal[3];
        float texCoord[2];
    };

    struct NodeMesh {
        const aiMesh* mesh;
        aiMatrix4x4 transform;
    };

    void CollectNodeMeshes(const aiScene* scene, const aiNode* node, const aiMatrix4x4& parentTransform, std::vector<NodeMesh>& output) {
        const aiMatrix4x4 transform = parentTransform * node->mTransformation;
        for (unsigned int i = 0; i < node->mNumMeshes; ++i)
            output.push_back({scene->mMeshes[node->mMeshes[i]], transform});
        for (unsigned int i = 0; i < node->mNumChildren; ++i)
            CollectNodeMeshes(scene, node->mChildren[i], transform, output);
    }

    bool IsWheelMesh(const aiMesh* mesh) {
        const std::string name = mesh->mName.C_Str();
        return name.rfind("Wheel.", 0) == 0 && name.find("_wheel_0") != std::string::npos;
    }

    void ConfigureVertexAttributes() {
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, texCoord)));
        glEnableVertexAttribArray(2);
    }
}

Mesh::Mesh()
    : m_vao(0),
    m_vbo(0),
    m_ebo(0),
    m_vertexCount(0),
    m_indexCount(0),
    m_indexed(false),
    m_materialTexture(0),
    m_secondaryTexture(0),
    m_submeshes(),
    m_textures() {}

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


bool Mesh::CreateCylinder(int segments) {
    Destroy();
    segments = std::max(6, segments);
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    const float pi = 3.14159265358979f;
    for (int i = 0; i <= segments; ++i) {
        const float angle = 2.0f * pi * static_cast<float>(i) / segments;
        const float x = std::cos(angle), z = std::sin(angle);
        vertices.push_back({{x * 0.5f, -0.5f, z * 0.5f}, {x, 0.0f, z}, {static_cast<float>(i) / segments, 0.0f}});
        vertices.push_back({{x * 0.5f, 0.5f, z * 0.5f}, {x, 0.0f, z}, {static_cast<float>(i) / segments, 1.0f}});
    }
    for (int i = 0; i < segments; ++i) {
        const unsigned int a = static_cast<unsigned int>(i * 2);
        indices.insert(indices.end(), {a, a + 1, a + 2, a + 1, a + 3, a + 2});
    }
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
    if (!m_vao || !m_vbo || !m_ebo) { Destroy(); return false; }
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<long>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<long>(indices.size() * sizeof(unsigned int)), indices.data(), GL_STATIC_DRAW);
    ConfigureVertexAttributes();
    glBindVertexArray(0);
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;
    return true;
}

bool Mesh::CreateDisc(int segments) {
    Destroy();
    segments = std::max(12, segments);
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.push_back({{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.5f, 0.5f}});
    const float pi = 3.14159265358979f;
    for (int i = 0; i < segments; ++i) {
        const float angle = 2.0f * pi * static_cast<float>(i) / segments;
        const float x = std::cos(angle), y = std::sin(angle);
        vertices.push_back({{x * 0.5f, y * 0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {x * 0.5f + 0.5f, y * 0.5f + 0.5f}});
    }
    for (int i = 0; i < segments; ++i)
        indices.insert(indices.end(), {0u, static_cast<unsigned int>(i + 1), static_cast<unsigned int>((i + 1) % segments + 1)});
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
    if (!m_vao || !m_vbo || !m_ebo) { Destroy(); return false; }
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<long>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<long>(indices.size() * sizeof(unsigned int)), indices.data(), GL_STATIC_DRAW);
    ConfigureVertexAttributes();
    glBindVertexArray(0);
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;
    return true;
}

bool Mesh::CreateFoliageBillboard(bool shrub) {
    Destroy();
    const Vertex vertices[] = {
        {{-0.5f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
        {{ 0.5f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
        {{ 0.5f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
        {{-0.5f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}}
    };
    const unsigned int indices[] = {0, 1, 2, 0, 2, 3, 2, 1, 0, 3, 2, 0};
    (void)shrub;

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
    if (!m_vao || !m_vbo || !m_ebo) {
        Destroy();
        return false;
    }

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    ConfigureVertexAttributes();
    glBindVertexArray(0);

    m_indexCount = 12;
    m_indexed = true;
    return true;
}

bool Mesh::LoadTexture(const std::string& path, bool removeBackground, bool secondary) {
    int width = 0, height = 0, channels = 0;
    stbi_uc* pixels = stbi_load(path.c_str(), &width, &height, &channels, STBI_rgb_alpha);
    if (!pixels || width <= 0 || height <= 0) {
        if (pixels) stbi_image_free(pixels);
        return false;
    }

    const size_t rowBytes = static_cast<size_t>(width) * 4;
    std::vector<unsigned char> row(rowBytes);
    for (int y = 0; y < height / 2; ++y) {
        unsigned char* top = pixels + static_cast<size_t>(y) * rowBytes;
        unsigned char* bottom = pixels + static_cast<size_t>(height - 1 - y) * rowBytes;
        std::copy(top, top + rowBytes, row.begin());
        std::copy(bottom, bottom + rowBytes, top);
        std::copy(row.begin(), row.end(), bottom);
    }

    if (removeBackground) {
        const auto corner = [&](int x, int y, int channel) {
            return pixels[(static_cast<size_t>(y) * width + x) * 4 + channel];
        };
        float background[3] = {};
        const int cornerX[4] = {0, width - 1, 0, width - 1};
        const int cornerY[4] = {0, 0, height - 1, height - 1};
        for (int i = 0; i < 4; ++i)
            for (int c = 0; c < 3; ++c)
                background[c] += corner(cornerX[i], cornerY[i], c) * 0.25f;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                unsigned char* pixel = pixels + (static_cast<size_t>(y) * width + x) * 4;
                const float dr = pixel[0] - background[0];
                const float dg = pixel[1] - background[1];
                const float db = pixel[2] - background[2];
                const float distance = std::sqrt(dr * dr + dg * dg + db * db);
                const float alpha = std::clamp((distance - 24.0f) / 36.0f, 0.0f, 1.0f);
                pixel[3] = static_cast<unsigned char>(alpha * 255.0f);
            }
        }
    }

    unsigned int texture = 0;
    glGenTextures(1, &texture);
    if (texture == 0) {
        stbi_image_free(pixels);
        return false;
    }

    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, removeBackground ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, removeBackground ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(pixels);

    if (secondary)
        m_secondaryTexture = texture;
    else
        m_materialTexture = texture;
    m_textures.push_back(texture);
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




bool Mesh::LoadFromFile(const std::string& path, bool wheelOnly) {
    Destroy();
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_ImproveCacheLocality);
    if (scene == nullptr || scene->mRootNode == nullptr || scene->mNumMeshes == 0)
        return false;

    std::vector<NodeMesh> sourceMeshes;
    CollectNodeMeshes(scene, scene->mRootNode, aiMatrix4x4(), sourceMeshes);
    if (sourceMeshes.empty())
        return false;

    aiVector3D minimum(FLT_MAX, FLT_MAX, FLT_MAX);
    aiVector3D maximum(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (const NodeMesh& item : sourceMeshes) {
        for (unsigned int i = 0; i < item.mesh->mNumVertices; ++i) {
            const aiVector3D p = item.transform * item.mesh->mVertices[i];
            minimum.x = std::min(minimum.x, p.x);
            minimum.y = std::min(minimum.y, p.y);
            minimum.z = std::min(minimum.z, p.z);
            maximum.x = std::max(maximum.x, p.x);
            maximum.y = std::max(maximum.y, p.y);
            maximum.z = std::max(maximum.z, p.z);
        }
    }
    const float horizontalLength = std::max(maximum.x - minimum.x, maximum.z - minimum.z);
    if (horizontalLength <= 0.001f || maximum.y - minimum.y <= 0.001f)
        return false;
    const float modelScale = 4.4f / horizontalLength;
    const aiVector3D modelCenter((minimum.x + maximum.x) * 0.5f, minimum.y, (minimum.z + maximum.z) * 0.5f);
    std::unordered_map<unsigned int, unsigned int> materialTextures;

    auto loadTexture = [&](unsigned int materialIndex) -> unsigned int {
        const auto cached = materialTextures.find(materialIndex);
        if (cached != materialTextures.end()) return cached->second;
        if (materialIndex >= scene->mNumMaterials) return 0;
        const aiMaterial* material = scene->mMaterials[materialIndex];
        aiString texturePath;
        if (material->GetTexture(aiTextureType_BASE_COLOR, 0, &texturePath) != AI_SUCCESS &&
            material->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath) != AI_SUCCESS) {
            materialTextures.emplace(materialIndex, 0);
            return 0;
        }
        const aiTexture* embedded = scene->GetEmbeddedTexture(texturePath.C_Str());
        if (embedded == nullptr || embedded->mHeight != 0 || embedded->mWidth == 0) {
            materialTextures.emplace(materialIndex, 0);
            return 0;
        }
        int width = 0, textureHeight = 0, channels = 0;
        stbi_uc* pixels = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(embedded->pcData), static_cast<int>(embedded->mWidth), &width, &textureHeight, &channels, STBI_rgb_alpha);
        if (pixels == nullptr) {
            materialTextures.emplace(materialIndex, 0);
            return 0;
        }
        unsigned int texture = 0;
        glGenTextures(1, &texture);
        if (texture != 0) {
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, textureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            m_textures.push_back(texture);
        }
        stbi_image_free(pixels);
        glBindTexture(GL_TEXTURE_2D, 0);
        materialTextures.emplace(materialIndex, texture);
        return texture;
    };

    bool loadedWheel = false;
    for (const NodeMesh& item : sourceMeshes) {
        const aiMesh* source = item.mesh;
        if (wheelOnly) {
            if (!IsWheelMesh(source) || loadedWheel) continue;
            loadedWheel = true;
        } else if (IsWheelMesh(source)) {
            continue;
        }

        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        vertices.reserve(source->mNumVertices);
        indices.reserve(source->mNumFaces * 3);
        aiMatrix3x3 normalTransform(item.transform);
        normalTransform.Inverse();
        normalTransform.Transpose();
        aiVector3D partMinimum(FLT_MAX, FLT_MAX, FLT_MAX);
        aiVector3D partMaximum(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        for (unsigned int i = 0; i < source->mNumVertices; ++i) {
            const aiVector3D p = item.transform * source->mVertices[i];
            const aiVector3D n = normalTransform * (source->HasNormals() ? source->mNormals[i] : aiVector3D(0.0f, 1.0f, 0.0f));
            Vertex vertex = {
                {(p.x - modelCenter.x) * modelScale, (p.y - modelCenter.y) * modelScale - 0.74f, (p.z - modelCenter.z) * modelScale},
                {n.x, n.y, n.z},
                {0.0f, 0.0f}
            };
            if (source->HasTextureCoords(0)) {
                vertex.texCoord[0] = source->mTextureCoords[0][i].x;
                vertex.texCoord[1] = source->mTextureCoords[0][i].y;
            }
            vertices.push_back(vertex);
            partMinimum.x = std::min(partMinimum.x, vertex.position[0]);
            partMinimum.y = std::min(partMinimum.y, vertex.position[1]);
            partMinimum.z = std::min(partMinimum.z, vertex.position[2]);
            partMaximum.x = std::max(partMaximum.x, vertex.position[0]);
            partMaximum.y = std::max(partMaximum.y, vertex.position[1]);
            partMaximum.z = std::max(partMaximum.z, vertex.position[2]);
        }
        for (unsigned int faceIndex = 0; faceIndex < source->mNumFaces; ++faceIndex) {
            const aiFace& face = source->mFaces[faceIndex];
            for (unsigned int i = 0; i < face.mNumIndices; ++i) indices.push_back(face.mIndices[i]);
        }
        if (vertices.empty() || indices.empty()) continue;

        if (wheelOnly) {
            const float cx = (partMinimum.x + partMaximum.x) * 0.5f;
            const float cy = (partMinimum.y + partMaximum.y) * 0.5f;
            const float cz = (partMinimum.z + partMaximum.z) * 0.5f;
            const float radius = std::max(partMaximum.y - partMinimum.y, partMaximum.z - partMinimum.z) * 0.5f;
            if (radius <= 0.001f) continue;
            const float wheelScale = 0.5f / radius;
            for (Vertex& vertex : vertices) {
                vertex.position[0] = (vertex.position[0] - cx) * wheelScale;
                vertex.position[1] = (vertex.position[1] - cy) * wheelScale;
                vertex.position[2] = (vertex.position[2] - cz) * wheelScale;
            }
        }

        Submesh submesh;
        glGenVertexArrays(1, &submesh.vao);
        glGenBuffers(1, &submesh.vbo);
        glGenBuffers(1, &submesh.ebo);
        if (submesh.vao == 0 || submesh.vbo == 0 || submesh.ebo == 0) {
            if (submesh.ebo) glDeleteBuffers(1, &submesh.ebo);
            if (submesh.vbo) glDeleteBuffers(1, &submesh.vbo);
            if (submesh.vao) glDeleteVertexArrays(1, &submesh.vao);
            Destroy();
            return false;
        }
        glBindVertexArray(submesh.vao);
        glBindBuffer(GL_ARRAY_BUFFER, submesh.vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<long>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, submesh.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<long>(indices.size() * sizeof(unsigned int)), indices.data(), GL_STATIC_DRAW);
        ConfigureVertexAttributes();
        glBindVertexArray(0);
        submesh.indexCount = static_cast<unsigned int>(indices.size());

        if (source->mMaterialIndex < scene->mNumMaterials) {
            const aiMaterial* material = scene->mMaterials[source->mMaterialIndex];
            aiColor4D color(1.0f, 1.0f, 1.0f, 1.0f);
            if (material->Get(AI_MATKEY_BASE_COLOR, color) != AI_SUCCESS)
                material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
            submesh.baseColor = Vec3(color.r, color.g, color.b);
            submesh.texture = loadTexture(source->mMaterialIndex);
        }
        m_submeshes.push_back(submesh);
        if (wheelOnly) break;
    }

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    return !m_submeshes.empty();
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
                {normal.x, normal.y, normal.z},
                {worldX * 0.08f, worldZ * 0.08f}
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



bool Mesh::CreateRoad(const Road& road) {
    return CreateRoadSection(road, 0, road.GetSegmentCount());
}

bool Mesh::CreateRoadSection(const Road& road, size_t firstSegment, size_t endSegment) {
    Destroy();
    const std::vector<Vec3>& leftEdge = road.GetLeftEdge();
    const std::vector<Vec3>& rightEdge = road.GetRightEdge();
    const size_t segmentCount = road.GetSegmentCount();
    firstSegment = std::min(firstSegment, segmentCount);
    endSegment = std::min(endSegment, segmentCount);
    if (firstSegment >= endSegment || leftEdge.size() < 2 || leftEdge.size() != rightEdge.size())
        return false;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve((endSegment - firstSegment + 1) * 2);
    indices.reserve((endSegment - firstSegment) * 6);

    for (size_t i = firstSegment; i <= endSegment; ++i) {
        const size_t previousIndex = i == 0 ? i : i - 1;
        const size_t nextIndex = std::min(i + 1, leftEdge.size() - 1);
        const Vec3 tangent = (leftEdge[nextIndex] + rightEdge[nextIndex]) -
            (leftEdge[previousIndex] + rightEdge[previousIndex]);
        const Vec3 width = rightEdge[i] - leftEdge[i];
        Vec3 normal = tangent.Cross(width).Normalized();
        if (normal.y < 0.0f)
            normal = -normal;
        vertices.push_back({{leftEdge[i].x, leftEdge[i].y, leftEdge[i].z}, {normal.x, normal.y, normal.z}, {0.0f, static_cast<float>(i) * 0.25f}});
        vertices.push_back({{rightEdge[i].x, rightEdge[i].y, rightEdge[i].z}, {normal.x, normal.y, normal.z}, {1.0f, static_cast<float>(i) * 0.25f}});
    }

    for (size_t i = 0; i < endSegment - firstSegment; ++i) {
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
    ConfigureVertexAttributes();
    glBindVertexArray(0);
    m_vertexCount = 0;
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;
    return true;
}

bool Mesh::CreateRoadsideWall(const Terrain& terrain, const Road& road, bool leftSide, size_t firstSegment, size_t endSegment, float offset, float wallHeight) {
    Destroy();
    const std::vector<Vec3>& leftEdge = road.GetLeftEdge();
    const std::vector<Vec3>& rightEdge = road.GetRightEdge();
    const size_t segmentCount = road.GetSegmentCount();
    firstSegment = std::min(firstSegment, segmentCount);
    endSegment = std::min(endSegment, segmentCount);
    if (firstSegment >= endSegment || leftEdge.size() < 2 || leftEdge.size() != rightEdge.size())
        return false;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve((endSegment - firstSegment + 1) * 4);
    indices.reserve((endSegment - firstSegment) * 6);

    for (size_t i = firstSegment; i <= endSegment; ++i) {
        const Vec3 center = (leftEdge[i] + rightEdge[i]) * 0.5f;
        const Vec3 edge = leftSide ? leftEdge[i] : rightEdge[i];
        Vec3 outward = edge - center;
        outward.y = 0.0f;
        if (outward.LengthSquared() < 0.0001f)
            outward = Vec3(leftSide ? -1.0f : 1.0f, 0.0f, 0.0f);
        else
            outward = outward.Normalized();

        const float x = edge.x + outward.x * offset;
        const float z = edge.z + outward.z * offset;
        const float ground = terrain.GetHeight(x, z);
        if (!std::isfinite(ground))
            return false;

        const float variation = 0.68f + 0.18f * std::sin(static_cast<float>(i) * 0.37f) +
            0.12f * std::sin(static_cast<float>(i) * 0.113f + (leftSide ? 1.7f : 0.3f));
        const float top = ground + wallHeight * variation;
        const Vec3 tangent = (leftEdge[std::min(i + 1, segmentCount)] + rightEdge[std::min(i + 1, segmentCount)]) -
            (leftEdge[i == 0 ? i : i - 1] + rightEdge[i == 0 ? i : i - 1]);
        Vec3 normal = leftSide ? tangent.Cross(Vec3(0.0f, 1.0f, 0.0f)) : Vec3(0.0f, 1.0f, 0.0f).Cross(tangent);
        normal.y = 0.0f;
        normal = normal.Normalized();

        vertices.push_back({{x, ground, z}, {normal.x, normal.y, normal.z}, {0.0f, 0.0f}});
        vertices.push_back({{x, top, z}, {normal.x, normal.y, normal.z}, {0.0f, 1.0f}});
    }

    for (size_t i = 0; i < endSegment - firstSegment; ++i) {
        const unsigned int bottomA = static_cast<unsigned int>(i * 2);
        const unsigned int topA = bottomA + 1;
        const unsigned int bottomB = bottomA + 2;
        const unsigned int topB = bottomA + 3;
        if (leftSide) {
            indices.insert(indices.end(), {bottomA, bottomB, topA, bottomB, topB, topA});
        } else {
            indices.insert(indices.end(), {bottomA, topA, bottomB, bottomB, topA, topB});
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
    glBufferData(GL_ARRAY_BUFFER, static_cast<long>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<long>(indices.size() * sizeof(unsigned int)), indices.data(), GL_STATIC_DRAW);
    ConfigureVertexAttributes();
    glBindVertexArray(0);
    m_vertexCount = 0;
    m_indexCount = static_cast<unsigned int>(indices.size());
    m_indexed = true;
    return true;
}

void Mesh::Draw() const {
    if (m_vao == 0) return;
    glBindVertexArray(m_vao);
    if (m_indexed) glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
    else glDrawArrays(GL_TRIANGLES, 0, m_vertexCount);
    glBindVertexArray(0);
}

void Mesh::Draw(Shader& shader, const Vec3& color) const {
    if (m_submeshes.empty()) {
        shader.SetVec3("uBaseColor", color);
        shader.SetInt("uBaseColorTexture", 0);
        shader.SetInt("uSecondaryTexture", 1);
        shader.SetInt("uUseTexture", m_materialTexture != 0 ? 1 : 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_materialTexture);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, m_secondaryTexture);
        Draw();
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
        shader.SetInt("uUseTexture", 0);
        return;
    }

    glActiveTexture(GL_TEXTURE0);
    for (const Submesh& submesh : m_submeshes) {
        shader.SetVec3("uBaseColor", Vec3(color.x * submesh.baseColor.x, color.y * submesh.baseColor.y, color.z * submesh.baseColor.z));
        shader.SetInt("uBaseColorTexture", 0);
        shader.SetInt("uUseTexture", submesh.texture != 0 ? 1 : 0);
        glBindTexture(GL_TEXTURE_2D, submesh.texture);
        glBindVertexArray(submesh.vao);
        glDrawElements(GL_TRIANGLES, submesh.indexCount, GL_UNSIGNED_INT, nullptr);
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    shader.SetInt("uUseTexture", 0);
}

void Mesh::Destroy() {
    for (Submesh& submesh : m_submeshes) {
        if (submesh.ebo) glDeleteBuffers(1, &submesh.ebo);
        if (submesh.vbo) glDeleteBuffers(1, &submesh.vbo);
        if (submesh.vao) glDeleteVertexArrays(1, &submesh.vao);
    }
    m_submeshes.clear();
    for (unsigned int texture : m_textures) if (texture) glDeleteTextures(1, &texture);
    m_textures.clear();
    if (m_ebo) { glDeleteBuffers(1, &m_ebo); m_ebo = 0; }
    if (m_vbo) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_vao) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    m_vertexCount = 0;
    m_indexCount = 0;
    m_indexed = false;
    m_materialTexture = 0;
    m_secondaryTexture = 0;
}