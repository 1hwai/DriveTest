#include "Mesh.h"

#include "../Physics/Terrain.h"
#include "../Physics/Road.h"

#include <glad/gl.h>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <cmath>

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



bool Mesh::CreateImprezaBody() {
    Destroy();
    struct Section { float z, width, bottom, shoulder, top, roofWidth; };
    const Section sections[] = {
        {-2.12f,.76f,-.28f,.10f,.38f,.69f},{-1.78f,.84f,-.30f,.13f,.43f,.73f},
        {-1.22f,.88f,-.30f,.17f,.47f,.74f},{-.62f,.82f,-.29f,.19f,.79f,.62f},
        {.35f,.82f,-.29f,.19f,.79f,.62f},{.92f,.87f,-.29f,.17f,.48f,.73f},
        {1.55f,.84f,-.29f,.13f,.43f,.72f},{2.10f,.77f,-.26f,.08f,.36f,.67f}
    };
    std::vector<Vertex> vertices;
    const auto tri=[&](const Vec3& a,const Vec3& b,const Vec3& c) {
        Vec3 n=(b-a).Cross(c-a).Normalized();
        vertices.push_back({{a.x,a.y,a.z},{n.x,n.y,n.z}});
        vertices.push_back({{b.x,b.y,b.z},{n.x,n.y,n.z}});
        vertices.push_back({{c.x,c.y,c.z},{n.x,n.y,n.z}});
    };
    const auto ring=[](const Section& s) {
        return std::vector<Vec3>{{-s.width,s.bottom,s.z},{s.width,s.bottom,s.z},
            {s.width,s.shoulder,s.z},{s.roofWidth,s.top,s.z},
            {-s.roofWidth,s.top,s.z},{-s.width,s.shoulder,s.z}};
    };
    std::vector<std::vector<Vec3>> rings;
    for(const Section& s:sections) rings.push_back(ring(s));
    for(size_t i=0;i+1<rings.size();++i) for(size_t j=0;j<rings[i].size();++j) {
        size_t k=(j+1)%rings[i].size();
        const Vec3& a=rings[i][j]; const Vec3& b=rings[i][k];
        const Vec3& c=rings[i+1][j]; const Vec3& d=rings[i+1][k];
        tri(a,b,c); tri(b,d,c);
    }
    for(size_t j=1;j+1<rings.front().size();++j) tri(rings.front()[0],rings.front()[j],rings.front()[j+1]);
    for(size_t j=1;j+1<rings.back().size();++j) tri(rings.back()[0],rings.back()[j+1],rings.back()[j]);
    tri({-.22f,.405f,1.08f},{.22f,.405f,1.08f},{.17f,.49f,1.54f});
    tri({-.22f,.405f,1.08f},{.17f,.49f,1.54f},{-.17f,.49f,1.54f});
    tri({-.72f,.40f,-1.76f},{-.66f,.77f,-1.72f},{-.60f,.77f,-1.65f});
    tri({.72f,.40f,-1.76f},{.60f,.77f,-1.65f},{.66f,.77f,-1.72f});
    tri({-.91f,.77f,-1.82f},{.91f,.77f,-1.82f},{.88f,.84f,-2.10f});
    tri({-.91f,.77f,-1.82f},{.88f,.84f,-2.10f},{-.88f,.84f,-2.10f});
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