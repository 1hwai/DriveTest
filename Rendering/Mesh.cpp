#include "Mesh.h"

#include <glad/gl.h>
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