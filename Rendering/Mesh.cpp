#include "Mesh.h"

#include <glad/gl.h>
#include <vector>
#include <cmath>

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

    const float vertices[] = {
         0.0f,  0.5f, 0.0f,
        -0.5f, -0.5f, 0.0f,
         0.5f, -0.5f, 0.0f
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
        3 * sizeof(float),
        nullptr
    );

    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    m_vertexCount = 3;
    m_indexCount = 0;
    m_indexed = false;

    return true;
}

bool Mesh::CreateCube() {
    Destroy();

    const float vertices[] = {
        // Front
        -0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,

        // Back
        -0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
        -0.5f,  0.5f, -0.5f
    };

    const unsigned int indices[] = {
        // Front
        0, 1, 2,
        2, 3, 0,

        // Back
        5, 4, 7,
        7, 6, 5,

        // Left
        4, 0, 3,
        3, 7, 4,

        // Right
        1, 5, 6,
        6, 2, 1,

        // Top
        3, 2, 6,
        6, 7, 3,

        // Bottom
        4, 5, 1,
        1, 0, 4
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

    glBindBuffer(
        GL_ELEMENT_ARRAY_BUFFER,
        m_ebo
    );

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
        3 * sizeof(float),
        nullptr
    );

    glEnableVertexAttribArray(0);

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

    // 큐브가 halfExtents=0.5짜리 단위 큐브인 것과 같은 관례로,
    // 반지름 0.5짜리 단위 구를 만든다(콜라이더 반지름 r은 스케일
    // 2r로 이 메시를 그리면 맞아떨어진다).
    const float radius = 0.5f;

    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    for (int ring = 0; ring <= rings; ++ring) {
        const float v = static_cast<float>(ring) / static_cast<float>(rings);
        const float phi = v * 3.14159265358979f; // 0..pi

        const float y = std::cos(phi);
        const float ringRadius = std::sin(phi);

        for (int segment = 0; segment <= segments; ++segment) {
            const float u = static_cast<float>(segment) / static_cast<float>(segments);
            const float theta = u * 2.0f * 3.14159265358979f;

            const float x = ringRadius * std::cos(theta);
            const float z = ringRadius * std::sin(theta);

            vertices.push_back(x * radius);
            vertices.push_back(y * radius);
            vertices.push_back(z * radius);
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
        static_cast<long>(vertices.size() * sizeof(float)),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glBindBuffer(
        GL_ELEMENT_ARRAY_BUFFER,
        m_ebo
    );

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
        3 * sizeof(float),
        nullptr
    );

    glEnableVertexAttribArray(0);

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