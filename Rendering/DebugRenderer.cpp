#include "DebugRenderer.h"

#include <glad/gl.h>
#include <cstddef>
#include <vector>

DebugRenderer::DebugRenderer()
    : m_vao(0),
    m_vbo(0),
    m_gridVertexCount(0),
    m_axisVertexCount(0) {}

DebugRenderer::~DebugRenderer() {
    Shutdown();
}

bool DebugRenderer::Initialize() {
    if (m_vao != 0)
        return false;

    constexpr float gridSize = 20.0f;
    constexpr float gridSpacing = 1.0f;
    constexpr float gridY = 0.0f;

    const float gridColor[3] = {0.22f, 0.22f, 0.22f};
    const float xAxisColor[3] = {0.85f, 0.20f, 0.20f};
    const float yAxisColor[3] = {0.25f, 0.85f, 0.25f};
    const float zAxisColor[3] = {0.20f, 0.45f, 0.90f};

    std::vector<Vertex> vertices;

    const int gridLineCount =
        static_cast<int>(gridSize * 2.0f / gridSpacing) + 1;

    vertices.reserve(gridLineCount * 4 + 6);

    for (int i = 0; i < gridLineCount; ++i) {
        const float coordinate =
            -gridSize + static_cast<float>(i) * gridSpacing;

        vertices.push_back({
            {coordinate, gridY, -gridSize},
            {gridColor[0], gridColor[1], gridColor[2]}
        });

        vertices.push_back({
            {coordinate, gridY, gridSize},
            {gridColor[0], gridColor[1], gridColor[2]}
        });

        vertices.push_back({
            {-gridSize, gridY, coordinate},
            {gridColor[0], gridColor[1], gridColor[2]}
        });

        vertices.push_back({
            {gridSize, gridY, coordinate},
            {gridColor[0], gridColor[1], gridColor[2]}
        });
    }

    m_gridVertexCount =
        static_cast<unsigned int>(vertices.size());

    vertices.push_back({
        {-gridSize, 0.0f, 0.0f},
        {xAxisColor[0], xAxisColor[1], xAxisColor[2]}
    });

    vertices.push_back({
        {gridSize, 0.0f, 0.0f},
        {xAxisColor[0], xAxisColor[1], xAxisColor[2]}
    });

    vertices.push_back({
        {0.0f, 0.0f, 0.0f},
        {yAxisColor[0], yAxisColor[1], yAxisColor[2]}
    });

    vertices.push_back({
        {0.0f, gridSize, 0.0f},
        {yAxisColor[0], yAxisColor[1], yAxisColor[2]}
    });

    vertices.push_back({
        {0.0f, 0.0f, -gridSize},
        {zAxisColor[0], zAxisColor[1], zAxisColor[2]}
    });

    vertices.push_back({
        {0.0f, 0.0f, gridSize},
        {zAxisColor[0], zAxisColor[1], zAxisColor[2]}
    });

    m_axisVertexCount = 6;

    if (!m_shader.Load(
        "Shaders/debug.vert",
        "Shaders/debug.frag")) {
        return false;
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    if (m_vao == 0 || m_vbo == 0) {
        Shutdown();
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
        reinterpret_cast<void*>(offsetof(Vertex, color))
    );
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}

void DebugRenderer::Render(const Camera& camera) {
    if (m_vao == 0)
        return;

    m_shader.Use();
    m_shader.SetMat4("uView", camera.GetViewMatrix());
    m_shader.SetMat4("uProjection", camera.GetProjectionMatrix());

    glBindVertexArray(m_vao);

    glDrawArrays(GL_LINES, 0, m_gridVertexCount);
    glDrawArrays(
        GL_LINES,
        m_gridVertexCount,
        m_axisVertexCount
    );

    glBindVertexArray(0);
}

void DebugRenderer::Shutdown() {
    if (m_vbo != 0) {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }

    if (m_vao != 0) {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }

    m_shader.Destroy();
    m_gridVertexCount = 0;
    m_axisVertexCount = 0;
}