#pragma once

#include "Shader.h"
#include "Camera.h"

class DebugRenderer {
public:
    DebugRenderer();
    ~DebugRenderer();

    bool Initialize();
    void Render(const Camera& camera);
    void Shutdown();

private:
    struct Vertex {
        float position[3];
        float color[3];
    };

    unsigned int m_vao;
    unsigned int m_vbo;
    unsigned int m_gridVertexCount;
    unsigned int m_axisVertexCount;

    Shader m_shader;
};