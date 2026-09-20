#pragma once

class Mesh {
public:
    Mesh();
    ~Mesh();

    bool CreateTriangle();
    bool CreateCube();
    bool CreateSphere(int segments = 16, int rings = 12);

    void Draw() const;
    void Destroy();

private:
    unsigned int m_vao;
    unsigned int m_vbo;
    unsigned int m_ebo;

    unsigned int m_vertexCount;
    unsigned int m_indexCount;

    bool m_indexed;
};