#pragma once

class Mesh {
public:
    Mesh();
    ~Mesh();

    bool CreateTriangle();
    bool CreateCube();

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