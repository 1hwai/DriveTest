#pragma once

class Terrain;

class Mesh {
public:
    Mesh();
    ~Mesh();

    bool CreateTriangle();
    bool CreateCube();
    bool CreateSphere(int segments = 16, int rings = 12);
    bool CreateWheel(int segments = 32, int widthSegments = 8);
    bool CreateTerrain(const Terrain& terrain);

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