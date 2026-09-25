#pragma once

#include <vector>
#include "../Core/Math/Vec3.h"

class Terrain {
public:
    Terrain(int resolution = 65, float size = 100.0f, float heightScale = 4.0f, float noiseScale = 0.035f, int octaves = 5, unsigned int seed = 1337);

    int GetResolution() const;
    float GetSize() const;
    const std::vector<float>& GetHeights() const;

    void GenerateSineWave(float amplitude, float phase);

    float GetHeight(float x, float z) const;
    Vec3 GetNormal(float x, float z) const;

    bool Raycast(const Vec3& origin, const Vec3& direction, float maxDistance, float& distance, Vec3& point, Vec3& normal) const;

private:
    float Fade(float t) const;
    float Lerp(float a, float b, float t) const;
    float Gradient(int ix, int iz, float x, float z) const;
    float SampleNoise(float x, float z) const;

    int m_resolution;
    float m_size;
    float m_heightScale;
    float m_noiseScale;
    int m_octaves;
    unsigned int m_seed;
    std::vector<float> m_heights;
};