#include "Terrain.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {
    constexpr float Epsilon = 0.000001f;

    unsigned int Hash(int x, int z, unsigned int seed) {
        std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u;
        h ^= static_cast<std::uint32_t>(z) * 668265263u;
        h ^= seed * 2246822519u;
        h ^= h >> 13;
        h *= 1274126177u;
        h ^= h >> 16;
        return h;
    }
}

Terrain::Terrain(int resolution, float size, float heightScale, float noiseScale, int octaves, unsigned int seed)
    : m_resolution(std::max(2, resolution)),
    m_size(std::max(1.0f, size)),
    m_heightScale(std::max(0.0f, heightScale)),
    m_noiseScale(std::max(0.0001f, noiseScale)),
    m_octaves(std::max(1, octaves)),
    m_seed(seed),
    m_heights(static_cast<size_t>(std::max(2, resolution)) * static_cast<size_t>(std::max(2, resolution)), 0.0f) {
    for (int z = 0; z < m_resolution; ++z) {
        for (int x = 0; x < m_resolution; ++x) {
            const float worldX = -m_size * 0.5f + m_size * static_cast<float>(x) / static_cast<float>(m_resolution - 1);
            const float worldZ = -m_size * 0.5f + m_size * static_cast<float>(z) / static_cast<float>(m_resolution - 1);

            m_heights[static_cast<size_t>(z) * m_resolution + x] =
                SampleNoise(worldX, worldZ) * m_heightScale;
        }
    }
}

int Terrain::GetResolution() const {
    return m_resolution;
}

float Terrain::GetSize() const {
    return m_size;
}

const std::vector<float>& Terrain::GetHeights() const {
    return m_heights;
}

float Terrain::Fade(float t) const {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float Terrain::Lerp(float a, float b, float t) const {
    return a + (b - a) * t;
}

float Terrain::Gradient(int ix, int iz, float x, float z) const {
    static const Vec3 gradients[] = {
        Vec3(1.0f, 0.0f, 1.0f),
        Vec3(-1.0f, 0.0f, 1.0f),
        Vec3(1.0f, 0.0f, -1.0f),
        Vec3(-1.0f, 0.0f, -1.0f)
    };

    const Vec3& gradient = gradients[Hash(ix, iz, m_seed) & 3u];

    return gradient.x * (x - static_cast<float>(ix)) +
        gradient.z * (z - static_cast<float>(iz));
}

float Terrain::SampleNoise(float x, float z) const {
    float value = 0.0f;
    float amplitude = 1.0f;
    float frequency = m_noiseScale;
    float amplitudeSum = 0.0f;

    for (int octave = 0; octave < m_octaves; ++octave) {
        const float sampleX = x * frequency;
        const float sampleZ = z * frequency;
        const int x0 = static_cast<int>(std::floor(sampleX));
        const int z0 = static_cast<int>(std::floor(sampleZ));
        const float tx = sampleX - static_cast<float>(x0);
        const float tz = sampleZ - static_cast<float>(z0);
        const float u = Fade(tx);
        const float v = Fade(tz);

        const float n00 = Gradient(x0, z0, sampleX, sampleZ);
        const float n10 = Gradient(x0 + 1, z0, sampleX, sampleZ);
        const float n01 = Gradient(x0, z0 + 1, sampleX, sampleZ);
        const float n11 = Gradient(x0 + 1, z0 + 1, sampleX, sampleZ);

        value += Lerp(Lerp(n00, n10, u), Lerp(n01, n11, u), v) * amplitude;
        amplitudeSum += amplitude;
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }

    if (amplitudeSum <= Epsilon)
        return 0.0f;

    return 0.5f + 0.5f * (value / amplitudeSum);
}

float Terrain::GetHeight(float x, float z) const {
    const float halfSize = m_size * 0.5f;

    if (x < -halfSize || x > halfSize || z < -halfSize || z > halfSize)
        return -std::numeric_limits<float>::infinity();

    const float gridScale = m_size / static_cast<float>(m_resolution - 1);
    const float gridX = (x + halfSize) / gridScale;
    const float gridZ = (z + halfSize) / gridScale;

    const int x0 = std::min(m_resolution - 2, static_cast<int>(std::floor(gridX)));
    const int z0 = std::min(m_resolution - 2, static_cast<int>(std::floor(gridZ)));

    const float tx = gridX - x0;
    const float tz = gridZ - z0;

    const auto HeightAt = [this](int ix, int iz) {
        return m_heights[static_cast<size_t>(iz) * m_resolution + ix];
    };

    const float h00 = HeightAt(x0, z0);
    const float h10 = HeightAt(x0 + 1, z0);
    const float h01 = HeightAt(x0, z0 + 1);
    const float h11 = HeightAt(x0 + 1, z0 + 1);

    return Lerp(Lerp(h00, h10, tx), Lerp(h01, h11, tx), tz);
}

Vec3 Terrain::GetNormal(float x, float z) const {
    const float cellSize = m_size / static_cast<float>(m_resolution - 1);
    const float center = GetHeight(x, z);
    const float left = GetHeight(x - cellSize, z);
    const float right = GetHeight(x + cellSize, z);
    const float down = GetHeight(x, z - cellSize);
    const float up = GetHeight(x, z + cellSize);

    const float dx =
        std::isfinite(left) && std::isfinite(right)
        ? (right - left) / (2.0f * cellSize)
        : 0.0f;

    const float dz =
        std::isfinite(down) && std::isfinite(up)
        ? (up - down) / (2.0f * cellSize)
        : 0.0f;

    return Vec3(-dx, 1.0f, -dz).Normalized();
}

bool Terrain::Raycast(const Vec3& origin, const Vec3& direction, float maxDistance, float& distance, Vec3& point, Vec3& normal) const {
    if (direction.LengthSquared() <= Epsilon || maxDistance < 0.0f)
        return false;

    const Vec3 dir = direction.Normalized();
    constexpr int Steps = 32;

    float previousT = 0.0f;
    float previousDelta = origin.y - GetHeight(origin.x, origin.z);

    if (std::isfinite(previousDelta) && previousDelta <= 0.0f) {
        distance = 0.0f;
        point = origin;
        normal = GetNormal(origin.x, origin.z);
        return true;
    }

    for (int i = 1; i <= Steps; ++i) {
        const float t = maxDistance * static_cast<float>(i) / static_cast<float>(Steps);
        const Vec3 sample = origin + dir * t;
        const float height = GetHeight(sample.x, sample.z);

        if (!std::isfinite(height))
            continue;

        const float currentDelta = sample.y - height;

        if (currentDelta > 0.0f || previousDelta <= 0.0f) {
            previousT = t;
            previousDelta = currentDelta;
            continue;
        }

        float low = previousT;
        float high = t;

        for (int iteration = 0; iteration < 8; ++iteration) {
            const float mid = (low + high) * 0.5f;
            const Vec3 midPoint = origin + dir * mid;
            const float midHeight = GetHeight(midPoint.x, midPoint.z);

            if (midPoint.y - midHeight > 0.0f)
                low = mid;
            else
                high = mid;
        }

        distance = high;
        point = origin + dir * distance;
        normal = GetNormal(point.x, point.z);
        return true;
    }

    return false;
}