#pragma once

#include <vector>

#include "../Core/Math/Vec3.h"

class Terrain;

class Road {
public:
    bool GenerateTestCourse(const Terrain& terrain, float width, float surfaceOffset);

    const std::vector<Vec3>& GetLeftEdge() const;
    const std::vector<Vec3>& GetRightEdge() const;
    size_t GetSegmentCount() const;

    bool Raycast(const Vec3& origin, const Vec3& direction, float maxDistance, float& distance, Vec3& point, Vec3& normal) const;

private:
    std::vector<Vec3> m_leftEdge;
    std::vector<Vec3> m_rightEdge;
};
