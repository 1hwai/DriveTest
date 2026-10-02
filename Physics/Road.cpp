#include "Road.h"

#include "Terrain.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr float Epsilon = 0.000001f;
    constexpr int SamplesPerSegment = 16;

    Vec3 CatmullRom(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t) {
        const float t2 = t * t;
        const float t3 = t2 * t;
        return (p1 * 2.0f +
            (p2 - p0) * t +
            (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 +
            (-p0 + p1 * 3.0f - p2 * 3.0f + p3) * t3) * 0.5f;
    }

    bool IntersectTriangle(const Vec3& origin, const Vec3& direction, const Vec3& a, const Vec3& b, const Vec3& c, float maxDistance, float& distance, Vec3& normal) {
        const Vec3 edge1 = b - a;
        const Vec3 edge2 = c - a;
        const Vec3 p = direction.Cross(edge2);
        const float determinant = edge1.Dot(p);

        if (std::fabs(determinant) <= Epsilon)
            return false;

        const float inverseDeterminant = 1.0f / determinant;
        const Vec3 offset = origin - a;
        const float u = offset.Dot(p) * inverseDeterminant;

        if (u < 0.0f || u > 1.0f)
            return false;

        const Vec3 q = offset.Cross(edge1);
        const float v = direction.Dot(q) * inverseDeterminant;

        if (v < 0.0f || u + v > 1.0f)
            return false;

        const float t = edge2.Dot(q) * inverseDeterminant;

        if (t < 0.0f || t > maxDistance)
            return false;

        distance = t;
        normal = edge1.Cross(edge2).Normalized();

        if (normal.y < 0.0f)
            normal = -normal;

        return true;
    }
}

bool Road::GenerateTestCourse(const Terrain& terrain, float width, float surfaceOffset) {
    m_leftEdge.clear();
    m_rightEdge.clear();

    if (width <= 0.0f)
        return false;

    const std::vector<Vec3> controlPoints = {
        Vec3(0.0f, 0.0f, -300.0f),
        Vec3(0.0f, 0.0f, -150.0f),
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 0.2f, 100.0f),
        Vec3(35.0f, 1.5f, 220.0f),
        Vec3(-35.0f, 3.0f, 340.0f),
        Vec3(0.0f, 1.0f, 460.0f),
        Vec3(0.0f, 0.0f, 600.0f),
        Vec3(0.0f, 0.0f, 750.0f)
    };

    std::vector<Vec3> centerline;
    centerline.reserve((controlPoints.size() - 1) * SamplesPerSegment + 1);

    for (size_t segment = 0; segment + 1 < controlPoints.size(); ++segment) {
        const Vec3& p0 = controlPoints[segment == 0 ? segment : segment - 1];
        const Vec3& p1 = controlPoints[segment];
        const Vec3& p2 = controlPoints[segment + 1];
        const Vec3& p3 = controlPoints[std::min(segment + 2, controlPoints.size() - 1)];

        for (int sample = 0; sample < SamplesPerSegment; ++sample) {
            const float t = static_cast<float>(sample) / SamplesPerSegment;
            Vec3 point = CatmullRom(p0, p1, p2, p3, t);
            point.y = terrain.GetHeight(point.x, point.z) + surfaceOffset;

            if (!std::isfinite(point.y))
                return false;

            centerline.push_back(point);
        }
    }

    Vec3 finalPoint = controlPoints.back();
    finalPoint.y = terrain.GetHeight(finalPoint.x, finalPoint.z) + surfaceOffset;

    if (!std::isfinite(finalPoint.y))
        return false;

    centerline.push_back(finalPoint);

    const float halfWidth = width * 0.5f;
    m_leftEdge.reserve(centerline.size());
    m_rightEdge.reserve(centerline.size());

    for (size_t i = 0; i < centerline.size(); ++i) {
        const Vec3& previous = centerline[i == 0 ? i : i - 1];
        const Vec3& next = centerline[std::min(i + 1, centerline.size() - 1)];
        Vec3 tangent(next.x - previous.x, 0.0f, next.z - previous.z);

        if (tangent.LengthSquared() <= Epsilon)
            return false;

        tangent = tangent.Normalized();
        const Vec3 right(tangent.z, 0.0f, -tangent.x);
        Vec3 leftPoint = centerline[i] - right * halfWidth;
        Vec3 rightPoint = centerline[i] + right * halfWidth;
        leftPoint.y = terrain.GetHeight(leftPoint.x, leftPoint.z) + surfaceOffset;
        rightPoint.y = terrain.GetHeight(rightPoint.x, rightPoint.z) + surfaceOffset;

        if (!std::isfinite(leftPoint.y) || !std::isfinite(rightPoint.y))
            return false;

        m_leftEdge.push_back(leftPoint);
        m_rightEdge.push_back(rightPoint);
    }

    return m_leftEdge.size() >= 2;
}

const std::vector<Vec3>& Road::GetLeftEdge() const {
    return m_leftEdge;
}

const std::vector<Vec3>& Road::GetRightEdge() const {
    return m_rightEdge;
}

size_t Road::GetSegmentCount() const {
    return m_leftEdge.size() > 1 ? m_leftEdge.size() - 1 : 0;
}

bool Road::Raycast(const Vec3& origin, const Vec3& direction, float maxDistance, float& distance, Vec3& point, Vec3& normal) const {
    if (direction.LengthSquared() <= Epsilon || maxDistance < 0.0f)
        return false;

    bool hit = false;
    float closestDistance = maxDistance;
    Vec3 closestNormal;

    for (size_t i = 0; i < GetSegmentCount(); ++i) {
        float candidateDistance;
        Vec3 candidateNormal;

        if (IntersectTriangle(origin, direction, m_leftEdge[i], m_leftEdge[i + 1], m_rightEdge[i], closestDistance, candidateDistance, candidateNormal)) {
            hit = true;
            closestDistance = candidateDistance;
            closestNormal = candidateNormal;
        }

        if (IntersectTriangle(origin, direction, m_rightEdge[i], m_leftEdge[i + 1], m_rightEdge[i + 1], closestDistance, candidateDistance, candidateNormal)) {
            hit = true;
            closestDistance = candidateDistance;
            closestNormal = candidateNormal;
        }
    }

    if (!hit)
        return false;

    distance = closestDistance;
    point = origin + direction * distance;
    normal = closestNormal;
    return true;
}
