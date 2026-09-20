#include "Collision.h"

#include "Contact.h"
#include "RigidBody.h"
#include "../Core/Debug/Logger.h"

#include <cmath>
#include <algorithm>

namespace {
    constexpr float Epsilon = 0.000001f;
}

bool Collision::CheckBoxBox(
    const Transform& transformA,
    const Vec3& halfExtentsA,
    RigidBody* bodyA,
    const Transform& transformB,
    const Vec3& halfExtentsB,
    RigidBody* bodyB,
    Contact& contact
) {
    const Mat3 rotationA =
        transformA.rotation.ToMat3();

    const Mat3 rotationB =
        transformB.rotation.ToMat3();

    const Vec3 axesA[3] = {
        Vec3(
            rotationA.m[0][0],
            rotationA.m[1][0],
            rotationA.m[2][0]
        ).Normalized(),

        Vec3(
            rotationA.m[0][1],
            rotationA.m[1][1],
            rotationA.m[2][1]
        ).Normalized(),

        Vec3(
            rotationA.m[0][2],
            rotationA.m[1][2],
            rotationA.m[2][2]
        ).Normalized()
    };

    const Vec3 axesB[3] = {
        Vec3(
            rotationB.m[0][0],
            rotationB.m[1][0],
            rotationB.m[2][0]
        ).Normalized(),

        Vec3(
            rotationB.m[0][1],
            rotationB.m[1][1],
            rotationB.m[2][1]
        ).Normalized(),

        Vec3(
            rotationB.m[0][2],
            rotationB.m[1][2],
            rotationB.m[2][2]
        ).Normalized()
    };

    const Vec3 centerDifference =
        transformB.position -
        transformA.position;

    float minimumPenetration = INFINITY;

    Vec3 collisionNormal(
        0.0f,
        1.0f,
        0.0f
    );

    int bestType = -1;
    int bestIndexA = -1;
    int bestIndexB = -1;

    // 축 선택 편향(tie-break epsilon)을 물체 크기에 비례해서 잡는다.
    // 고정값(예: 0.0001)은 아주 작은 물체(포인트 매스에 가까운)나
    // 아주 큰 물체에서는 상대적으로 너무 작거나 너무 커서, 완전히
    // 안정적으로 맞닿아 있는(면-면) 접촉인데도 부동소수점 잡음만으로
    // 모서리-모서리 축이 근소하게 이겨버리는 순간이 생긴다 — 그러면
    // 매니폴드가 4점에서 1점으로 붕괴하면서 토크가 한 점에 쏠려
    // 멀쩡히 균형 잡힌 물체가 서서히 넘어지게 된다.
    const float sizeScale =
        halfExtentsA.x + halfExtentsA.y + halfExtentsA.z +
        halfExtentsB.x + halfExtentsB.y + halfExtentsB.z;

    const float axisTieEpsilon =
        std::max(0.0001f, sizeScale * 0.0005f);

    auto TestAxis =
        [&](const Vec3& axis, int type, int indexA, int indexB) -> bool {
        if (axis.LengthSquared() <= Epsilon)
            return true;

        const Vec3 normalizedAxis =
            axis.Normalized();

        float overlap = 0.0f;

        if (!OverlapOnAxis(
            normalizedAxis,
            centerDifference,
            axesA,
            halfExtentsA,
            axesB,
            halfExtentsB,
            overlap
        )) {
            return false;
        }

        if (overlap < minimumPenetration - axisTieEpsilon) {
            minimumPenetration = overlap;
            collisionNormal = normalizedAxis;

            if (collisionNormal.Dot(centerDifference) < 0.0f)
                collisionNormal = -collisionNormal;

            bestType = type;
            bestIndexA = indexA;
            bestIndexB = indexB;
        }

        return true;
        };

    // A face normals
    for (int i = 0; i < 3; ++i) {
        if (!TestAxis(axesA[i], 0, i, -1))
            return false;
    }

    // B face normals
    for (int i = 0; i < 3; ++i) {
        if (!TestAxis(axesB[i], 1, -1, i))
            return false;
    }

    // Edge cross products
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Vec3 axis =
                axesA[i].Cross(axesB[j]);

            if (!TestAxis(axis, 2, i, j))
                return false;
        }
    }

    Logger::Debug(
        "[COLLISION] "
        "TYPE=" + std::to_string(bestType) +
        " AXIS_A=" + std::to_string(bestIndexA) +
        " AXIS_B=" + std::to_string(bestIndexB) +
        " PEN=" + std::to_string(minimumPenetration) +
        " N=(" +
        std::to_string(collisionNormal.x) + "," +
        std::to_string(collisionNormal.y) + "," +
        std::to_string(collisionNormal.z) +
        ")"
    );

    contact.SetBodies(bodyA, bodyB);
    contact.SetNormal(collisionNormal);
    contact.ClearPoints();

    // Face-face contact.
    if (bestType == 0) {
        BuildFaceManifold(
            transformA,
            halfExtentsA,
            bestIndexA,
            collisionNormal,
            transformB,
            halfExtentsB,
            true,
            contact,
            minimumPenetration
        );
    }
    else if (bestType == 1) {
        // Reference face is on B.
        // Its outward normal must point from B toward A.
        BuildFaceManifold(
            transformB,
            halfExtentsB,
            bestIndexB,
            -collisionNormal,
            transformA,
            halfExtentsA,
            false,
            contact,
            minimumPenetration
        );
    }
    else {
        BuildEdgeContact(
            transformA,
            halfExtentsA,
            transformB,
            halfExtentsB,
            bestIndexA,
            bestIndexB,
            collisionNormal,
            contact,
            minimumPenetration
        );
    }

    return contact.GetPointCount() > 0;
}

bool Collision::OverlapOnAxis(
    const Vec3& axis,
    const Vec3& centerDifference,
    const Vec3 axesA[3],
    const Vec3& halfExtentsA,
    const Vec3 axesB[3],
    const Vec3& halfExtentsB,
    float& overlap
) {
    const float distance =
        std::fabs(centerDifference.Dot(axis));

    const float radiusA =
        halfExtentsA.x * std::fabs(axesA[0].Dot(axis)) +
        halfExtentsA.y * std::fabs(axesA[1].Dot(axis)) +
        halfExtentsA.z * std::fabs(axesA[2].Dot(axis));

    const float radiusB =
        halfExtentsB.x * std::fabs(axesB[0].Dot(axis)) +
        halfExtentsB.y * std::fabs(axesB[1].Dot(axis)) +
        halfExtentsB.z * std::fabs(axesB[2].Dot(axis));

    overlap =
        radiusA +
        radiusB -
        distance;

    return overlap >= -Epsilon;
}

Collision::Face Collision::GetFace(
    const Transform& transform,
    const Vec3& halfExtents,
    int axisIndex,
    const Vec3& normal
) {
    const Mat3 rotation =
        transform.rotation.ToMat3();

    const Vec3 axes[3] = {
        Vec3(
            rotation.m[0][0],
            rotation.m[1][0],
            rotation.m[2][0]
        ).Normalized(),

        Vec3(
            rotation.m[0][1],
            rotation.m[1][1],
            rotation.m[2][1]
        ).Normalized(),

        Vec3(
            rotation.m[0][2],
            rotation.m[1][2],
            rotation.m[2][2]
        ).Normalized()
    };

    Face face;

    face.normal = normal;

    const float extent =
        axisIndex == 0 ? halfExtents.x :
        axisIndex == 1 ? halfExtents.y :
        halfExtents.z;

    const float sign =
        axes[axisIndex].Dot(normal) >= 0.0f
        ? 1.0f
        : -1.0f;

    face.center =
        transform.position +
        axes[axisIndex] * extent * sign;

    int tangentIndex1 = (axisIndex + 1) % 3;
    int tangentIndex2 = (axisIndex + 2) % 3;

    face.tangent1 = axes[tangentIndex1];
    face.tangent2 = axes[tangentIndex2];

    face.extent1 =
        tangentIndex1 == 0 ? halfExtents.x :
        tangentIndex1 == 1 ? halfExtents.y :
        halfExtents.z;

    face.extent2 =
        tangentIndex2 == 0 ? halfExtents.x :
        tangentIndex2 == 1 ? halfExtents.y :
        halfExtents.z;

    return face;
}

void Collision::GetFaceVertices(
    const Face& face,
    Vec3 vertices[4]
) {
    const Vec3 tangent1 =
        face.tangent1 * face.extent1;

    const Vec3 tangent2 =
        face.tangent2 * face.extent2;

    vertices[0] =
        face.center + tangent1 + tangent2;

    vertices[1] =
        face.center - tangent1 + tangent2;

    vertices[2] =
        face.center - tangent1 - tangent2;

    vertices[3] =
        face.center + tangent1 - tangent2;
}

int Collision::ClipPolygonAgainstPlane(
    const Vec3* input,
    int inputCount,
    Vec3* output,
    const Vec3& planeNormal,
    float planeOffset
) {
    if (inputCount == 0)
        return 0;

    int outputCount = 0;

    for (int i = 0; i < inputCount; ++i) {
        const Vec3& current = input[i];
        const Vec3& previous =
            input[(i + inputCount - 1) % inputCount];

        const float currentDistance =
            current.Dot(planeNormal) - planeOffset;

        const float previousDistance =
            previous.Dot(planeNormal) - planeOffset;

        const bool currentInside =
            currentDistance <= Epsilon;

        const bool previousInside =
            previousDistance <= Epsilon;

        if (currentInside != previousInside) {
            const float denominator =
                previousDistance - currentDistance;

            if (std::fabs(denominator) > Epsilon) {
                const float t =
                    previousDistance / denominator;

                output[outputCount++] =
                    previous +
                    (current - previous) * t;
            }
        }

        if (currentInside)
            output[outputCount++] = current;
    }

    return outputCount;
}

bool Collision::BuildFaceManifold(
    const Transform& referenceTransform,
    const Vec3& referenceHalfExtents,
    int referenceAxisIndex,
    const Vec3& referenceNormal,
    const Transform& incidentTransform,
    const Vec3& incidentHalfExtents,
    bool referenceIsBodyA,
    Contact& contact,
    float penetration
) {
    const Mat3 referenceRotation =
        referenceTransform.rotation.ToMat3();

    const Mat3 incidentRotation =
        incidentTransform.rotation.ToMat3();

    const Vec3 referenceAxes[3] = {
        Vec3(
            referenceRotation.m[0][0],
            referenceRotation.m[1][0],
            referenceRotation.m[2][0]
        ).Normalized(),

        Vec3(
            referenceRotation.m[0][1],
            referenceRotation.m[1][1],
            referenceRotation.m[2][1]
        ).Normalized(),

        Vec3(
            referenceRotation.m[0][2],
            referenceRotation.m[1][2],
            referenceRotation.m[2][2]
        ).Normalized()
    };

    const Vec3 incidentAxes[3] = {
        Vec3(
            incidentRotation.m[0][0],
            incidentRotation.m[1][0],
            incidentRotation.m[2][0]
        ).Normalized(),

        Vec3(
            incidentRotation.m[0][1],
            incidentRotation.m[1][1],
            incidentRotation.m[2][1]
        ).Normalized(),

        Vec3(
            incidentRotation.m[0][2],
            incidentRotation.m[1][2],
            incidentRotation.m[2][2]
        ).Normalized()
    };

    const Face referenceFace =
        GetFace(
            referenceTransform,
            referenceHalfExtents,
            referenceAxisIndex,
            referenceNormal
        );

    // Pick the incident face whose normal is most opposite
    // to the reference face normal.
    int incidentAxisIndex = 0;
    float minimumDot =
        std::fabs(
            incidentAxes[0].Dot(referenceNormal)
        );

    for (int i = 1; i < 3; ++i) {
        const float dot =
            std::fabs(
                incidentAxes[i].Dot(referenceNormal)
            );

        if (dot > minimumDot) {
            minimumDot = dot;
            incidentAxisIndex = i;
        }
    }

    Vec3 incidentNormal =
        incidentAxes[incidentAxisIndex];

    if (incidentNormal.Dot(referenceNormal) > 0.0f)
        incidentNormal = -incidentNormal;

    const Face incidentFace =
        GetFace(
            incidentTransform,
            incidentHalfExtents,
            incidentAxisIndex,
            incidentNormal
        );

    Vec3 polygonA[8];
    Vec3 polygonB[8];

    GetFaceVertices(
        incidentFace,
        polygonA
    );

    int polygonCount = 4;

    const Vec3 tangent1 =
        referenceFace.tangent1;

    const Vec3 tangent2 =
        referenceFace.tangent2;

    const float plane1 =
        (referenceFace.center +
            tangent1 * referenceFace.extent1).Dot(tangent1);

    const float plane2 =
        (referenceFace.center -
            tangent1 * referenceFace.extent1).Dot(-tangent1);

    const float plane3 =
        (referenceFace.center +
            tangent2 * referenceFace.extent2).Dot(tangent2);

    const float plane4 =
        (referenceFace.center -
            tangent2 * referenceFace.extent2).Dot(-tangent2);

    polygonCount =
        ClipPolygonAgainstPlane(
            polygonA,
            polygonCount,
            polygonB,
            tangent1,
            plane1
        );

    if (polygonCount == 0)
        return false;

    polygonCount =
        ClipPolygonAgainstPlane(
            polygonB,
            polygonCount,
            polygonA,
            -tangent1,
            plane2
        );

    if (polygonCount == 0)
        return false;

    polygonCount =
        ClipPolygonAgainstPlane(
            polygonA,
            polygonCount,
            polygonB,
            tangent2,
            plane3
        );

    if (polygonCount == 0)
        return false;

    polygonCount =
        ClipPolygonAgainstPlane(
            polygonB,
            polygonCount,
            polygonA,
            -tangent2,
            plane4
        );

    if (polygonCount == 0)
        return false;

    const float referencePlane =
        referenceFace.center.Dot(referenceNormal);

    for (int i = 0; i < polygonCount; ++i) {
        const Vec3& point =
            polygonA[i];

        const float distance =
            point.Dot(referenceNormal) -
            referencePlane;

        if (distance <= Epsilon) {
            const float pointPenetration =
                -distance;

            // point       = 충돌면(incident) 위의 실제 표면점
            // contactPosition = 그 점을 참조면(reference)에 투영한, 참조 바디의 실제 표면점
            // 이 둘은 서로 다른 점이며 정확히 penetration만큼 떨어져 있다.
            const Vec3 referenceSurfacePoint =
                point -
                referenceNormal * distance;

            const Vec3& incidentSurfacePoint =
                point;

            const Vec3& pointOnA =
                referenceIsBodyA
                ? referenceSurfacePoint
                : incidentSurfacePoint;

            const Vec3& pointOnB =
                referenceIsBodyA
                ? incidentSurfacePoint
                : referenceSurfacePoint;

            Logger::Debug(
                "[MANIFOLD] "
                "REF=" + std::to_string(referenceAxisIndex) +
                " INC=" + std::to_string(incidentAxisIndex) +
                " POINT=(" +
                std::to_string(referenceSurfacePoint.x) + "," +
                std::to_string(referenceSurfacePoint.y) + "," +
                std::to_string(referenceSurfacePoint.z) +
                ") "
                "PEN=" + std::to_string(pointPenetration)
            );

            contact.AddPoint(
                pointOnA,
                pointOnB,
                std::min(
                    pointPenetration,
                    penetration
                )
            );
        }

        if (contact.GetPointCount() >= Contact::MaxPoints)
            break;
    }

    return contact.GetPointCount() > 0;
}

void Collision::GetEdge(
    const Transform& transform,
    const Vec3& halfExtents,
    int edgeAxis,
    const Vec3& direction,
    Vec3& start,
    Vec3& end
) {
    const Mat3 rotation =
        transform.rotation.ToMat3();

    const Vec3 axes[3] = {
        Vec3(
            rotation.m[0][0],
            rotation.m[1][0],
            rotation.m[2][0]
        ).Normalized(),

        Vec3(
            rotation.m[0][1],
            rotation.m[1][1],
            rotation.m[2][1]
        ).Normalized(),

        Vec3(
            rotation.m[0][2],
            rotation.m[1][2],
            rotation.m[2][2]
        ).Normalized()
    };

    Vec3 center =
        transform.position;

    for (int i = 0; i < 3; ++i) {
        if (i == edgeAxis)
            continue;

        const float extent =
            i == 0 ? halfExtents.x :
            i == 1 ? halfExtents.y :
            halfExtents.z;

        const float sign =
            axes[i].Dot(direction) >= 0.0f
            ? 1.0f
            : -1.0f;

        center += axes[i] * extent * sign;
    }

    const float edgeExtent =
        edgeAxis == 0 ? halfExtents.x :
        edgeAxis == 1 ? halfExtents.y :
        halfExtents.z;

    start =
        center -
        axes[edgeAxis] * edgeExtent;

    end =
        center +
        axes[edgeAxis] * edgeExtent;
}

void Collision::ClosestPointsOnSegments(
    const Vec3& p1,
    const Vec3& q1,
    const Vec3& p2,
    const Vec3& q2,
    Vec3& point1,
    Vec3& point2
) {
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;

    const float a = d1.Dot(d1);
    const float e = d2.Dot(d2);
    const float f = d2.Dot(r);

    float s = 0.0f;
    float t = 0.0f;

    if (a <= Epsilon && e <= Epsilon) {
        point1 = p1;
        point2 = p2;
        return;
    }

    if (a <= Epsilon) {
        s = 0.0f;
        t = f / e;
        t = std::max(0.0f, std::min(1.0f, t));
    }
    else {
        const float c = d1.Dot(r);

        if (e <= Epsilon) {
            t = 0.0f;
            s = -c / a;
            s = std::max(0.0f, std::min(1.0f, s));
        }
        else {
            const float b = d1.Dot(d2);
            const float denominator =
                a * e - b * b;

            if (denominator > Epsilon)
                s = (b * f - c * e) / denominator;
            else
                s = 0.0f;

            s = std::max(0.0f, std::min(1.0f, s));

            t = (b * s + f) / e;

            if (t < 0.0f) {
                t = 0.0f;
                s = -c / a;
                s = std::max(0.0f, std::min(1.0f, s));
            }
            else if (t > 1.0f) {
                t = 1.0f;
                s = (b - c) / a;
                s = std::max(0.0f, std::min(1.0f, s));
            }
        }
    }

    point1 =
        p1 + d1 * s;

    point2 =
        p2 + d2 * t;
}

bool Collision::BuildEdgeContact(
    const Transform& transformA,
    const Vec3& halfExtentsA,
    const Transform& transformB,
    const Vec3& halfExtentsB,
    int edgeAxisA,
    int edgeAxisB,
    const Vec3& normal,
    Contact& contact,
    float penetration
) {
    Vec3 startA;
    Vec3 endA;
    Vec3 startB;
    Vec3 endB;

    GetEdge(
        transformA,
        halfExtentsA,
        edgeAxisA,
        -normal,
        startA,
        endA
    );

    GetEdge(
        transformB,
        halfExtentsB,
        edgeAxisB,
        normal,
        startB,
        endB
    );

    Vec3 pointA;
    Vec3 pointB;

    ClosestPointsOnSegments(
        startA,
        endA,
        startB,
        endB,
        pointA,
        pointB
    );

    contact.AddPoint(
        pointA,
        pointB,
        penetration
    );

    return true;
}

bool Collision::CheckSphereSphere(
    const Transform& transformA,
    float radiusA,
    RigidBody* bodyA,
    const Transform& transformB,
    float radiusB,
    RigidBody* bodyB,
    Contact& contact
) {
    const Vec3 delta =
        transformB.position -
        transformA.position;

    const float distanceSquared =
        delta.Dot(delta);

    const float radiusSum =
        radiusA + radiusB;

    if (distanceSquared >=
        radiusSum * radiusSum) {
        return false;
    }

    const float distance =
        std::sqrt(distanceSquared);

    Vec3 normal(0.0f, 1.0f, 0.0f);

    if (distance > Epsilon)
        normal = delta * (1.0f / distance);

    const float penetration =
        radiusSum - distance;

    const Vec3 pointOnA =
        transformA.position +
        normal * radiusA;

    const Vec3 pointOnB =
        transformB.position -
        normal * radiusB;

    contact.SetBodies(bodyA, bodyB);
    contact.SetNormal(normal);
    contact.ClearPoints();

    contact.AddPoint(
        pointOnA,
        pointOnB,
        penetration
    );

    return true;
}

bool Collision::CheckSphereBox(
    const Transform& sphereTransform,
    float sphereRadius,
    RigidBody* sphereBody,
    const Transform& boxTransform,
    const Vec3& boxHalfExtents,
    RigidBody* boxBody,
    bool sphereIsBodyA,
    Contact& contact
) {
    const Mat3 boxRotation =
        boxTransform.rotation.ToMat3();

    const Vec3 boxAxes[3] = {
        Vec3(boxRotation.m[0][0], boxRotation.m[1][0], boxRotation.m[2][0]).Normalized(),
        Vec3(boxRotation.m[0][1], boxRotation.m[1][1], boxRotation.m[2][1]).Normalized(),
        Vec3(boxRotation.m[0][2], boxRotation.m[1][2], boxRotation.m[2][2]).Normalized()
    };

    const Vec3 delta =
        sphereTransform.position -
        boxTransform.position;

    // 구 중심을 박스 로컬 프레임으로.
    const Vec3 localCenter(
        delta.Dot(boxAxes[0]),
        delta.Dot(boxAxes[1]),
        delta.Dot(boxAxes[2])
    );

    const float halfArr[3] = {
        boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z
    };

    const float localArr[3] = {
        localCenter.x, localCenter.y, localCenter.z
    };

    float clampedArr[3];
    bool insideBox = true;

    for (int i = 0; i < 3; ++i) {
        clampedArr[i] = localArr[i];

        if (clampedArr[i] > halfArr[i]) {
            clampedArr[i] = halfArr[i];
            insideBox = false;
        }
        else if (clampedArr[i] < -halfArr[i]) {
            clampedArr[i] = -halfArr[i];
            insideBox = false;
        }
    }

    Vec3 closestLocal;
    Vec3 boxOutwardNormalLocal(0.0f, 0.0f, 0.0f);
    float penetration = 0.0f;

    if (!insideBox) {
        // 구 중심이 박스 바깥에 있는 일반적인 경우: 박스 표면에서 가장
        // 가까운 점과 구 중심 사이의 거리로 판정한다.
        closestLocal = Vec3(clampedArr[0], clampedArr[1], clampedArr[2]);

        const Vec3 diff = localCenter - closestLocal;
        const float distanceSquared = diff.Dot(diff);

        if (distanceSquared >= sphereRadius * sphereRadius)
            return false;

        const float distance = std::sqrt(distanceSquared);

        boxOutwardNormalLocal =
            distance > Epsilon
            ? diff * (1.0f / distance)
            : Vec3(0.0f, 1.0f, 0.0f);

        penetration = sphereRadius - distance;
    }
    else {
        // 구 중심이 박스 안에 완전히 박혀 있는 경우: 가장 얕은 면으로
        // 밀어낸다(6면 중 침투가 가장 적은 축/방향).
        float bestPenetration = -INFINITY;
        int bestAxis = 0;
        float bestSign = 1.0f;

        for (int i = 0; i < 3; ++i) {
            const float distToPositive = halfArr[i] - localArr[i];
            const float distToNegative = halfArr[i] + localArr[i];

            if (distToPositive > bestPenetration) {
                bestPenetration = distToPositive;
                bestAxis = i;
                bestSign = 1.0f;
            }

            if (distToNegative > bestPenetration) {
                bestPenetration = distToNegative;
                bestAxis = i;
                bestSign = -1.0f;
            }
        }

        closestLocal = Vec3(localArr[0], localArr[1], localArr[2]);

        Vec3 axisVec(0.0f, 0.0f, 0.0f);
        if (bestAxis == 0) axisVec = Vec3(1.0f, 0.0f, 0.0f);
        else if (bestAxis == 1) axisVec = Vec3(0.0f, 1.0f, 0.0f);
        else axisVec = Vec3(0.0f, 0.0f, 1.0f);

        boxOutwardNormalLocal = axisVec * bestSign;
        penetration = sphereRadius + bestPenetration;
    }

    const Vec3 boxOutwardNormal =
        boxAxes[0] * boxOutwardNormalLocal.x +
        boxAxes[1] * boxOutwardNormalLocal.y +
        boxAxes[2] * boxOutwardNormalLocal.z;

    const Vec3 closestWorld =
        boxTransform.position +
        boxAxes[0] * closestLocal.x +
        boxAxes[1] * closestLocal.y +
        boxAxes[2] * closestLocal.z;

    const Vec3 pointOnSphere =
        sphereTransform.position -
        boxOutwardNormal * sphereRadius;

    // 최종 normal은 항상 "A -> B" 방향이어야 한다.
    // boxOutwardNormal은 박스 -> 구 방향이다.
    RigidBody* finalBodyA = sphereIsBodyA ? sphereBody : boxBody;
    RigidBody* finalBodyB = sphereIsBodyA ? boxBody : sphereBody;

    const Vec3 finalNormal =
        sphereIsBodyA ? -boxOutwardNormal : boxOutwardNormal;

    const Vec3 pointOnA =
        sphereIsBodyA ? pointOnSphere : closestWorld;

    const Vec3 pointOnB =
        sphereIsBodyA ? closestWorld : pointOnSphere;

    contact.SetBodies(finalBodyA, finalBodyB);
    contact.SetNormal(finalNormal);
    contact.ClearPoints();

    contact.AddPoint(
        pointOnA,
        pointOnB,
        penetration
    );

    return true;
}
