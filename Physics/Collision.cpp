#include "Collision.h"

#include "Contact.h"
#include "RigidBody.h"
#include "../Core/Debug/Logger.h"

#include <cmath>
#include <algorithm>

namespace {
    constexpr float Epsilon = 0.000001f;
    constexpr float AxisTieEpsilon = 0.0001f;
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

        if (overlap < minimumPenetration - AxisTieEpsilon) {
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

            const Vec3 contactPosition =
                point -
                referenceNormal * distance;

            Logger::Debug(
                "[MANIFOLD] "
                "REF=" + std::to_string(referenceAxisIndex) +
                " INC=" + std::to_string(incidentAxisIndex) +
                " POINT=(" +
                std::to_string(contactPosition.x) + "," +
                std::to_string(contactPosition.y) + "," +
                std::to_string(contactPosition.z) +
                ") "
                "PEN=" + std::to_string(pointPenetration)
            );

            contact.AddPoint(
                contactPosition,
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

    const Vec3 contactPosition =
        (pointA + pointB) * 0.5f;

    contact.AddPoint(
        contactPosition,
        penetration
    );

    return true;
}