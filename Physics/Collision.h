#pragma once

#include "../Core/Math/Vec3.h"
#include "../Core/Math/Mat3.h"
#include "../Core/Math/Transform.h"

class Contact;
class RigidBody;

class Collision {
public:
    static bool CheckBoxBox(
        const Transform& transformA,
        const Vec3& halfExtentsA,
        RigidBody* bodyA,
        const Transform& transformB,
        const Vec3& halfExtentsB,
        RigidBody* bodyB,
        Contact& contact
    );

private:
    struct BoxAxes {
        Vec3 axes[3];
    };

    struct Face {
        Vec3 center;
        Vec3 normal;
        Vec3 tangent1;
        Vec3 tangent2;
        float extent1;
        float extent2;
    };

    static bool OverlapOnAxis(
        const Vec3& axis,
        const Vec3& centerDifference,
        const Vec3 axesA[3],
        const Vec3& halfExtentsA,
        const Vec3 axesB[3],
        const Vec3& halfExtentsB,
        float& overlap
    );

    static Face GetFace(
        const Transform& transform,
        const Vec3& halfExtents,
        int axisIndex,
        const Vec3& normal
    );

    static void GetFaceVertices(
        const Face& face,
        Vec3 vertices[4]
    );

    static int ClipPolygonAgainstPlane(
        const Vec3* input,
        int inputCount,
        Vec3* output,
        const Vec3& planeNormal,
        float planeOffset
    );

    static bool BuildFaceManifold(
        const Transform& referenceTransform,
        const Vec3& referenceHalfExtents,
        int referenceAxisIndex,
        const Vec3& referenceNormal,
        const Transform& incidentTransform,
        const Vec3& incidentHalfExtents,
        Contact& contact,
        float penetration
    );

    static bool BuildEdgeContact(
        const Transform& transformA,
        const Vec3& halfExtentsA,
        const Transform& transformB,
        const Vec3& halfExtentsB,
        int edgeAxisA,
        int edgeAxisB,
        const Vec3& normal,
        Contact& contact,
        float penetration
    );

    static void GetEdge(
        const Transform& transform,
        const Vec3& halfExtents,
        int edgeAxis,
        const Vec3& direction,
        Vec3& start,
        Vec3& end
    );

    static void ClosestPointsOnSegments(
        const Vec3& p1,
        const Vec3& q1,
        const Vec3& p2,
        const Vec3& q2,
        Vec3& point1,
        Vec3& point2
    );
};