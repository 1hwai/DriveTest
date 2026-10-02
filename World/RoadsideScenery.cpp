#include "RoadsideScenery.h"

#include "../Core/Object.h"
#include "../Physics/Terrain.h"
#include "../Physics/Road.h"
#include "../Rendering/Mesh.h"
#include "../Rendering/MeshManager.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

std::vector<std::unique_ptr<Object>> RoadsideScenery::Create(
    MeshManager& meshManager,
    const Terrain& terrain,
    const Road& road
) {
    std::vector<std::unique_ptr<Object>> objects;
    if (!meshManager.CreateCylinder("cylinder") ||
        !meshManager.CreateDisc("sign_disc") ||
        !meshManager.CreateFoliageBillboard("tree_billboard") ||
        !meshManager.CreateFoliageBillboard("shrub_billboard", true))
        return {};

    Mesh* cubeMesh = meshManager.Get("cube");
    Mesh* cylinderMesh = meshManager.Get("cylinder");
    Mesh* treeMesh = meshManager.Get("tree_billboard");
    Mesh* shrubMesh = meshManager.Get("shrub_billboard");
    Mesh* signDiscMesh = meshManager.Get("sign_disc");

    const auto addScenery = [&objects](const std::string& name, Mesh* mesh,
        const Vec3& position, const Vec3& scale, const Vec3& color,
        const Quaternion& rotation = Quaternion(), bool billboard = false) {
        if (!mesh) return;
        auto object = std::make_unique<Object>();
        object->SetName(name);
        object->SetScenePersistent(false);
        object->SetMesh(mesh);
        object->SetColor(color);
        object->SetBillboard(billboard);
        object->GetTransform().position = position;
        object->GetTransform().scale = scale;
        object->GetTransform().rotation = rotation;
        objects.push_back(std::move(object));
    };

    const std::vector<Vec3>& leftEdge = road.GetLeftEdge();
    const std::vector<Vec3>& rightEdge = road.GetRightEdge();
    const Vec3 postColor(0.72f, 0.62f, 0.42f);
    const Vec3 white(0.95f, 0.95f, 0.9f);
    const Vec3 red(0.8f, 0.04f, 0.03f);

    for (size_t i = 0; i < leftEdge.size(); i += 2) {
        const Vec3 center((leftEdge[i].x + rightEdge[i].x) * 0.5f,
            (leftEdge[i].y + rightEdge[i].y) * 0.5f,
            (leftEdge[i].z + rightEdge[i].z) * 0.5f);
        Vec3 leftOutward = leftEdge[i] - center;
        Vec3 rightOutward = rightEdge[i] - center;
        leftOutward.y = 0.0f;
        rightOutward.y = 0.0f;
        if (leftOutward.LengthSquared() > 0.0001f) leftOutward = leftOutward.Normalized();
        if (rightOutward.LengthSquared() > 0.0001f) rightOutward = rightOutward.Normalized();

        for (int side = 0; side < 2; ++side) {
            const Vec3 outward = side == 0 ? leftOutward : rightOutward;
            const Vec3 edge = side == 0 ? leftEdge[i] : rightEdge[i];
            const float treeX = edge.x + outward.x * 8.0f;
            const float treeZ = edge.z + outward.z * 8.0f;
            const float treeGround = terrain.GetHeight(treeX, treeZ);
            if (!std::isfinite(treeGround)) continue;

            if (i % 6 == 0) {
                const float height = 9.0f + static_cast<float>((i / 6 + side * 2) % 5);
                addScenery("RoadsideTreeBillboard", treeMesh,
                    Vec3(treeX, treeGround, treeZ), Vec3(height * 0.58f, height, 1.0f),
                    white, Quaternion(), true);
            }

            if (i % 6 == 0) {
                const float shrubX = edge.x + outward.x * 3.8f;
                const float shrubZ = edge.z + outward.z * 3.8f;
                const float shrubGround = terrain.GetHeight(shrubX, shrubZ);
                if (std::isfinite(shrubGround)) {
                    const Vec3 shrubPosition(shrubX, shrubGround, shrubZ);
                    const Quaternion crossed = Quaternion::FromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), 1.57079632679f);
                    addScenery("RoadsideShrubA", shrubMesh, shrubPosition,
                        Vec3(2.2f, 1.25f, 1.0f), white);
                    addScenery("RoadsideShrubB", shrubMesh, shrubPosition,
                        Vec3(2.2f, 1.25f, 1.0f), white, crossed);
                }
            }

            if (i % 10 == 0) {
                const float postX = edge.x + outward.x * 0.9f;
                const float postZ = edge.z + outward.z * 0.9f;
                const float postGround = m_terrain->GetHeight(postX, postZ);
                if (std::isfinite(postGround))
                    addScenery("RoadsideWoodPost", cubeMesh,
                        Vec3(postX, postGround + 0.42f, postZ),
                        Vec3(0.12f, 0.85f, 0.12f), postColor);
            }

            // Exactly two speed-limit signs along the test route.
            if (side == 0 && ((i >= leftEdge.size() / 3 && i < leftEdge.size() / 3 + 2) || (i >= (leftEdge.size() * 2) / 3 && i < (leftEdge.size() * 2) / 3 + 2))) {
                const float signX = edge.x + outward.x * 2.0f;
                const float signZ = edge.z + outward.z * 2.0f;
                const float signGround = m_terrain->GetHeight(signX, signZ);
                if (!std::isfinite(signGround)) continue;
                const float yaw = std::atan2(-outward.x, -outward.z);
                const Quaternion signRotation = Quaternion::FromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), yaw);
                const Vec3 signBase(signX, signGround, signZ);
                const Vec3 signCenter = signBase + Vec3(0.0f, 2.15f, 0.0f);
                addScenery("SpeedLimitSignPole", cylinderMesh,
                    signBase + Vec3(0.0f, 1.05f, 0.0f), Vec3(0.10f, 2.1f, 0.10f), postColor);
                addScenery("SpeedLimitSignRedRim", signDiscMesh, signCenter,
                    Vec3(0.82f, 0.82f, 0.06f), red, signRotation);
                addScenery("SpeedLimitSignFace", signDiscMesh, signCenter - outward * 0.04f,
                    Vec3(0.68f, 0.68f, 0.06f), white, signRotation);

                const Vec3 digitCenter = signCenter - outward * 0.08f;
                const Vec3 digitColor(0.08f, 0.08f, 0.08f);
                const auto addDigitSegment = [&](float x, float y, bool horizontal) {
                    const Vec3 localPosition(x, y, 0.04f);
                    const Vec3 segmentScale = horizontal
                        ? Vec3(0.13f, 0.035f, 0.025f)
                        : Vec3(0.035f, 0.12f, 0.025f);
                    addScenery("SpeedLimitSignDigit", cubeMesh,
                        digitCenter + signRotation * localPosition,
                        segmentScale, digitColor, signRotation);
                };

                addDigitSegment(-0.13f, 0.15f, true);
                addDigitSegment(-0.20f, 0.075f, false);
                addDigitSegment(-0.13f, 0.0f, true);
                addDigitSegment(-0.20f, -0.075f, false);
                addDigitSegment(-0.13f, -0.15f, true);
                addDigitSegment(-0.06f, -0.075f, false);
                addDigitSegment(0.13f, 0.15f, true);
                addDigitSegment(0.06f, 0.075f, false);
                addDigitSegment(0.20f, 0.075f, false);
                addDigitSegment(0.06f, -0.075f, false);
                addDigitSegment(0.20f, -0.075f, false);
                addDigitSegment(0.13f, -0.15f, true);
            }
        }
    }


    return objects;
}
