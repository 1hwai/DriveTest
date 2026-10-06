#pragma once

#include <memory>
#include <string>

#include "../Core/Math/Vec3.h"

class MeshManager;
class PhysicsWorld;
class Object;
class Terrain;
class Road;

struct BoxSettings {
    std::string name;
    Vec3 position;
    Vec3 halfExtents;
    float mass;
    float restitution;
    float friction;
};

struct SphereSettings {
    std::string name;
    Vec3 position;
    float radius;
    float mass;
    float restitution;
    float friction;
};

struct PlaneSettings {
    std::string name;
    float height;
};

struct TerrainSettings {
    std::string name;
    float restitution;
    float friction;
};

struct RoadSettings {
    std::string name;
    float restitution;
    float friction;
};

class SceneFactory {
public:
    SceneFactory(
        MeshManager& meshManager,
        PhysicsWorld& physicsWorld
    );

    std::unique_ptr<Object> CreateBox(
        const BoxSettings& settings
    );

    std::unique_ptr<Object> CreateSphere(
        const SphereSettings& settings
    );

    std::unique_ptr<Object> CreatePlane(
        const PlaneSettings& settings
    );

    std::unique_ptr<Object> CreateTerrain(
        const TerrainSettings& settings,
        const Terrain& terrain
    );

    std::unique_ptr<Object> CreateTerrainCollider(
        const TerrainSettings& settings,
        const Terrain& terrain
    );

    std::unique_ptr<Object> CreateRoad(
        const RoadSettings& settings,
        const Road& road
    );

private:
    MeshManager& m_meshManager;
    PhysicsWorld& m_physicsWorld;
};