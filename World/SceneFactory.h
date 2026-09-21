#pragma once

#include <memory>
#include <string>

#include "../Core/Math/Vec3.h"

class MeshManager;
class PhysicsWorld;
class Object;

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

private:
    MeshManager& m_meshManager;
    PhysicsWorld& m_physicsWorld;
};