#include "SceneSerializer.h"

#include "World.h"
#include "../Core/Object.h"
#include "../Physics/Collider.h"
#include "../Physics/RigidBody.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace {
    constexpr const char* SceneHeader = "DRIVETEST_SCENE 1";

    bool ReadVec3(std::istream& stream, Vec3& value) {
        return static_cast<bool>(
            stream >> value.x >> value.y >> value.z
        );
    }

    bool ReadQuaternion(
        std::istream& stream,
        Quaternion& value
    ) {
        return static_cast<bool>(
            stream >> value.w >>
            value.x >>
            value.y >>
            value.z
        );
    }
}

bool SceneSerializer::Save(
    const World& world,
    const std::string& path
) {
    const std::filesystem::path filePath(path);

    if (filePath.has_parent_path()) {
        std::error_code error;
        std::filesystem::create_directories(
            filePath.parent_path(),
            error
        );

        if (error)
            return false;
    }

    std::ofstream file(path);

    if (!file)
        return false;

    file << SceneHeader << '\n';
    file << std::setprecision(9);

    const Camera& camera = world.GetCamera();
    const Vec3& cameraPosition = camera.GetPosition();
    const Vec3& cameraRotation = camera.GetRotation();

    file << "CAMERA\n";
    file << cameraPosition.x << ' '
         << cameraPosition.y << ' '
         << cameraPosition.z << '\n';
    file << cameraRotation.x << ' '
         << cameraRotation.y << ' '
         << cameraRotation.z << '\n';

    const auto& objects = world.GetObjects();

    file << "OBJECTS " << objects.size() << '\n';

    for (const auto& objectPtr : objects) {
        if (!objectPtr)
            continue;

        const Object& object = *objectPtr;
        const RigidBody* body = object.GetRigidBody();
        const Collider* collider = object.GetCollider();

        if (!body || !collider)
            return false;

        file << "OBJECT\n";
        file << "NAME " << std::quoted(object.GetName()) << '\n';

        const Vec3& position = body->GetPosition();
        const Quaternion& rotation = body->GetOrientation();
        const Vec3& linearVelocity = body->GetLinearVelocity();
        const Vec3& angularVelocity = body->GetAngularVelocity();

        file << "POSITION "
             << position.x << ' '
             << position.y << ' '
             << position.z << '\n';

        file << "ROTATION "
             << rotation.w << ' '
             << rotation.x << ' '
             << rotation.y << ' '
             << rotation.z << '\n';

        file << "LINEAR_VELOCITY "
             << linearVelocity.x << ' '
             << linearVelocity.y << ' '
             << linearVelocity.z << '\n';

        file << "ANGULAR_VELOCITY "
             << angularVelocity.x << ' '
             << angularVelocity.y << ' '
             << angularVelocity.z << '\n';

        file << "MASS " << body->GetMass() << '\n';

        const Material& material = collider->GetMaterial();

        file << "MATERIAL "
             << material.GetRestitution() << ' '
             << material.GetFriction() << '\n';

        if (collider->GetShape() == ColliderShape::Box) {
            const Vec3& halfExtents =
                collider->GetHalfExtents();

            file << "SHAPE BOX "
                 << halfExtents.x << ' '
                 << halfExtents.y << ' '
                 << halfExtents.z << '\n';
        }
        else {
            file << "SHAPE SPHERE "
                 << collider->GetRadius() << '\n';
        }

        file << "END_OBJECT\n";
    }

    file << "END_SCENE\n";

    return static_cast<bool>(file);
}

bool SceneSerializer::Load(
    World& world,
    const std::string& path
) {
    std::ifstream file(path);

    if (!file)
        return false;

    std::string header;
    std::getline(file, header);

    if (header != SceneHeader)
        return false;

    std::string token;

    if (!(file >> token) || token != "CAMERA")
        return false;

    Vec3 cameraPosition;
    Vec3 cameraRotation;

    if (!ReadVec3(file, cameraPosition) ||
        !ReadVec3(file, cameraRotation)) {
        return false;
    }

    if (!(file >> token) || token != "OBJECTS")
        return false;

    size_t objectCount = 0;

    if (!(file >> objectCount))
        return false;

    struct LoadedObject {
        std::string name;
        Vec3 position;
        Quaternion rotation;
        Vec3 linearVelocity;
        Vec3 angularVelocity;
        float mass = 0.0f;
        float restitution = 0.0f;
        float friction = 0.5f;
        ColliderShape shape = ColliderShape::Box;
        Vec3 halfExtents = Vec3(0.5f, 0.5f, 0.5f);
        float radius = 0.5f;
    };

    std::vector<LoadedObject> loadedObjects;
    loadedObjects.reserve(objectCount);

    for (size_t i = 0; i < objectCount; ++i) {
        if (!(file >> token) || token != "OBJECT")
            return false;

        LoadedObject loaded;

        if (!(file >> token) || token != "NAME")
            return false;

        if (!(file >> std::quoted(loaded.name)))
            return false;

        if (!(file >> token) || token != "POSITION" ||
            !ReadVec3(file, loaded.position)) {
            return false;
        }

        if (!(file >> token) || token != "ROTATION" ||
            !ReadQuaternion(file, loaded.rotation)) {
            return false;
        }

        if (!(file >> token) || token != "LINEAR_VELOCITY" ||
            !ReadVec3(file, loaded.linearVelocity)) {
            return false;
        }

        if (!(file >> token) || token != "ANGULAR_VELOCITY" ||
            !ReadVec3(file, loaded.angularVelocity)) {
            return false;
        }

        if (!(file >> token) || token != "MASS" ||
            !(file >> loaded.mass)) {
            return false;
        }

        if (!(file >> token) || token != "MATERIAL" ||
            !(file >> loaded.restitution >> loaded.friction)) {
            return false;
        }

        if (!(file >> token) || token != "SHAPE")
            return false;

        std::string shape;
        if (!(file >> shape))
            return false;

        if (shape == "BOX") {
            loaded.shape = ColliderShape::Box;

            if (!ReadVec3(file, loaded.halfExtents))
                return false;
        }
        else if (shape == "SPHERE") {
            loaded.shape = ColliderShape::Sphere;

            if (!(file >> loaded.radius))
                return false;
        }
        else {
            return false;
        }

        if (!(file >> token) || token != "END_OBJECT")
            return false;

        loadedObjects.push_back(loaded);
    }

    if (!(file >> token) || token != "END_SCENE")
        return false;

    world.ClearObjects();

    world.GetCamera().SetPosition(cameraPosition);
    world.GetCamera().SetRotation(cameraRotation);

    for (const LoadedObject& loaded : loadedObjects) {
        Object* object = nullptr;

        if (loaded.shape == ColliderShape::Box) {
            object = world.CreateBox({
                loaded.name,
                loaded.position,
                loaded.halfExtents,
                loaded.mass,
                loaded.restitution,
                loaded.friction
            });
        }
        else {
            object = world.CreateSphere({
                loaded.name,
                loaded.position,
                loaded.radius,
                loaded.mass,
                loaded.restitution,
                loaded.friction
            });
        }

        if (!object)
            return false;

        RigidBody* body = object->GetRigidBody();

        body->SetOrientation(loaded.rotation);
        body->SetLinearVelocity(loaded.linearVelocity);
        body->SetAngularVelocity(loaded.angularVelocity);
    }

    return true;
}
