#include "World.h"

#include <sstream>

#include "../Core/Debug/Logger.h"

World::World()
    : m_physicsWorld(nullptr) {}

World::~World() {
    Shutdown();
}

bool World::Initialize(
    MeshManager& meshManager,
    PhysicsWorld& physicsWorld
) {
    m_physicsWorld = &physicsWorld;

    // Zero gravity for testing purposes
    // m_physicsWorld->SetGravity(Vec3(0.0f, 0.0f, 0.0f));

    Mesh* cubeMesh = meshManager.Get("cube");

    meshManager.CreateSphere("sphere");
    Mesh* sphereMesh = meshManager.Get("sphere");

    // Object 1 - Dynamic
    //auto object1 = std::make_unique<Object>();
    //object1->SetMesh(cubeMesh);

    //RigidBody* body1 =
    //    m_physicsWorld->CreateRigidBody();

    //body1->SetPosition(
    //    Vec3(0.0f, 3.0f, 0.0f)
    //);

    //body1->SetMass(1.0f);
    //body1->SetBoxInertia(
    //    Vec3(1.0f, 1.0f, 1.0f)
    //);

    //Collider* collider1 =
    //    m_physicsWorld->CreateCollider();

    //collider1->SetHalfExtents(
    //    Vec3(0.5f, 0.5f, 0.5f)
    //);

    //collider1->SetRigidBody(body1);

    //collider1->GetMaterial().SetRestitution(0.0f);
    //collider1->GetMaterial().SetFriction(0.0f);

    //object1->SetRigidBody(body1);
    //object1->SetCollider(collider1);

    //object1->GetTransform().position =
    //    body1->GetPosition();

    //AddObject(std::move(object1));


    // Object 2 - Dynamic
    // auto object2 = std::make_unique<Object>();
    // object2->SetMesh(cubeMesh);

    // RigidBody* body2 =
    //     m_physicsWorld->CreateRigidBody();

    // body2->SetPosition(
    //     Vec3(0.3f, 1.1f, 0.0f)
    // );

    // body2->SetMass(1.0f);
    // body2->SetBoxInertia(
    //     Vec3(1.0f, 1.0f, 1.0f)
    // );

    // Collider* collider2 =
    //     m_physicsWorld->CreateCollider();

    // collider2->SetHalfExtents(
    //     Vec3(0.5f, 0.5f, 0.5f)
    // );

    // collider2->SetRigidBody(body2);

    // collider2->GetMaterial().SetRestitution(0.0f);
    // collider2->GetMaterial().SetFriction(0.0f);

    // object2->SetRigidBody(body2);
    // object2->SetCollider(collider2);

    // object2->GetTransform().position =
    //     body2->GetPosition();

    // AddObject(std::move(object2));


    // Object 3 - Static Ground
    auto object3 = std::make_unique<Object>();
    object3->SetMesh(cubeMesh);

    RigidBody* body3 =
        m_physicsWorld->CreateRigidBody();

    body3->SetPosition(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    body3->SetMass(0.0f);

    Collider* collider3 =
        m_physicsWorld->CreateCollider();

    collider3->SetHalfExtents(
        Vec3(1.0f, 0.5f, 1.0f)
    );

    collider3->SetRigidBody(body3);

    collider3->GetMaterial().SetRestitution(0.0f);
    collider3->GetMaterial().SetFriction(0.0f);

    object3->SetRigidBody(body3);
    object3->SetCollider(collider3);

    object3->GetTransform().position =
        body3->GetPosition();

    AddObject(std::move(object3));


    // Object 4 - Dynamic Sphere (구가 큐브와 동일하게 작동하는지 확인용)
    auto object4 = std::make_unique<Object>();
    object4->SetMesh(sphereMesh);

    RigidBody* body4 =
        m_physicsWorld->CreateRigidBody();

    body4->SetPosition(
        Vec3(0.0f, 2.5f, 0.0f)
    );

    body4->SetMass(1.0f);
    body4->SetSphereInertia(0.5f);

    Collider* collider4 =
        m_physicsWorld->CreateCollider();

    collider4->SetShape(ColliderShape::Sphere);
    collider4->SetRadius(0.5f);

    collider4->SetRigidBody(body4);

    collider4->GetMaterial().SetRestitution(0.9f);
    collider4->GetMaterial().SetFriction(0.4f);

    object4->SetRigidBody(body4);
    object4->SetCollider(collider4);

    object4->GetTransform().position =
        body4->GetPosition();

    AddObject(std::move(object4));

    TestRaycast();

    return true;
}

void World::Update(
    float deltaTime,
    const InputManager& input
) {
    const Keyboard& keyboard = input.GetKeyboard();
    constexpr float cameraSpeed = 5.0f;

    Vec3 movement(0.0f, 0.0f, 0.0f);

    if (keyboard.IsDown(SDL_SCANCODE_W))
        movement.z += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_S))
        movement.z -= 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_D))
        movement.x += 1.0f;

    if (keyboard.IsDown(SDL_SCANCODE_A))
        movement.x -= 1.0f;

    m_camera.MoveLocal(movement * cameraSpeed * deltaTime);

    for (auto& object : m_objects) {
        if (object == nullptr)
            continue;

        RigidBody* body = object->GetRigidBody();

        if (body == nullptr)
            continue;

        object->GetTransform().position = body->GetPosition();
        object->GetTransform().rotation = body->GetOrientation();

        // 콜라이더 크기에 맞춰 렌더 스케일을 매 프레임 동기화한다.
        // (이전에는 여기서 scale을 전혀 건드리지 않아 Transform 기본값인
        // (1,1,1)에 고정되어 있었고, 그래서 halfExtents를 직육면체로
        // 줘도 화면엔 항상 정육면체 메시 그대로 보였다. 큐브 메시/구
        // 메시 둘 다 "반지름/반폭 0.5" 단위로 만들어져 있으므로,
        // 박스는 halfExtents*2, 구는 반지름*2를 스케일로 쓰면 된다.)
        const Collider* collider = object->GetCollider();

        if (collider != nullptr) {
            if (collider->GetShape() == ColliderShape::Sphere) {
                const float diameter = collider->GetRadius() * 2.0f;
                object->GetTransform().scale = Vec3(diameter, diameter, diameter);
            }
            else {
                object->GetTransform().scale = collider->GetHalfExtents() * 2.0f;
            }
        }
    }
}

Camera& World::GetCamera() {
    return m_camera;
}

const Camera& World::GetCamera() const {
    return m_camera;
}

const std::vector<std::unique_ptr<Object>>&
World::GetObjects() const {
    return m_objects;
}

void World::AddObject(std::unique_ptr<Object> object) {
    m_objects.push_back(std::move(object));
}

void World::Shutdown() {
    m_objects.clear();
    m_physicsWorld = nullptr;
}
void World::TestRaycast() const {
    if (m_physicsWorld == nullptr)
        return;

    Ray sphereRay;
    sphereRay.origin = Vec3(0.0f, 5.0f, 0.0f);
    sphereRay.direction = Vec3(0.0f, -1.0f, 0.0f);

    RaycastResult sphereResult;

    if (m_physicsWorld->Raycast(
        sphereRay,
        sphereResult
    )) {
        std::ostringstream message;
        message << "[Raycast Test] Sphere HIT distance="
            << sphereResult.distance
            << " point=("
            << sphereResult.point.x << ", "
            << sphereResult.point.y << ", "
            << sphereResult.point.z << ") normal=("
            << sphereResult.normal.x << ", "
            << sphereResult.normal.y << ", "
            << sphereResult.normal.z << ")";

        Logger::Info(message.str());
    }
    else {
        Logger::Error("[Raycast Test] Sphere MISS");
    }

    Ray groundRay;
    groundRay.origin = Vec3(0.0f, 1.5f, 0.0f);
    groundRay.direction = Vec3(0.0f, -1.0f, 0.0f);

    RaycastResult groundResult;

    if (m_physicsWorld->Raycast(
        groundRay,
        groundResult
    )) {
        std::ostringstream message;
        message << "[Raycast Test] Ground HIT distance="
            << groundResult.distance
            << " point=("
            << groundResult.point.x << ", "
            << groundResult.point.y << ", "
            << groundResult.point.z << ") normal=("
            << groundResult.normal.x << ", "
            << groundResult.normal.y << ", "
            << groundResult.normal.z << ")";

        Logger::Info(message.str());
    }
    else {
        Logger::Error("[Raycast Test] Ground MISS");
    }
}
