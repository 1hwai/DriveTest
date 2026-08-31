#include "World.h"

World::World() = default;

World::~World() {
    Shutdown();
}

bool World::Initialize(MeshManager& meshManager) {
    auto object = std::make_unique<Object>();

	Mesh* cubeMesh = meshManager.Get("cube");
	object->SetMesh(cubeMesh);

    object->GetTransform().position =
        Vec3(0.0f, 0.0f, 5.0f);

	AddObject(std::move(object));

    return true;
}

void World::Update(
    float deltaTime,
    const InputManager& input
) {
    const Keyboard& keyboard =
        input.GetKeyboard();

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

    m_camera.MoveLocal(
        movement * cameraSpeed * deltaTime
    );
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
}