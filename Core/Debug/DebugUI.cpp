#include "DebugUI.h"

#include "../SimulationController.h"
#include "../../Core/Object.h"
#include "../../Physics/RigidBody.h"
#include "../../Physics/Collider.h"
#include "../../Physics/Material.h"
#include "../../World/World.h"
#include "../../World/SceneSerializer.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_opengl3.h>

DebugUI::DebugUI()
    : m_simulation(nullptr),
    m_world(nullptr),
    m_initialized(false),
    m_showSimulationWindow(true),
    m_showHierarchyWindow(true),
    m_showInspectorWindow(true),
    m_showCreateWindow(true),
    m_showSceneWindow(true),
    m_selectedObjectIndex(-1),
    m_scenePath("Scenes/Test.scene") {}

bool DebugUI::Initialize(
    SDL_Window* window,
    SDL_GLContext context,
    SimulationController& simulation,
    World& world
) {
    if (m_initialized ||
        window == nullptr ||
        context == nullptr) {

        return false;
    }

    m_simulation = &simulation;
    m_world = &world;

    IMGUI_CHECKVERSION();

    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    if (!ImGui_ImplSDL3_InitForOpenGL(
        window,
        context
    )) {
        ImGui::DestroyContext();
        m_simulation = nullptr;
        m_world = nullptr;
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
        m_simulation = nullptr;
        m_world = nullptr;
        return false;
    }

    m_initialized = true;

    return true;
}

void DebugUI::ProcessEvent(const SDL_Event& event) {
    if (!m_initialized)
        return;

    ImGui_ImplSDL3_ProcessEvent(&event);
}

void DebugUI::Render() {
    if (!m_initialized ||
        m_simulation == nullptr ||
        m_world == nullptr) {

        return;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    if (m_showHierarchyWindow) {
        ImGui::Begin(
            "Hierarchy",
            &m_showHierarchyWindow
        );

        const auto& objects =
            m_world->GetObjects();

        for (size_t i = 0; i < objects.size(); ++i) {
            const Object* object = objects[i].get();

            if (object == nullptr)
                continue;

            const bool selected =
                m_selectedObjectIndex ==
                static_cast<int>(i);

            if (ImGui::Selectable(
                object->GetName().c_str(),
                selected
            )) {
                m_selectedObjectIndex =
                    static_cast<int>(i);
            }
        }

        ImGui::End();
    }

    if (m_showCreateWindow) {
        ImGui::Begin(
            "Create Object",
            &m_showCreateWindow
        );

        static int objectType = 0;
        static char name[64] = "Box";
        static float position[3] = { 0.0f, 3.0f, 0.0f };
        static float halfExtents[3] = { 0.5f, 0.5f, 0.5f };
        static float radius = 0.5f;
        static float mass = 1.0f;
        static float restitution = 0.0f;
        static float friction = 0.5f;

        const char* objectTypes[] = {
            "Box",
            "Sphere"
        };

        ImGui::Combo(
            "Type",
            &objectType,
            objectTypes,
            2
        );

        ImGui::InputText(
            "Name",
            name,
            sizeof(name)
        );

        ImGui::DragFloat3(
            "Position",
            position,
            0.05f
        );

        if (objectType == 0) {
            ImGui::DragFloat3(
                "Half Extents",
                halfExtents,
                0.05f,
                0.001f
            );
        }
        else {
            ImGui::DragFloat(
                "Radius",
                &radius,
                0.05f,
                0.001f
            );
        }

        ImGui::DragFloat(
            "Mass",
            &mass,
            0.1f,
            0.0f
        );

        ImGui::SliderFloat(
            "Restitution",
            &restitution,
            0.0f,
            1.0f
        );

        ImGui::SliderFloat(
            "Friction",
            &friction,
            0.0f,
            1.0f
        );

        if (ImGui::Button("Create")) {
            Object* object = nullptr;

            if (objectType == 0) {
                object = m_world->CreateBox({
                    name,
                    Vec3(
                        position[0],
                        position[1],
                        position[2]
                    ),
                    Vec3(
                        halfExtents[0],
                        halfExtents[1],
                        halfExtents[2]
                    ),
                    mass,
                    restitution,
                    friction
                });
            }
            else {
                object = m_world->CreateSphere({
                    name,
                    Vec3(
                        position[0],
                        position[1],
                        position[2]
                    ),
                    radius,
                    mass,
                    restitution,
                    friction
                });
            }

            if (object != nullptr) {
                const auto& objects =
                    m_world->GetObjects();

                m_selectedObjectIndex =
                    static_cast<int>(objects.size()) - 1;
            }
        }

        ImGui::End();
    }

    if (m_showInspectorWindow) {
        ImGui::Begin(
            "Inspector",
            &m_showInspectorWindow
        );

        const auto& objects =
            m_world->GetObjects();

        Object* selectedObject = nullptr;

        if (m_selectedObjectIndex >= 0 &&
            m_selectedObjectIndex <
                static_cast<int>(objects.size())) {

            selectedObject =
                objects[m_selectedObjectIndex].get();
        }

        if (selectedObject == nullptr) {
            ImGui::Text("No object selected.");
        }
        else {
            ImGui::Text(
                "Object: %s",
                selectedObject->GetName().c_str()
            );

            ImGui::Separator();

            if (ImGui::Button("Delete Object")) {
                if (m_world->DestroyObject(selectedObject)) {
                    if (m_selectedObjectIndex >=
                        static_cast<int>(m_world->GetObjects().size())) {

                        m_selectedObjectIndex =
                            static_cast<int>(m_world->GetObjects().size()) - 1;
                    }

                    m_selectedObjectIndex = -1;
                    selectedObject = nullptr;
                }
            }

            ImGui::Separator();

            if (selectedObject == nullptr) {
                ImGui::Text("Object deleted.");
            }
            else {
                Transform& transform =
                selectedObject->GetTransform();

            RigidBody* body =
                selectedObject->GetRigidBody();

            if (ImGui::CollapsingHeader(
                "Transform",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
                ImGui::DragFloat3(
                    "Position",
                    &transform.position.x,
                    0.05f
                );

                ImGui::DragFloat4(
                    "Rotation",
                    &transform.rotation.w,
                    0.01f
                );

                ImGui::DragFloat3(
                    "Scale",
                    &transform.scale.x,
                    0.05f,
                    0.001f
                );

                if (body != nullptr) {
                    body->SetPosition(
                        transform.position
                    );
                    body->SetOrientation(
                        transform.rotation
                    );
                }
            }

            if (body != nullptr) {

                if (ImGui::CollapsingHeader(
                    "RigidBody",
                    ImGuiTreeNodeFlags_DefaultOpen
                )) {
                    float mass = body->GetMass();

                    if (ImGui::DragFloat(
                        "Mass",
                        &mass,
                        0.1f,
                        0.0f
                    )) {
                        body->SetMass(mass);

                        if (Collider* collider =
                            selectedObject->GetCollider()) {

                            if (collider->GetShape() ==
                                ColliderShape::Box) {
                                body->SetBoxInertia(
                                    collider->GetHalfExtents() * 2.0f
                                );
                            }
                            else {
                                body->SetSphereInertia(
                                    collider->GetRadius()
                                );
                            }
                        }
                    }

                    Vec3 linearVelocity =
                        body->GetLinearVelocity();

                    if (ImGui::DragFloat3(
                        "Linear Velocity",
                        &linearVelocity.x,
                        0.05f
                    )) {
                        body->SetLinearVelocity(
                            linearVelocity
                        );
                    }

                    Vec3 angularVelocity =
                        body->GetAngularVelocity();

                    if (ImGui::DragFloat3(
                        "Angular Velocity",
                        &angularVelocity.x,
                        0.05f
                    )) {
                        body->SetAngularVelocity(
                            angularVelocity
                        );
                    }

                    ImGui::Text(
                        "Sleeping: %s",
                        body->IsSleeping()
                            ? "Yes"
                            : "No"
                    );
                }
            }

            if (Collider* collider =
                selectedObject->GetCollider()) {

                if (ImGui::CollapsingHeader(
                    "Collider",
                    ImGuiTreeNodeFlags_DefaultOpen
                )) {
                    const char* shapeNames[] = {
                        "Box",
                        "Sphere"
                    };

                    int shape =
                        collider->GetShape() ==
                            ColliderShape::Box
                            ? 0
                            : 1;

                    if (ImGui::Combo(
                        "Shape",
                        &shape,
                        shapeNames,
                        2
                    )) {
                        collider->SetShape(
                            shape == 0
                                ? ColliderShape::Box
                                : ColliderShape::Sphere
                        );

                        if (collider->GetShape() ==
                            ColliderShape::Box) {
                            transform.scale =
                                collider->GetHalfExtents() * 2.0f;
                            body->SetBoxInertia(
                                collider->GetHalfExtents() * 2.0f
                            );
                        }
                        else {
                            const float radius =
                                collider->GetRadius();

                            transform.scale =
                                Vec3(
                                    radius * 2.0f,
                                    radius * 2.0f,
                                    radius * 2.0f
                                );
                            body->SetSphereInertia(radius);
                        }
                    }

                    if (collider->GetShape() ==
                        ColliderShape::Box) {

                        Vec3 halfExtents =
                            collider->GetHalfExtents();

                        if (ImGui::DragFloat3(
                            "Half Extents",
                            &halfExtents.x,
                            0.05f,
                            0.001f
                        )) {
                            collider->SetHalfExtents(
                                halfExtents
                            );

                            transform.scale =
                                halfExtents * 2.0f;

                            body->SetBoxInertia(
                                halfExtents * 2.0f
                            );
                        }
                    }
                    else {
                        float radius =
                            collider->GetRadius();

                        if (ImGui::DragFloat(
                            "Radius",
                            &radius,
                            0.05f,
                            0.001f
                        )) {
                            collider->SetRadius(radius);

                            transform.scale =
                                Vec3(
                                    radius * 2.0f,
                                    radius * 2.0f,
                                    radius * 2.0f
                                );

                            body->SetSphereInertia(radius);
                        }
                    }

                    Material& material =
                        collider->GetMaterial();

                    float restitution =
                        material.GetRestitution();

                    if (ImGui::SliderFloat(
                        "Restitution",
                        &restitution,
                        0.0f,
                        1.0f
                    )) {
                        material.SetRestitution(
                            restitution
                        );
                    }

                    float friction =
                        material.GetFriction();

                    if (ImGui::SliderFloat(
                        "Friction",
                        &friction,
                        0.0f,
                        1.0f
                    )) {
                        material.SetFriction(friction);
                    }
                }
                }
            }
        }

        ImGui::End();
    }

    if (m_showSceneWindow) {
        ImGui::Begin("Scene", &m_showSceneWindow);

        ImGui::InputText("Path", m_scenePath, sizeof(m_scenePath));

        if (ImGui::Button("Save Scene"))
            m_sceneStatus = SceneSerializer::Save(*m_world, m_scenePath)
                ? "Scene saved."
                : "Failed to save scene.";

        ImGui::SameLine();

        if (ImGui::Button("Load Scene")) {
            m_simulation->Pause();
            const bool loaded =
                SceneSerializer::Load(*m_world, m_scenePath);

            if (loaded)
                m_selectedObjectIndex = -1;

            m_sceneStatus = loaded
                ? "Scene loaded."
                : "Failed to load scene.";
        }

        ImGui::Text("%s", m_sceneStatus.c_str());
        ImGui::End();
    }

    if (m_showSimulationWindow) {
        ImGui::Begin(
            "Simulation",
            &m_showSimulationWindow
        );

        if (m_simulation->IsPaused()) {
            if (ImGui::Button("Resume"))
                m_simulation->Resume();

            ImGui::SameLine();

            if (ImGui::Button("Step"))
                m_simulation->RequestSingleStep();
        }
        else {
            if (ImGui::Button("Pause"))
                m_simulation->Pause();
        }

        float timeScale =
            m_simulation->GetTimeScale();

        if (ImGui::SliderFloat(
            "Time Scale",
            &timeScale,
            0.0f,
            4.0f,
            "%.2f"
        )) {
            m_simulation->SetTimeScale(timeScale);
        }

        ImGui::Text(
            "State: %s",
            m_simulation->IsPaused()
                ? "Paused"
                : "Running"
        );

        ImGui::Text(
            "Fixed Step: %.6f s",
            1.0f / 120.0f
        );

        ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(
        ImGui::GetDrawData()
    );
}

void DebugUI::Shutdown() {
    if (!m_initialized)
        return;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    m_simulation = nullptr;
    m_world = nullptr;
    m_selectedObjectIndex = -1;
    m_initialized = false;
}