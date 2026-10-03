#include "DebugUI.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "../SimulationController.h"
#include "../../Core/Object.h"
#include "../../Physics/RigidBody.h"
#include "../../Physics/Collider.h"
#include "../../Physics/Material.h"
#include "../../World/Scene.h"
#include "../../World/SceneSerializer.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_opengl3.h>

DebugUI::DebugUI()
    : m_simulation(nullptr),
    m_scene(nullptr),
    m_initialized(false),
    m_showSimulationWindow(true),
    m_showHierarchyWindow(true),
    m_showInspectorWindow(true),
    m_showCreateWindow(true),
    m_showSceneWindow(true),
    m_selectedObjectIndex(-1),
    m_scenePath{} {
    std::snprintf(
        m_scenePath,
        sizeof(m_scenePath),
        "%s",
        "Scenes/Test.scene"
    );
}

bool DebugUI::Initialize(
    SDL_Window* window,
    SDL_GLContext context,
    SimulationController& simulation,
    Scene& scene
) {
    if (m_initialized ||
        window == nullptr ||
        context == nullptr) {

        return false;
    }

    m_simulation = &simulation;
    m_scene = &scene;

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
        m_scene = nullptr;
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
        m_simulation = nullptr;
        m_scene = nullptr;
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
        m_scene == nullptr) {

        return;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    {
        const Car& car = m_scene->GetCar();
        const Engine& engine = car.GetEngine();
        const int gear = car.GetTransmission().GetGear();
        const float rpm = engine.GetRPM();
        const float idleRPM = engine.GetIdleRPM();
        const float redlineRPM = engine.GetRedlineRPM();

        ImGui::SetNextWindowPos(
            ImVec2(20.0f, 20.0f),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowBgAlpha(0.75f);

        ImGui::Begin(
            "Vehicle Telemetry",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings
        );

        const char* gearText = "N";

        if (gear > 0)
            gearText = gear == 1 ? "1" :
                gear == 2 ? "2" :
                gear == 3 ? "3" :
                gear == 4 ? "4" :
                gear == 5 ? "5" : "?";
        else if (gear < 0)
            gearText = "R";

        ImGui::Text("GEAR  %s", gearText);
        ImGui::Text("RPM   %.0f / %.0f", rpm, redlineRPM);

        const float rpmProgress =
            (rpm - idleRPM) /
            (redlineRPM - idleRPM);

        ImGui::ProgressBar(
            rpmProgress,
            ImVec2(220.0f, 18.0f)
        );

        ImGui::Text("SHIFT  [+] Up   [-] Down");
        if (!engine.IsRunning()) {
            ImGui::Spacing();
            if (ImGui::Button("START ENGINE", ImVec2(220.0f, 32.0f)))
                m_scene->GetCar().GetEngine().Start();
        }
        else {
            ImGui::TextUnformatted("ENGINE RUNNING");
        }

        ImGui::End();
    }

    {
        const float speedKmh =
            m_scene->GetCar().GetSpeedKmh();

        constexpr float maxSpeed = 240.0f;
        constexpr float radius = 78.0f;
        constexpr float startAngle = 2.35619449f;
        constexpr float endAngle = 7.06858347f;

        ImGui::SetNextWindowPos(
            ImVec2(
                ImGui::GetMainViewport()->WorkPos.x +
                    ImGui::GetMainViewport()->WorkSize.x - 210.0f,
                ImGui::GetMainViewport()->WorkPos.y + 20.0f
            ),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowSize(
            ImVec2(190.0f, 190.0f),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowBgAlpha(0.75f);

        ImGui::Begin(
            "Speedometer",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoInputs
        );

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 windowPos = ImGui::GetWindowPos();
        const ImVec2 center(
            windowPos.x + 95.0f,
            windowPos.y + 92.0f
        );

        drawList->AddCircle(
            center,
            radius,
            IM_COL32(220, 220, 220, 255),
            64,
            3.0f
        );

        const float normalizedSpeed =
            std::clamp(speedKmh / maxSpeed, 0.0f, 1.0f);

        drawList->PathArcTo(
            center,
            radius - 5.0f,
            startAngle,
            endAngle,
            64
        );
        drawList->PathStroke(
            IM_COL32(255, 255, 255, 255),
            0,
            5.0f
        );

        for (int i = 0; i <= 12; ++i) {
            const float t =
                static_cast<float>(i) / 12.0f;
            const float angle =
                startAngle +
                (endAngle - startAngle) * t;

            const ImVec2 outer(
                center.x + std::cos(angle) * (radius - 7.0f),
                center.y + std::sin(angle) * (radius - 7.0f)
            );
            const ImVec2 inner(
                center.x + std::cos(angle) * (radius - 17.0f),
                center.y + std::sin(angle) * (radius - 17.0f)
            );

            drawList->AddLine(
                outer,
                inner,
                IM_COL32(220, 220, 220, 255),
                2.0f
            );
        }

        const float needleAngle =
            startAngle +
            (endAngle - startAngle) * normalizedSpeed;

        const ImVec2 needleEnd(
            center.x + std::cos(needleAngle) * (radius - 22.0f),
            center.y + std::sin(needleAngle) * (radius - 22.0f)
        );

        drawList->AddLine(
            center,
            needleEnd,
            IM_COL32(255, 255, 255, 255),
            4.0f
        );
        drawList->AddCircleFilled(
            center,
            6.0f,
            IM_COL32(255, 255, 255, 255)
        );

        const std::string speedText =
            std::to_string(static_cast<int>(std::round(speedKmh)));
        const ImVec2 textSize =
            ImGui::CalcTextSize(speedText.c_str());

        drawList->AddText(
            ImVec2(
                center.x - textSize.x * 0.5f,
                center.y + 25.0f
            ),
            IM_COL32(255, 255, 255, 255),
            speedText.c_str()
        );

        const char* unit = "km/h";
        const ImVec2 unitSize =
            ImGui::CalcTextSize(unit);

        drawList->AddText(
            ImVec2(
                center.x - unitSize.x * 0.5f,
                center.y + 42.0f
            ),
            IM_COL32(190, 190, 190, 255),
            unit
        );

        ImGui::End();
    }


    {
        const Car& car = m_scene->GetCar();
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        const float bottom = viewport->WorkPos.y + viewport->WorkSize.y;

        ImGui::SetNextWindowPos(
            ImVec2(viewport->WorkPos.x + 20.0f, bottom - 150.0f),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowBgAlpha(0.75f);
        ImGui::Begin(
            "Debug Input",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoInputs
        );

        ImGui::TextUnformatted("DEBUG / INPUT");
        ImGui::Text("Throttle");
        ImGui::SameLine(90.0f);
        ImGui::ProgressBar(car.GetThrottle(), ImVec2(130.0f, 14.0f), "");
        ImGui::Text("Brake");
        ImGui::SameLine(90.0f);
        ImGui::ProgressBar(car.GetBrake(), ImVec2(130.0f, 14.0f), "");
        ImGui::Text("Clutch");
        ImGui::SameLine(90.0f);
        ImGui::ProgressBar(car.GetClutch(), ImVec2(130.0f, 14.0f), "");
        ImGui::Text("Steering  %+0.2f", car.GetSteering());
        ImGui::End();
    }

    {
        const Car& car = m_scene->GetCar();
        const RigidBody* chassis = car.GetChassis();
        const Vec3 velocity = chassis != nullptr
            ? chassis->GetLinearVelocity()
            : Vec3();
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        const float panelWidth = 280.0f;
        const float bottom = viewport->WorkPos.y + viewport->WorkSize.y;

        ImGui::SetNextWindowPos(
            ImVec2(
                viewport->WorkPos.x + viewport->WorkSize.x * 0.5f - panelWidth * 0.5f,
                bottom - 128.0f
            ),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowSize(ImVec2(panelWidth, 108.0f), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.75f);
        ImGui::Begin(
            "Debug Velocity",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoInputs
        );

        ImGui::TextUnformatted("DEBUG / WORLD VELOCITY (m/s)");
        ImGui::Text("Vx %+.2f   Vy %+.2f   Vz %+.2f",
            velocity.x, velocity.y, velocity.z);

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 windowPos = ImGui::GetWindowPos();
        const ImVec2 origin(windowPos.x + panelWidth * 0.5f, windowPos.y + 77.0f);
        drawList->AddLine(
            ImVec2(origin.x - 48.0f, origin.y),
            ImVec2(origin.x + 48.0f, origin.y),
            IM_COL32(130, 130, 130, 180),
            1.0f
        );
        drawList->AddLine(
            ImVec2(origin.x, origin.y - 20.0f),
            ImVec2(origin.x, origin.y + 20.0f),
            IM_COL32(130, 130, 130, 180),
            1.0f
        );
        const float vectorScale = 3.0f;
        const float dx = velocity.x * vectorScale;
        const float dy = velocity.z * vectorScale;
        const float vectorLength = std::sqrt(dx * dx + dy * dy);
        const float scale = vectorLength > 45.0f ? 45.0f / vectorLength : 1.0f;
        const ImVec2 vectorEnd(origin.x + dx * scale, origin.y + dy * scale);
        drawList->AddLine(origin, vectorEnd, IM_COL32(255, 220, 80, 255), 3.0f);
        drawList->AddCircleFilled(vectorEnd, 3.5f, IM_COL32(255, 220, 80, 255));
        drawList->AddText(ImVec2(origin.x + 51.0f, origin.y - 7.0f),
            IM_COL32(220, 220, 220, 255), "X");
        drawList->AddText(ImVec2(origin.x - 4.0f, origin.y + 20.0f),
            IM_COL32(220, 220, 220, 255), "Z");
        ImGui::End();
    }

    {
        const Car& car = m_scene->GetCar();
        const RigidBody* chassis = car.GetChassis();
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        const float panelSize = 126.0f;
        const float bottom = viewport->WorkPos.y + viewport->WorkSize.y;
        const Mat3 rotation = chassis != nullptr
            ? chassis->GetOrientation().ToMat3()
            : Mat3::Identity();

        ImGui::SetNextWindowPos(
            ImVec2(
                viewport->WorkPos.x + viewport->WorkSize.x - panelSize - 20.0f,
                bottom - panelSize - 10.0f
            ),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowSize(ImVec2(panelSize, panelSize), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.75f);
        ImGui::Begin(
            "Debug Basis",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoInputs
        );

        ImGui::TextUnformatted("DEBUG / CAR AXES");
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 windowPos = ImGui::GetWindowPos();
        const ImVec2 origin(windowPos.x + panelSize * 0.5f, windowPos.y + 79.0f);
        const Vec3 axes[3] = {
            rotation * Vec3(1.0f, 0.0f, 0.0f),
            rotation * Vec3(0.0f, 1.0f, 0.0f),
            rotation * Vec3(0.0f, 0.0f, 1.0f)
        };
        const ImU32 colors[3] = {
            IM_COL32(245, 85, 85, 255),
            IM_COL32(90, 235, 120, 255),
            IM_COL32(90, 155, 255, 255)
        };
        const char* labels[3] = { "X", "Y", "Z" };

        drawList->AddCircleFilled(origin, 3.0f, IM_COL32(235, 235, 235, 255));
        for (int i = 0; i < 3; ++i) {
            const float projectedX = axes[i].x - axes[i].z * 0.45f;
            const float projectedY = -axes[i].y + axes[i].z * 0.35f;
            const ImVec2 end(
                origin.x + projectedX * 32.0f,
                origin.y + projectedY * 32.0f
            );
            drawList->AddLine(origin, end, colors[i], 2.5f);
            drawList->AddText(ImVec2(end.x + 3.0f, end.y - 7.0f), colors[i], labels[i]);
        }
        ImGui::End();
    }

    {
        const Car& car = m_scene->GetCar();
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        constexpr float panelWidth = 270.0f;
        constexpr float panelHeight = 224.0f;
        constexpr float tireWidth = 74.0f;
        constexpr float tireHeight = 58.0f;
        constexpr float maxDisplayedLoad = 12000.0f;

        ImGui::SetNextWindowPos(
            ImVec2(
                viewport->WorkPos.x + viewport->WorkSize.x - panelWidth - 20.0f,
                viewport->WorkPos.y + 220.0f
            ),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowSize(ImVec2(panelWidth, panelHeight), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.78f);
        ImGui::Begin(
            "Tire Loads / Slip",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoInputs
        );

        ImGui::TextUnformatted("TIRE LOAD / SLIP");
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 windowPos = ImGui::GetWindowPos();
        const float leftX = windowPos.x + 16.0f;
        const float rightX = windowPos.x + 180.0f;
        const float frontY = windowPos.y + 34.0f;
        const float rearY = windowPos.y + 126.0f;

        for (size_t i = 0; i < WheelCount; ++i) {
            const WheelIndex index = static_cast<WheelIndex>(i);
            const Wheel& wheel = car.GetWheel(index);
            const Tire& tire = car.GetTire(index);
            const TireState& state = tire.GetState();
            const float load = wheel.IsGrounded() ? tire.GetNormalLoad() : 0.0f;
            const float slipSpeed = wheel.IsGrounded()
                ? std::sqrt(
                    state.longitudinalSlipVelocity * state.longitudinalSlipVelocity +
                    state.lateralVelocity * state.lateralVelocity
                )
                : 0.0f;

            const bool isLeft = i == ToIndex(WheelIndex::FrontLeft) ||
                i == ToIndex(WheelIndex::RearLeft);
            const bool isFront = i == ToIndex(WheelIndex::FrontLeft) ||
                i == ToIndex(WheelIndex::FrontRight);
            const float x = isLeft ? leftX : rightX;
            const float y = isFront ? frontY : rearY;
            const ImVec2 topLeft(x, y);
            const ImVec2 bottomRight(x + tireWidth, y + tireHeight);
            const float loadFraction = std::clamp(load / maxDisplayedLoad, 0.0f, 1.0f);

            drawList->AddRectFilled(topLeft, bottomRight, IM_COL32(45, 45, 45, 230));
            if (loadFraction > 0.0f) {
                const float barHeight = (tireHeight - 2.0f) * loadFraction;
                drawList->AddRectFilled(
                    ImVec2(topLeft.x + 1.0f, bottomRight.y - barHeight - 1.0f),
                    ImVec2(bottomRight.x - 1.0f, bottomRight.y - 1.0f),
                    IM_COL32(55, 205, 105, 230)
                );
            }
            drawList->AddRect(topLeft, bottomRight, IM_COL32(175, 175, 175, 255), 0.0f, 0, 1.5f);

            char loadText[32];
            std::snprintf(loadText, sizeof(loadText), "%.0f N", load);
            const ImVec2 loadSize = ImGui::CalcTextSize(loadText);
            drawList->AddText(
                ImVec2(x + (tireWidth - loadSize.x) * 0.5f, y + 7.0f),
                IM_COL32(245, 245, 245, 255),
                loadText
            );

            char slipText[32];
            std::snprintf(slipText, sizeof(slipText), "%.3f", slipSpeed);
            const ImVec2 slipSize = ImGui::CalcTextSize(slipText);
            drawList->AddText(
                ImVec2(x + (tireWidth - slipSize.x) * 0.5f, y + 32.0f),
                slipSpeed > 0.05f ? IM_COL32(255, 90, 90, 255) : IM_COL32(220, 220, 220, 255),
                slipText
            );
        }

        ImGui::End();
    }

    if (m_showHierarchyWindow) {
        ImGui::Begin(
            "Hierarchy",
            &m_showHierarchyWindow
        );

        const auto& objects =
            m_scene->GetObjects();

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
                object = m_scene->CreateBox({
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
                object = m_scene->CreateSphere({
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
                    m_scene->GetObjects();

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
            m_scene->GetObjects();

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
                if (m_scene->DestroyObject(selectedObject)) {
                    if (m_selectedObjectIndex >=
                        static_cast<int>(m_scene->GetObjects().size())) {

                        m_selectedObjectIndex =
                            static_cast<int>(m_scene->GetObjects().size()) - 1;
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
            m_sceneStatus = SceneSerializer::Save(*m_scene, m_scenePath)
                ? "Scene saved."
                : "Failed to save scene.";

        ImGui::SameLine();

        if (ImGui::Button("Load Scene")) {
            m_simulation->Pause();
            const bool loaded =
                SceneSerializer::Load(*m_scene, m_scenePath);

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

        bool chaseCamera =
            m_scene->GetCameraMode() == CameraMode::Chase;

        if (ImGui::Checkbox(
            "Chase Camera",
            &chaseCamera
        )) {
            m_scene->SetCameraMode(
                chaseCamera
                    ? CameraMode::Chase
                    : CameraMode::Free
            );
        }

        ImGui::Separator();

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
    m_scene = nullptr;
    m_selectedObjectIndex = -1;
    m_initialized = false;
}