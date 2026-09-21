#include "DebugUI.h"

#include "SimulationController.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_opengl3.h>

DebugUI::DebugUI()
    : m_simulation(nullptr),
    m_initialized(false),
    m_showSimulationWindow(true) {}

bool DebugUI::Initialize(
    SDL_Window* window,
    SDL_GLContext context,
    SimulationController& simulation
) {
    if (m_initialized || window == nullptr || context == nullptr)
        return false;

    m_simulation = &simulation;

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
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
        m_simulation = nullptr;
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
    if (!m_initialized || m_simulation == nullptr)
        return;

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

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
    m_initialized = false;
}