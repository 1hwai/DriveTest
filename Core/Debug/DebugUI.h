#pragma once

#include <SDL3/SDL.h>
#include <string>

class SimulationController;
class World;

class DebugUI {
public:
    DebugUI();

    bool Initialize(
        SDL_Window* window,
        SDL_GLContext context,
        SimulationController& simulation,
        World& world
    );

    void ProcessEvent(const SDL_Event& event);
    void Render();
    void Shutdown();

private:
    SimulationController* m_simulation;
    World* m_world;

    bool m_initialized;
    bool m_showSimulationWindow;
    bool m_showHierarchyWindow;
    bool m_showInspectorWindow;
    bool m_showCreateWindow;
    bool m_showSceneWindow;
    int m_selectedObjectIndex;
    char m_scenePath[256];
    std::string m_sceneStatus;
};