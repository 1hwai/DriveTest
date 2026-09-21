#pragma once

#include <SDL3/SDL.h>

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
    int m_selectedObjectIndex;
};