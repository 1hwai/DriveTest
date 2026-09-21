#pragma once

#include <SDL3/SDL.h>

class SimulationController;

class DebugUI {
public:
    DebugUI();

    bool Initialize(
        SDL_Window* window,
        SDL_GLContext context,
        SimulationController& simulation
    );

    void ProcessEvent(const SDL_Event& event);
    void Render();
    void Shutdown();

private:
    SimulationController* m_simulation;
    bool m_initialized;
    bool m_showSimulationWindow;
};