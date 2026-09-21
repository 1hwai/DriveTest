#pragma once

#include <SDL3/SDL.h>

#include "Shader.h"
#include "../World/World.h"

class Renderer {
public:
    Renderer();
    ~Renderer();

    bool Initialize();
    void Render(const World& world);
    void Present();
    void Shutdown();

    SDL_Window* GetWindow() const;
    SDL_GLContext GetContext() const;

private:
    SDL_Window* m_window;
    SDL_GLContext m_context;

    Shader m_shader;
};