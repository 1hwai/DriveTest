#include "Renderer.h"
#include "Mesh.h"

#include <glad/gl.h>
#include <SDL3/SDL_opengl.h>
#include <iostream>

namespace {
    Mat3 GetNormalMatrix(const Mat4& model) {
        Mat3 upper;

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column)
                upper.m[row][column] = model.m[row][column];
        }

        return upper.Inversed().Transposed();
    }
}

Renderer::Renderer()
    : m_window(nullptr),
    m_context(nullptr) {}

Renderer::~Renderer() {
    Shutdown();
}

bool Renderer::Initialize() {
    if (m_window != nullptr)
        return false;

    if (!SDL_Init(SDL_INIT_VIDEO))
        return false;

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(
        SDL_GL_CONTEXT_PROFILE_MASK,
        SDL_GL_CONTEXT_PROFILE_CORE
    );

    m_window = SDL_CreateWindow(
        "DriveTest",
        1280,
        720,
        SDL_WINDOW_OPENGL
    );

    if (m_window == nullptr) {
        SDL_Quit();
        return false;
    }

    m_context = SDL_GL_CreateContext(m_window);

    if (m_context == nullptr) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;

        SDL_Quit();
        return false;
    }

    int version = gladLoadGL(
        (GLADloadfunc)SDL_GL_GetProcAddress
    );

    if (version == 0) {
        SDL_GL_DestroyContext(m_context);
        m_context = nullptr;

        SDL_DestroyWindow(m_window);
        m_window = nullptr;

        SDL_Quit();
        return false;
    }

    SDL_GL_SetSwapInterval(1);
    std::cout << "Initializing Renderer..." << std::endl;

    if (!m_shader.Load(
        "Shaders/basic.vert",
        "Shaders/basic.frag")) {
        std::cerr << "Failed to load shader." << std::endl;
        return false;
    }

    if (!m_debugRenderer.Initialize()) {
        std::cerr << "Failed to initialize debug renderer." << std::endl;
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    return true;
}

void Renderer::Render(const World& world) {
    if (m_context == nullptr)
        return;

    int width = 0;
    int height = 0;

    SDL_GetWindowSizeInPixels(
        m_window,
        &width,
        &height
    );

    glViewport(0, 0, width, height);

    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_shader.Use();

    const Mat4 view =
        world.GetCamera().GetViewMatrix();

    const Mat4 projection =
        world.GetCamera().GetProjectionMatrix();

    m_shader.SetMat4("uView", view);
    m_shader.SetMat4("uProjection", projection);
    m_shader.SetVec3(
        "uLightDirection",
        Vec3(-0.45f, -1.0f, -0.65f).Normalized()
    );
    m_shader.SetVec3(
        "uCameraPosition",
        world.GetCamera().GetPosition()
    );

    for (const auto& object : world.GetObjects()) {
        const Mesh* mesh = object->GetMesh();

        if (mesh == nullptr)
            continue;

        const Mat4 model =
            object->GetTransform().GetMatrix();

        m_shader.SetMat4("uModel", model);
        m_shader.SetMat3(
            "uNormalMatrix",
            GetNormalMatrix(model)
        );

        mesh->Draw();
    }

    m_debugRenderer.Render(world.GetCamera());
}

void Renderer::Present() {
    if (m_window == nullptr)
        return;

    SDL_GL_SwapWindow(m_window);
}

SDL_Window* Renderer::GetWindow() const {
    return m_window;
}

SDL_GLContext Renderer::GetContext() const {
    return m_context;
}

void Renderer::Shutdown() {
    if (m_context != nullptr) {
        m_debugRenderer.Shutdown();
        m_shader.Destroy();

        SDL_GL_DestroyContext(m_context);
        m_context = nullptr;
    }

    if (m_window != nullptr) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }

    SDL_Quit();
}