#include "Renderer.h"
#include "Mesh.h"

#include <glad/gl.h>
#include <SDL3/SDL_opengl.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "../Core/Math/Transform.h"

namespace {
    struct Plane {
        float a;
        float b;
        float c;
        float d;
    };

    Mat3 GetNormalMatrix(const Mat4& model) {
        Mat3 upper;

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column)
                upper.m[row][column] = model.m[row][column];
        }

        return upper.Inversed().Transposed();
    }

    Plane MakePlane(const Mat4& matrix, int row, float sign) {
        return {
            matrix.m[3][0] + sign * matrix.m[row][0],
            matrix.m[3][1] + sign * matrix.m[row][1],
            matrix.m[3][2] + sign * matrix.m[row][2],
            matrix.m[3][3] + sign * matrix.m[row][3]
        };
    }

    bool IsSphereInsideFrustum(
        const Mat4& viewProjection,
        const Vec3& center,
        float radius
    ) {
        const Plane planes[] = {
            MakePlane(viewProjection, 0, 1.0f),
            MakePlane(viewProjection, 0, -1.0f),
            MakePlane(viewProjection, 1, 1.0f),
            MakePlane(viewProjection, 1, -1.0f),
            MakePlane(viewProjection, 2, 1.0f),
            MakePlane(viewProjection, 2, -1.0f)
        };

        for (const Plane& plane : planes) {
            const float length = std::sqrt(
                plane.a * plane.a +
                plane.b * plane.b +
                plane.c * plane.c
            );

            if (length <= 0.000001f)
                continue;

            const float distance =
                plane.a * center.x +
                plane.b * center.y +
                plane.c * center.z +
                plane.d;

            if (distance < -radius * length)
                return false;
        }

        return true;
    }

    float GetCullRadius(const Object& object) {
        const Vec3& scale = object.GetTransform().scale;
        const float scaleRadius = std::max({
            std::abs(scale.x),
            std::abs(scale.y),
            std::abs(scale.z)
        });

        switch (object.GetRenderSurface()) {
        case RenderSurface::Terrain:
            return 1000.0f;
        case RenderSurface::Tarmac:
        case RenderSurface::Gravel:
        case RenderSurface::Transition:
            return 100.0f;
        default:
            return std::max(3.0f, scaleRadius * 3.0f);
        }
    }
}

Renderer::Renderer()
    : m_window(nullptr),
    m_context(nullptr),
    m_fullscreen(false) {}

Renderer::~Renderer() {
    Shutdown();
}

bool Renderer::Initialize() {
    if (m_window != nullptr)
        return false;

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
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
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
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

void Renderer::Render(const Scene& world) {
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

    glClearColor(0.38f, 0.67f, 0.94f, 1.0f);
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

    const Mat4 viewProjection = projection * view;
    const Vec3 cameraPosition = world.GetCamera().GetPosition();
    constexpr float MaxRenderDistance = 1500.0f;

    for (const auto& object : world.GetObjects()) {
        const Mesh* mesh = object->GetMesh();

        if (mesh == nullptr)
            continue;

        const Transform& objectTransform = object->GetTransform();
        const Vec3 toObject = objectTransform.position - cameraPosition;
        const float distanceSquared = toObject.LengthSquared();
        const float cullRadius = GetCullRadius(*object);
        const RenderSurface surface = object->GetRenderSurface();
        const bool largeWorldSurface =
            surface == RenderSurface::Terrain ||
            surface == RenderSurface::Tarmac ||
            surface == RenderSurface::Gravel ||
            surface == RenderSurface::Transition;

        if (!largeWorldSurface) {
            const float maxDistance = MaxRenderDistance + cullRadius;

            if (distanceSquared > maxDistance * maxDistance)
                continue;

            if (!IsSphereInsideFrustum(
                viewProjection,
                objectTransform.position,
                cullRadius
            ))
                continue;
        }

        Transform renderTransform = objectTransform;
        if (object->IsBillboard()) {
            const Vec3 toCamera = cameraPosition - renderTransform.position;
            const float yaw = std::atan2(toCamera.x, toCamera.z);
            renderTransform.rotation = Quaternion::FromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), yaw);
        }
        const Mat4 model = renderTransform.GetMatrix();

        m_shader.SetMat4("uModel", model);
        m_shader.SetMat3(
            "uNormalMatrix",
            GetNormalMatrix(model)
        );

        m_shader.SetInt("uSurfaceType", static_cast<int>(object->GetRenderSurface()));
        m_shader.SetFloat("uSurfaceBlend", object->GetSurfaceBlend());
        mesh->Draw(m_shader, object->GetColor());
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

void Renderer::SetFullscreen(bool fullscreen) {
    if (m_window == nullptr || m_fullscreen == fullscreen)
        return;

    if (SDL_SetWindowFullscreen(m_window, fullscreen))
        m_fullscreen = fullscreen;
}

bool Renderer::IsFullscreen() const {
    return m_fullscreen;
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