cmake_minimum_required(VERSION 3.20)

project(DriveTest LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SDL3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_executable(DriveTest
    main.cpp

    Core/Application.cpp
    Core/Time.cpp
    Core/SimulationController.cpp
    Core/Object.cpp
    Core/Debug/Logger.cpp
    Core/Debug/Performance.cpp
    Core/Debug/DebugUI.cpp

    Physics/RigidBody.cpp
    Physics/Collision.cpp
    Physics/Collider.cpp
    Physics/Contact.cpp
    Physics/Solver.cpp
    Physics/PhysicsWorld.cpp
    Physics/Terrain.cpp
    Physics/Material.cpp

    Vehicle/Car.cpp
    Vehicle/Wheel.cpp
    Vehicle/Suspension.cpp
    Vehicle/Tire.cpp
    Vehicle/Engine.cpp
    Vehicle/Transmission.cpp
    Vehicle/Differential.cpp

    Rendering/Renderer.cpp
    Rendering/Camera.cpp
    Rendering/DebugRenderer.cpp
    Rendering/Mesh.cpp
    Rendering/Shader.cpp
    Rendering/MeshManager.cpp

    Input/InputManager.cpp
    Input/Keyboard.cpp
    Input/WheelInput.cpp

    Core/Math/Vec3.cpp
    Core/Math/Mat4.cpp
    Core/Math/Transform.cpp
    Core/Math/Mat3.cpp
    Core/Math/Quaternion.cpp

    Core/Debug/Logger.cpp

    World/World.cpp
    World/SceneFactory.cpp
    World/SceneSerializer.cpp

    External/glad-generated/src/gl.c
)

target_link_libraries(DriveTest PRIVATE
    SDL3::SDL3
    imgui::imgui
)

add_custom_command(TARGET DriveTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/Shaders"
        "$<TARGET_FILE_DIR:DriveTest>/Shaders"
)

target_include_directories(DriveTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/External/glad-generated/include"
)

if(WIN32)
    target_link_libraries(DriveTest PRIVATE
        opengl32
    )
elseif(UNIX)
    target_link_libraries(DriveTest PRIVATE
        GL
    )
endif()