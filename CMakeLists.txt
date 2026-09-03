cmake_minimum_required(VERSION 3.20)

project(DriveTest LANGUAGES C CXX)

find_package(SDL3 CONFIG REQUIRED)

add_executable(DriveTest
    main.cpp

    Core/Application.cpp
    Core/Time.cpp
    Core/Object.cpp

    Physics/RigidBody.cpp
    Physics/Collision.cpp
    Physics/Contact.cpp
    Physics/Solver.cpp
    Physics/PhysicsWorld.cpp

    Vehicle/Car.cpp
    Vehicle/Wheel.cpp
    Vehicle/Suspension.cpp
    Vehicle/Tire.cpp
    Vehicle/Engine.cpp
    Vehicle/Transmission.cpp
    Vehicle/Differential.cpp

    Rendering/Renderer.cpp
    Rendering/Camera.cpp
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

    Core/Debug/Logger.cpp

    World/World.cpp


    External/glad-generated/src/gl.c
 "Core/Math/Quaternion.cpp")

add_custom_command(TARGET DriveTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/Shaders"
        "$<TARGET_FILE_DIR:DriveTest>/Shaders"
)

target_include_directories(DriveTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/External/glad-generated/include"
)

target_link_libraries(DriveTest PRIVATE
    SDL3::SDL3
    opengl32
)