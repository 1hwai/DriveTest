cmake_minimum_required(VERSION 3.20)

project(DriveTest LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SDL3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_path(STB_IMAGE_INCLUDE_DIR stb_image.h REQUIRED)

add_executable(DriveTest
    main.cpp

    Audio/AudioSystem.cpp

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
    Physics/Road.cpp
    Physics/Material.cpp

    Vehicle/Car.cpp
    Vehicle/VehicleConfig.cpp
    Vehicle/Powertrain.cpp
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
    Input/GamepadInput.cpp

    Core/Math/Vec3.cpp
    Core/Math/Mat4.cpp
    Core/Math/Transform.cpp
    Core/Math/Mat3.cpp
    Core/Math/Quaternion.cpp

    Core/Debug/Logger.cpp

    World/Scene.cpp
    World/RoadsideScenery.cpp
    World/TestTrack.cpp
    World/SceneFactory.cpp
    World/SceneSerializer.cpp

    External/glad-generated/src/gl.c
)

target_compile_definitions(DriveTest PRIVATE
    DRIVETEST_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(DriveTest PRIVATE
    SDL3::SDL3
    imgui::imgui
    assimp::assimp
)

add_custom_command(TARGET DriveTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/Shaders"
        "$<TARGET_FILE_DIR:DriveTest>/Shaders"
)

target_include_directories(DriveTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/External/glad-generated/include"
    "${STB_IMAGE_INCLUDE_DIR}"
)


option(
    DRIVETEST_BUILD_PHYSICS_DIAGNOSTICS
    "Build the headless physics diagnostics executable"
    OFF
)

if(DRIVETEST_BUILD_PHYSICS_DIAGNOSTICS)
    add_executable(DriveTestPhysicsDiagnostics
        Tests/PhysicsDiagnostics.cpp

        Core/Debug/Logger.cpp

        Physics/RigidBody.cpp
        Physics/Collision.cpp
        Physics/Collider.cpp
        Physics/Contact.cpp
        Physics/Solver.cpp
        Physics/PhysicsWorld.cpp
        Physics/Terrain.cpp
        Physics/Road.cpp
        Physics/Material.cpp

        Vehicle/Car.cpp
        Vehicle/Powertrain.cpp
        Vehicle/Wheel.cpp
        Vehicle/Suspension.cpp
        Vehicle/Tire.cpp
        Vehicle/Engine.cpp
        Vehicle/Transmission.cpp
        Vehicle/Differential.cpp

        Core/Math/Vec3.cpp
        Core/Math/Mat3.cpp
        Core/Math/Mat4.cpp
        Core/Math/Quaternion.cpp
        Core/Math/Transform.cpp
    )

    target_include_directories(
        DriveTestPhysicsDiagnostics
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
    )

    target_compile_definitions(DriveTestPhysicsDiagnostics PRIVATE
        DRIVETEST_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
    )

    enable_testing()

    add_test(
        NAME PhysicsDiagnostics
        COMMAND DriveTestPhysicsDiagnostics
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
endif()


option(
    DRIVETEST_BUILD_VEHICLE_DIAGNOSTICS
    "Build the headless vehicle diagnostics executable"
    OFF
)

if(DRIVETEST_BUILD_VEHICLE_DIAGNOSTICS)
    add_executable(DriveTestVehicleDiagnostics
        Tests/VehicleDiagnostics.cpp

        Core/Debug/Logger.cpp

        Physics/RigidBody.cpp
        Physics/Collision.cpp
        Physics/Collider.cpp
        Physics/Contact.cpp
        Physics/Solver.cpp
        Physics/PhysicsWorld.cpp
        Physics/Terrain.cpp
        Physics/Road.cpp
        Physics/Material.cpp

        Vehicle/Car.cpp
        Vehicle/VehicleConfig.cpp
        Vehicle/Powertrain.cpp
        Vehicle/Wheel.cpp
        Vehicle/Suspension.cpp
        Vehicle/Tire.cpp
        Vehicle/Engine.cpp
        Vehicle/Transmission.cpp
        Vehicle/Differential.cpp

        Core/Math/Vec3.cpp
        Core/Math/Mat3.cpp
        Core/Math/Mat4.cpp
        Core/Math/Quaternion.cpp
        Core/Math/Transform.cpp
    )

    target_include_directories(
        DriveTestVehicleDiagnostics
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
    )

    target_compile_definitions(DriveTestVehicleDiagnostics PRIVATE
        DRIVETEST_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
    )

    enable_testing()

    add_test(
        NAME VehicleDiagnostics
        COMMAND DriveTestVehicleDiagnostics
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
endif()


if(WIN32)
    target_link_libraries(DriveTest PRIVATE
        opengl32
    )
elseif(UNIX)
    target_link_libraries(DriveTest PRIVATE
        GL
    )
endif()
