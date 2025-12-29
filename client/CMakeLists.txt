cmake_minimum_required(VERSION 3.16)
project(carwars CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Jolt Physics via FetchContent
include(FetchContent)
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.2.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build
)

# Jolt build options
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(JoltPhysics)

# Source files
set(SOURCES
    src/main.cpp
    src/platform/platform.cpp
    src/math/vec3.cpp
    src/math/mat4.cpp
    src/render/camera.cpp
    src/render/shader.cpp
    src/render/floor.cpp
    src/render/mesh.cpp
    src/render/obj_loader.cpp
    src/render/line_render.cpp
    src/game/entity.cpp
    src/game/config_loader.cpp
    src/game/equipment_loader.cpp
    src/game/handling.cpp
    src/game/maneuver.cpp
    src/ui/ui_render.cpp
    src/ui/ui_text.cpp
    src/physics/jolt_physics.cpp
    src/vendor/cJSON.cpp
)

# Executable
add_executable(carwars ${SOURCES})

# Include directories
target_include_directories(carwars PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${SDL2_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(carwars
    Jolt
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    m
)

# Jolt compile definitions
target_compile_definitions(carwars PRIVATE
    JPH_PROFILE_ENABLED
    JPH_DEBUG_RENDERER
)

# Assets are at ../../assets relative to build directory
# No need to copy - code uses relative paths
