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

# LuaJIT for scripting (fallback to Lua 5.1 if not found)
pkg_check_modules(LUAJIT luajit)
if(NOT LUAJIT_FOUND)
    message(WARNING "LuaJIT not found, trying Lua 5.1...")
    pkg_check_modules(LUA REQUIRED lua5.1)
    set(LUA_LIBRARIES ${LUA_LIBRARIES})
    set(LUA_INCLUDE_DIRS ${LUA_INCLUDE_DIRS})
else()
    set(LUA_LIBRARIES ${LUAJIT_LIBRARIES})
    set(LUA_INCLUDE_DIRS ${LUAJIT_INCLUDE_DIRS})
    message(STATUS "Found LuaJIT: ${LUAJIT_LIBRARIES}")
endif()

# FetchContent for external dependencies
include(FetchContent)

# Jolt Physics
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

# Sol3 (header-only Lua C++ bindings)
FetchContent_Declare(
    sol2
    GIT_REPOSITORY https://github.com/ThePhD/sol2.git
    GIT_TAG v3.3.0
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(JoltPhysics sol2)

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
    src/render/texture.cpp
    src/render/line_render.cpp
    src/game/entity.cpp
    src/game/config_loader.cpp
    src/game/equipment_loader.cpp
    src/game/handling.cpp
    src/game/maneuver.cpp
    src/ui/ui_render.cpp
    src/ui/ui_text.cpp
    src/physics/jolt_physics.cpp
    src/script/reflex_script.cpp
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
    ${LUA_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(carwars
    Jolt
    sol2::sol2
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    ${LUA_LIBRARIES}
    m
)

# Jolt compile definitions
target_compile_definitions(carwars PRIVATE
    JPH_DEBUG_RENDERER
)

# Assets are at ../../assets relative to build directory
# No need to copy - code uses relative paths
