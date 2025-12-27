cmake_minimum_required(VERSION 3.16)
project(carwars C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-O2")

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# ODE - handle different library names on Linux vs MinGW
pkg_check_modules(ODE ode)
if(NOT ODE_FOUND)
    # MinGW uses ode_double
    pkg_check_modules(ODE ode-double)
    if(NOT ODE_FOUND)
        # Manual fallback for MinGW
        if(WIN32 OR MINGW)
            set(ODE_LIBRARIES ode_double)
            set(ODE_INCLUDE_DIRS "")
        else()
            message(FATAL_ERROR "ODE library not found")
        endif()
    endif()
endif()

# Source files
set(SOURCES
    src/main.c
    src/platform/platform.c
    src/math/vec3.c
    src/math/mat4.c
    src/render/camera.c
    src/render/shader.c
    src/render/floor.c
    src/render/mesh.c
    src/render/obj_loader.c
    src/render/line_render.c
    src/game/entity.c
    src/game/config_loader.c
    src/game/equipment_loader.c
    src/ui/ui_render.c
    src/ui/ui_text.c
    src/physics/ode_physics.c
    src/vendor/cJSON.c
)

# Executable
add_executable(carwars ${SOURCES})

# Include directories
target_include_directories(carwars PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${SDL2_INCLUDE_DIRS}
    ${ODE_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(carwars
    ${SDL2_LIBRARIES}
    ${ODE_LIBRARIES}
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    m
)

# Assets are at ../../assets relative to build directory
# No need to copy - code uses relative paths
