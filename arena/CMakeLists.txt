cmake_minimum_required(VERSION 3.16)
project(arena C)

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
    src/game/entity.c
    src/ui/ui_render.c
    src/ui/ui_text.c
)

# Executable
add_executable(arena ${SOURCES})

# Include directories
target_include_directories(arena PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${SDL2_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(arena
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    m
)

# Copy assets to build directory (assets is at project root level)
file(COPY ${CMAKE_SOURCE_DIR}/../assets DESTINATION ${CMAKE_BINARY_DIR})
