#include "floor.h"
#include <stdio.h>

// Vertex shader - simple pass-through with world position
static const char* floor_vert_src =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "out vec3 worldPos;\n"
    "uniform mat4 view;\n"
    "uniform mat4 projection;\n"
    "void main() {\n"
    "    worldPos = aPos;\n"
    "    gl_Position = projection * view * vec4(aPos, 1.0);\n"
    "}\n";

// Fragment shader - procedural grid with concrete look
static const char* floor_frag_src =
    "#version 330 core\n"
    "in vec3 worldPos;\n"
    "out vec4 FragColor;\n"
    "uniform float gridSize;\n"
    "uniform vec3 cameraPos;\n"
    "\n"
    "// Simple hash for noise\n"
    "float hash(vec2 p) {\n"
    "    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);\n"
    "}\n"
    "\n"
    "// Value noise\n"
    "float noise(vec2 p) {\n"
    "    vec2 i = floor(p);\n"
    "    vec2 f = fract(p);\n"
    "    f = f * f * (3.0 - 2.0 * f);\n"
    "    float a = hash(i);\n"
    "    float b = hash(i + vec2(1.0, 0.0));\n"
    "    float c = hash(i + vec2(0.0, 1.0));\n"
    "    float d = hash(i + vec2(1.0, 1.0));\n"
    "    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    // Base concrete color with noise variation\n"
    "    float n = noise(worldPos.xz * 0.5) * 0.1;\n"
    "    float n2 = noise(worldPos.xz * 2.0) * 0.05;\n"
    "    vec3 concrete = vec3(0.35, 0.35, 0.38) + n + n2;\n"
    "\n"
    "    // Grid lines\n"
    "    vec2 grid = abs(fract(worldPos.xz / gridSize - 0.5) - 0.5) / fwidth(worldPos.xz / gridSize);\n"
    "    float line = min(grid.x, grid.y);\n"
    "    float gridLine = 1.0 - min(line, 1.0);\n"
    "\n"
    "    // Thicker lines every 5 units\n"
    "    vec2 majorGrid = abs(fract(worldPos.xz / (gridSize * 5.0) - 0.5) - 0.5) / fwidth(worldPos.xz / (gridSize * 5.0));\n"
    "    float majorLine = min(majorGrid.x, majorGrid.y);\n"
    "    float majorGridLine = 1.0 - min(majorLine * 0.5, 1.0);\n"
    "\n"
    "    // Axis lines (at origin)\n"
    "    float xAxis = 1.0 - min(abs(worldPos.z) / fwidth(worldPos.z) * 0.3, 1.0);\n"
    "    float zAxis = 1.0 - min(abs(worldPos.x) / fwidth(worldPos.x) * 0.3, 1.0);\n"
    "\n"
    "    // Combine colors\n"
    "    vec3 color = concrete;\n"
    "    color = mix(color, vec3(0.25, 0.25, 0.28), gridLine * 0.5);      // Minor grid\n"
    "    color = mix(color, vec3(0.2, 0.2, 0.23), majorGridLine * 0.7);   // Major grid\n"
    "    color = mix(color, vec3(0.8, 0.2, 0.2), xAxis * 0.8);            // X axis (red)\n"
    "    color = mix(color, vec3(0.2, 0.2, 0.8), zAxis * 0.8);            // Z axis (blue)\n"
    "\n"
    "    // Distance fog\n"
    "    float dist = length(worldPos.xz - cameraPos.xz);\n"
    "    float fog = 1.0 - exp(-dist * 0.008);\n"
    "    color = mix(color, vec3(0.15, 0.15, 0.18), fog);\n"
    "\n"
    "    FragColor = vec4(color, 1.0);\n"
    "}\n";

bool floor_init(Floor* f, float size, float grid_size) {
    f->size = size;
    f->grid_size = grid_size;

    // Create shader
    if (!shader_create(&f->shader, floor_vert_src, floor_frag_src)) {
        fprintf(stderr, "Failed to create floor shader\n");
        return false;
    }

    // Create floor quad vertices (large plane)
    float half = size / 2.0f;
    float vertices[] = {
        // Position
        -half, 0.0f, -half,
         half, 0.0f, -half,
         half, 0.0f,  half,

        -half, 0.0f, -half,
         half, 0.0f,  half,
        -half, 0.0f,  half,
    };

    // Create VAO and VBO
    glGenVertexArrays(1, &f->vao);
    glGenBuffers(1, &f->vbo);

    glBindVertexArray(f->vao);

    glBindBuffer(GL_ARRAY_BUFFER, f->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    return true;
}

void floor_destroy(Floor* f) {
    shader_destroy(&f->shader);
    glDeleteVertexArrays(1, &f->vao);
    glDeleteBuffers(1, &f->vbo);
}

void floor_render(Floor* f, Mat4* view, Mat4* projection, Vec3 camera_pos) {
    shader_use(&f->shader);
    shader_set_mat4(&f->shader, "view", view);
    shader_set_mat4(&f->shader, "projection", projection);
    shader_set_float(&f->shader, "gridSize", f->grid_size);
    shader_set_vec3(&f->shader, "cameraPos", camera_pos);

    glBindVertexArray(f->vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    // Reset to no shader (for legacy rendering)
    glUseProgram(0);
}
