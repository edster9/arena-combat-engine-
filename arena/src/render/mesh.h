#ifndef MESH_H
#define MESH_H

#include <stdbool.h>
#include <GL/glew.h>
#include "shader.h"
#include "../math/mat4.h"
#include "../math/vec3.h"

// A simple box mesh with lighting
typedef struct BoxMesh {
    GLuint vao;
    GLuint vbo;
    int vertex_count;
} BoxMesh;

// Box mesh renderer (shared shader for all boxes)
typedef struct BoxRenderer {
    Shader shader;
    BoxMesh unit_box;  // 1x1x1 box centered at origin
    bool valid;
} BoxRenderer;

// Initialize the box renderer
bool box_renderer_init(BoxRenderer* r);
void box_renderer_destroy(BoxRenderer* r);

// Begin/end batch rendering
void box_renderer_begin(BoxRenderer* r, Mat4* view, Mat4* projection, Vec3 light_dir);

// Draw a box with position, size, and color
void box_renderer_draw(BoxRenderer* r, Vec3 pos, Vec3 size, Vec3 color);

// End rendering (resets OpenGL state)
void box_renderer_end(BoxRenderer* r);

#endif // MESH_H
