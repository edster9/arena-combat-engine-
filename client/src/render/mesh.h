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

// Box mesh renderer (shared shader for all boxes and loaded meshes)
typedef struct BoxRenderer {
    Shader shader;
    BoxMesh unit_box;  // 1x1x1 box centered at origin
    bool valid;
    // Cached uniform locations (avoid glGetUniformLocation every frame)
    GLint u_model;
    GLint u_view;
    GLint u_projection;
    GLint u_lightDir;
    GLint u_objectColor;
} BoxRenderer;

// Initialize the box renderer
bool box_renderer_init(BoxRenderer* r);
void box_renderer_destroy(BoxRenderer* r);

// Begin/end batch rendering
void box_renderer_begin(BoxRenderer* r, Mat4* view, Mat4* projection, Vec3 light_dir);

// Draw a box with position, size, and color
void box_renderer_draw(BoxRenderer* r, Vec3 pos, Vec3 size, Vec3 color);

// Draw a box with position, size, rotation (Y axis), and color
void box_renderer_draw_rotated(BoxRenderer* r, Vec3 pos, Vec3 size, float rotation_y, Vec3 color);

// Draw a box with position, size, full 3x3 rotation matrix (row-major), and color
void box_renderer_draw_rotated_matrix(BoxRenderer* r, Vec3 pos, Vec3 size, const float* rot_matrix, Vec3 color);

// Draw a loaded mesh (VAO) with position, scale, rotation and color
// rotation_y is in radians
void box_renderer_draw_mesh(BoxRenderer* r, GLuint vao, int vertex_count,
                            Vec3 pos, float scale, float rotation_y, Vec3 color);

// End rendering (resets OpenGL state)
void box_renderer_end(BoxRenderer* r);

#endif // MESH_H
