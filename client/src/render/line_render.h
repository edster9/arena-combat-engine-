#ifndef LINE_RENDER_H
#define LINE_RENDER_H

#include <GL/glew.h>
#include <stdbool.h>
#include "../math/vec3.h"
#include "../math/mat4.h"

/*
 * Line Renderer (Batched)
 * =======================
 * Draws lines and paths in 3D space with efficient batching.
 * All lines are collected during draw calls and rendered in one batch at end().
 */

// Maximum vertices in batch (each line = 2 vertices)
// 4096 vertices = 2048 lines, enough for complex scenes
#define MAX_LINE_VERTICES 4096

// Vertex with color (position + RGBA)
typedef struct {
    float x, y, z;
    float r, g, b, a;
} LineVertex;

typedef struct LineRenderer {
    GLuint shader_program;
    GLuint vao;
    GLuint vbo;

    // Uniform locations
    GLint u_view;
    GLint u_projection;

    // Batch buffer
    LineVertex* batch;
    int batch_count;
    int batch_capacity;
} LineRenderer;

// Initialize the line renderer
bool line_renderer_init(LineRenderer* lr);

// Cleanup
void line_renderer_destroy(LineRenderer* lr);

// Begin rendering lines (call once per frame before drawing)
void line_renderer_begin(LineRenderer* lr, Mat4* view, Mat4* projection);

// Draw a line between two points (batched - actual draw happens at end)
void line_renderer_draw_line(LineRenderer* lr, Vec3 start, Vec3 end, Vec3 color, float alpha);

// Draw a path (series of connected points)
void line_renderer_draw_path(LineRenderer* lr, Vec3* points, int count, Vec3 color, float alpha);

// Draw a circle on the ground (for waypoints/markers)
void line_renderer_draw_circle(LineRenderer* lr, Vec3 center, float radius, Vec3 color, float alpha);

// End rendering - flushes all batched lines in ONE draw call
void line_renderer_end(LineRenderer* lr);

#endif // LINE_RENDER_H
