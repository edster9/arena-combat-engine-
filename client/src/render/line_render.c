#include "line_render.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Vertex shader - per-vertex color
static const char* line_vertex_shader =
    "#version 330 core\n"
    "layout(location = 0) in vec3 a_pos;\n"
    "layout(location = 1) in vec4 a_color;\n"
    "\n"
    "uniform mat4 u_view;\n"
    "uniform mat4 u_projection;\n"
    "\n"
    "out vec4 v_color;\n"
    "\n"
    "void main() {\n"
    "    gl_Position = u_projection * u_view * vec4(a_pos, 1.0);\n"
    "    v_color = a_color;\n"
    "}\n";

// Fragment shader - uses interpolated vertex color
static const char* line_fragment_shader =
    "#version 330 core\n"
    "in vec4 v_color;\n"
    "out vec4 frag_color;\n"
    "\n"
    "void main() {\n"
    "    frag_color = v_color;\n"
    "}\n";

static GLuint compile_shader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        fprintf(stderr, "Line Shader compile error: %s\n", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool line_renderer_init(LineRenderer* lr) {
    // Compile shaders
    GLuint vert = compile_shader(GL_VERTEX_SHADER, line_vertex_shader);
    if (!vert) return false;

    GLuint frag = compile_shader(GL_FRAGMENT_SHADER, line_fragment_shader);
    if (!frag) {
        glDeleteShader(vert);
        return false;
    }

    // Link program
    lr->shader_program = glCreateProgram();
    glAttachShader(lr->shader_program, vert);
    glAttachShader(lr->shader_program, frag);
    glLinkProgram(lr->shader_program);

    GLint success;
    glGetProgramiv(lr->shader_program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(lr->shader_program, sizeof(log), NULL, log);
        fprintf(stderr, "Line Shader link error: %s\n", log);
        glDeleteShader(vert);
        glDeleteShader(frag);
        glDeleteProgram(lr->shader_program);
        return false;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);

    // Get uniform locations
    lr->u_view = glGetUniformLocation(lr->shader_program, "u_view");
    lr->u_projection = glGetUniformLocation(lr->shader_program, "u_projection");

    // Create VAO/VBO for batched rendering
    glGenVertexArrays(1, &lr->vao);
    glGenBuffers(1, &lr->vbo);

    glBindVertexArray(lr->vao);
    glBindBuffer(GL_ARRAY_BUFFER, lr->vbo);

    // Allocate buffer for max vertices (will be filled dynamically)
    glBufferData(GL_ARRAY_BUFFER, MAX_LINE_VERTICES * sizeof(LineVertex), NULL, GL_DYNAMIC_DRAW);

    // Position attribute (location 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)0);

    // Color attribute (location 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);

    // Allocate CPU-side batch buffer
    lr->batch = (LineVertex*)malloc(MAX_LINE_VERTICES * sizeof(LineVertex));
    lr->batch_count = 0;
    lr->batch_capacity = MAX_LINE_VERTICES;

    printf("Line Renderer initialized (batched, max %d vertices)\n", MAX_LINE_VERTICES);
    return true;
}

void line_renderer_destroy(LineRenderer* lr) {
    if (lr->batch) {
        free(lr->batch);
        lr->batch = NULL;
    }
    if (lr->vao) glDeleteVertexArrays(1, &lr->vao);
    if (lr->vbo) glDeleteBuffers(1, &lr->vbo);
    if (lr->shader_program) glDeleteProgram(lr->shader_program);
}

void line_renderer_begin(LineRenderer* lr, Mat4* view, Mat4* projection) {
    // Reset batch
    lr->batch_count = 0;

    // Set up shader and uniforms (matrices don't change during batch)
    glUseProgram(lr->shader_program);
    glUniformMatrix4fv(lr->u_view, 1, GL_FALSE, view->m);
    glUniformMatrix4fv(lr->u_projection, 1, GL_FALSE, projection->m);
}

// Add a vertex to the batch
static void batch_add_vertex(LineRenderer* lr, float x, float y, float z,
                             float r, float g, float b, float a) {
    if (lr->batch_count >= lr->batch_capacity) {
        // Batch full - could flush here, but for now just skip
        return;
    }
    LineVertex* v = &lr->batch[lr->batch_count++];
    v->x = x; v->y = y; v->z = z;
    v->r = r; v->g = g; v->b = b; v->a = a;
}

void line_renderer_draw_line(LineRenderer* lr, Vec3 start, Vec3 end, Vec3 color, float alpha) {
    batch_add_vertex(lr, start.x, start.y, start.z, color.x, color.y, color.z, alpha);
    batch_add_vertex(lr, end.x, end.y, end.z, color.x, color.y, color.z, alpha);
}

void line_renderer_draw_path(LineRenderer* lr, Vec3* points, int count, Vec3 color, float alpha) {
    if (count < 2) return;

    // Draw as line strip converted to individual lines
    for (int i = 0; i < count - 1; i++) {
        line_renderer_draw_line(lr, points[i], points[i + 1], color, alpha);
    }
}

void line_renderer_draw_circle(LineRenderer* lr, Vec3 center, float radius, Vec3 color, float alpha) {
    const int segments = 12;

    for (int i = 0; i < segments; i++) {
        float a1 = (float)i / segments * 2.0f * M_PI;
        float a2 = (float)((i + 1) % segments) / segments * 2.0f * M_PI;

        Vec3 p1 = {center.x + cosf(a1) * radius, center.y, center.z + sinf(a1) * radius};
        Vec3 p2 = {center.x + cosf(a2) * radius, center.y, center.z + sinf(a2) * radius};

        line_renderer_draw_line(lr, p1, p2, color, alpha);
    }
}

void line_renderer_end(LineRenderer* lr) {
    if (lr->batch_count == 0) {
        glUseProgram(0);
        return;
    }

    // Enable blending for alpha
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(2.0f);

    // Upload ALL vertices in ONE call
    glBindVertexArray(lr->vao);
    glBindBuffer(GL_ARRAY_BUFFER, lr->vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, lr->batch_count * sizeof(LineVertex), lr->batch);

    // Draw ALL lines in ONE call
    glDrawArrays(GL_LINES, 0, lr->batch_count);

    // Cleanup
    glBindVertexArray(0);
    glUseProgram(0);
    glLineWidth(1.0f);
}
