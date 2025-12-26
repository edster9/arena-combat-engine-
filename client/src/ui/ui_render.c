#include "ui_render.h"
#include "../math/mat4.h"
#include <stdio.h>
#include <string.h>

// Vertex shader for UI rendering
static const char* ui_vertex_shader =
    "#version 330 core\n"
    "layout(location = 0) in vec2 a_pos;\n"
    "\n"
    "uniform mat4 u_projection;\n"
    "uniform vec4 u_rect;  // x, y, width, height\n"
    "\n"
    "out vec2 v_local_pos;  // Position within rect (0-1)\n"
    "\n"
    "void main() {\n"
    "    // a_pos is 0-1, scale to rect size and position\n"
    "    vec2 screen_pos = u_rect.xy + a_pos * u_rect.zw;\n"
    "    gl_Position = u_projection * vec4(screen_pos, 0.0, 1.0);\n"
    "    v_local_pos = a_pos;\n"
    "}\n";

// Fragment shader for UI rendering (with rounded corners and border)
static const char* ui_fragment_shader =
    "#version 330 core\n"
    "in vec2 v_local_pos;\n"
    "\n"
    "uniform vec4 u_color;\n"
    "uniform vec4 u_border_color;\n"
    "uniform float u_border_width;\n"
    "uniform float u_corner_radius;\n"
    "uniform vec4 u_rect;  // x, y, width, height\n"
    "\n"
    "out vec4 frag_color;\n"
    "\n"
    "float rounded_box_sdf(vec2 center_pos, vec2 size, float radius) {\n"
    "    vec2 q = abs(center_pos) - size + radius;\n"
    "    return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - radius;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    vec2 size = u_rect.zw;\n"
    "    vec2 half_size = size * 0.5;\n"
    "    \n"
    "    // Convert local_pos (0-1) to center-relative coords\n"
    "    vec2 center_pos = (v_local_pos - 0.5) * size;\n"
    "    \n"
    "    // Clamp corner radius to not exceed half the smallest dimension\n"
    "    float radius = min(u_corner_radius, min(half_size.x, half_size.y));\n"
    "    \n"
    "    // Calculate SDF for outer edge\n"
    "    float dist = rounded_box_sdf(center_pos, half_size, radius);\n"
    "    \n"
    "    // Anti-aliased edge\n"
    "    float aa = 1.0;  // 1 pixel of anti-aliasing\n"
    "    float alpha = 1.0 - smoothstep(-aa, aa, dist);\n"
    "    \n"
    "    if (alpha < 0.001) discard;\n"
    "    \n"
    "    // Border: check if we're in the border region\n"
    "    vec4 final_color = u_color;\n"
    "    if (u_border_width > 0.0) {\n"
    "        float inner_dist = rounded_box_sdf(center_pos, half_size - u_border_width, max(0.0, radius - u_border_width));\n"
    "        float border_blend = smoothstep(-aa, aa, inner_dist);\n"
    "        final_color = mix(u_color, u_border_color, border_blend);\n"
    "    }\n"
    "    \n"
    "    frag_color = vec4(final_color.rgb, final_color.a * alpha);\n"
    "}\n";

// Compile a shader
static GLuint compile_shader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        fprintf(stderr, "UI Shader compile error: %s\n", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool ui_renderer_init(UIRenderer* ui) {
    memset(ui, 0, sizeof(UIRenderer));

    // Compile shaders
    GLuint vert = compile_shader(GL_VERTEX_SHADER, ui_vertex_shader);
    if (!vert) return false;

    GLuint frag = compile_shader(GL_FRAGMENT_SHADER, ui_fragment_shader);
    if (!frag) {
        glDeleteShader(vert);
        return false;
    }

    // Link program
    ui->shader_program = glCreateProgram();
    glAttachShader(ui->shader_program, vert);
    glAttachShader(ui->shader_program, frag);
    glLinkProgram(ui->shader_program);

    GLint success;
    glGetProgramiv(ui->shader_program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(ui->shader_program, sizeof(log), NULL, log);
        fprintf(stderr, "UI Shader link error: %s\n", log);
        glDeleteShader(vert);
        glDeleteShader(frag);
        glDeleteProgram(ui->shader_program);
        return false;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);

    // Get uniform locations
    ui->u_projection = glGetUniformLocation(ui->shader_program, "u_projection");
    ui->u_rect = glGetUniformLocation(ui->shader_program, "u_rect");
    ui->u_color = glGetUniformLocation(ui->shader_program, "u_color");
    ui->u_border_color = glGetUniformLocation(ui->shader_program, "u_border_color");
    ui->u_border_width = glGetUniformLocation(ui->shader_program, "u_border_width");
    ui->u_corner_radius = glGetUniformLocation(ui->shader_program, "u_corner_radius");

    // Create VAO/VBO for a unit quad (0,0) to (1,1)
    float quad_vertices[] = {
        // Triangle 1
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        // Triangle 2
        0.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
    };

    glGenVertexArrays(1, &ui->vao);
    glGenBuffers(1, &ui->vbo);

    glBindVertexArray(ui->vao);
    glBindBuffer(GL_ARRAY_BUFFER, ui->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertices), quad_vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    glBindVertexArray(0);

    printf("UI Renderer initialized\n");
    return true;
}

void ui_renderer_destroy(UIRenderer* ui) {
    if (ui->vao) glDeleteVertexArrays(1, &ui->vao);
    if (ui->vbo) glDeleteBuffers(1, &ui->vbo);
    if (ui->shader_program) glDeleteProgram(ui->shader_program);
    memset(ui, 0, sizeof(UIRenderer));
}

void ui_renderer_begin(UIRenderer* ui, int screen_width, int screen_height) {
    ui->screen_width = screen_width;
    ui->screen_height = screen_height;

    // Set up orthographic projection (origin at top-left, Y down)
    Mat4 projection = mat4_ortho(0, (float)screen_width, (float)screen_height, 0, -1, 1);

    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Disable depth test for 2D UI
    glDisable(GL_DEPTH_TEST);

    // Use shader
    glUseProgram(ui->shader_program);
    glUniformMatrix4fv(ui->u_projection, 1, GL_FALSE, projection.m);

    // Bind VAO
    glBindVertexArray(ui->vao);
}

void ui_renderer_end(UIRenderer* ui) {
    (void)ui;
    glBindVertexArray(0);
    glUseProgram(0);

    // Restore depth test
    glEnable(GL_DEPTH_TEST);
}

void ui_draw_rect(UIRenderer* ui, UIRect rect, UIColor color) {
    glUniform4f(ui->u_rect, rect.x, rect.y, rect.width, rect.height);
    glUniform4f(ui->u_color, color.r, color.g, color.b, color.a);
    glUniform4f(ui->u_border_color, 0, 0, 0, 0);
    glUniform1f(ui->u_border_width, 0.0f);
    glUniform1f(ui->u_corner_radius, 0.0f);

    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void ui_draw_rect_bordered(UIRenderer* ui, UIRect rect, UIColor fill_color,
                           UIColor border_color, float border_width) {
    glUniform4f(ui->u_rect, rect.x, rect.y, rect.width, rect.height);
    glUniform4f(ui->u_color, fill_color.r, fill_color.g, fill_color.b, fill_color.a);
    glUniform4f(ui->u_border_color, border_color.r, border_color.g, border_color.b, border_color.a);
    glUniform1f(ui->u_border_width, border_width);
    glUniform1f(ui->u_corner_radius, 0.0f);

    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void ui_draw_rect_rounded(UIRenderer* ui, UIRect rect, UIColor color, float corner_radius) {
    glUniform4f(ui->u_rect, rect.x, rect.y, rect.width, rect.height);
    glUniform4f(ui->u_color, color.r, color.g, color.b, color.a);
    glUniform4f(ui->u_border_color, 0, 0, 0, 0);
    glUniform1f(ui->u_border_width, 0.0f);
    glUniform1f(ui->u_corner_radius, corner_radius);

    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void ui_draw_panel(UIRenderer* ui, UIRect rect, UIColor fill_color,
                   UIColor border_color, float border_width, float corner_radius) {
    glUniform4f(ui->u_rect, rect.x, rect.y, rect.width, rect.height);
    glUniform4f(ui->u_color, fill_color.r, fill_color.g, fill_color.b, fill_color.a);
    glUniform4f(ui->u_border_color, border_color.r, border_color.g, border_color.b, border_color.a);
    glUniform1f(ui->u_border_width, border_width);
    glUniform1f(ui->u_corner_radius, corner_radius);

    glDrawArrays(GL_TRIANGLES, 0, 6);
}
