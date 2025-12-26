#define STB_TRUETYPE_IMPLEMENTATION
#include "../vendor/stb_truetype.h"

#include "ui_text.h"
#include "../math/mat4.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Vertex shader for text rendering
static const char* text_vertex_shader =
    "#version 330 core\n"
    "layout(location = 0) in vec2 a_pos;\n"
    "layout(location = 1) in vec2 a_uv;\n"
    "\n"
    "uniform mat4 u_projection;\n"
    "\n"
    "out vec2 v_uv;\n"
    "\n"
    "void main() {\n"
    "    gl_Position = u_projection * vec4(a_pos, 0.0, 1.0);\n"
    "    v_uv = a_uv;\n"
    "}\n";

// Fragment shader for text rendering
static const char* text_fragment_shader =
    "#version 330 core\n"
    "in vec2 v_uv;\n"
    "\n"
    "uniform sampler2D u_texture;\n"
    "uniform vec4 u_color;\n"
    "\n"
    "out vec4 frag_color;\n"
    "\n"
    "void main() {\n"
    "    float alpha = texture(u_texture, v_uv).r;\n"
    "    frag_color = vec4(u_color.rgb, u_color.a * alpha);\n"
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
        fprintf(stderr, "Text Shader compile error: %s\n", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool text_renderer_init(TextRenderer* tr, const char* font_path, float font_size) {
    memset(tr, 0, sizeof(TextRenderer));
    tr->font_size = font_size;

    // Read font file
    FILE* f = fopen(font_path, "rb");
    if (!f) {
        fprintf(stderr, "Failed to open font: %s\n", font_path);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    unsigned char* font_data = malloc(file_size);
    if (!font_data) {
        fclose(f);
        return false;
    }
    fread(font_data, 1, file_size, f);
    fclose(f);

    // Initialize stb_truetype
    stbtt_fontinfo font;
    if (!stbtt_InitFont(&font, font_data, 0)) {
        fprintf(stderr, "Failed to init font\n");
        free(font_data);
        return false;
    }

    // Get font metrics
    float scale = stbtt_ScaleForPixelHeight(&font, font_size);
    int ascent, descent, line_gap;
    stbtt_GetFontVMetrics(&font, &ascent, &descent, &line_gap);
    tr->ascent = ascent * scale;
    tr->descent = descent * scale;
    tr->line_height = (ascent - descent + line_gap) * scale;

    // Create atlas - start with reasonable size
    tr->atlas_width = 512;
    tr->atlas_height = 512;
    unsigned char* atlas = calloc(tr->atlas_width * tr->atlas_height, 1);

    // Pack characters into atlas
    int x = 1, y = 1;
    int row_height = 0;

    for (int i = 0; i < FONT_ATLAS_NUM_CHARS; i++) {
        int c = FONT_ATLAS_FIRST_CHAR + i;

        int w, h, xoff, yoff;
        unsigned char* bitmap = stbtt_GetCodepointBitmap(&font, 0, scale, c, &w, &h, &xoff, &yoff);

        // Check if we need to move to next row
        if (x + w + 1 >= tr->atlas_width) {
            x = 1;
            y += row_height + 1;
            row_height = 0;
        }

        // Check if atlas is too small
        if (y + h + 1 >= tr->atlas_height) {
            fprintf(stderr, "Font atlas too small!\n");
            if (bitmap) stbtt_FreeBitmap(bitmap, NULL);
            free(atlas);
            free(font_data);
            return false;
        }

        // Copy glyph to atlas
        if (bitmap) {
            for (int row = 0; row < h; row++) {
                memcpy(atlas + (y + row) * tr->atlas_width + x,
                       bitmap + row * w, w);
            }
            stbtt_FreeBitmap(bitmap, NULL);
        }

        // Store character info
        tr->chars[i].x0 = (float)xoff;
        tr->chars[i].y0 = (float)yoff;
        tr->chars[i].x1 = (float)(xoff + w);
        tr->chars[i].y1 = (float)(yoff + h);

        tr->chars[i].u0 = (float)x / tr->atlas_width;
        tr->chars[i].v0 = (float)y / tr->atlas_height;
        tr->chars[i].u1 = (float)(x + w) / tr->atlas_width;
        tr->chars[i].v1 = (float)(y + h) / tr->atlas_height;

        int advance, lsb;
        stbtt_GetCodepointHMetrics(&font, c, &advance, &lsb);
        tr->chars[i].advance = advance * scale;

        // Update position
        if (h > row_height) row_height = h;
        x += w + 1;
    }

    free(font_data);

    // Create OpenGL texture
    glGenTextures(1, &tr->texture);
    glBindTexture(GL_TEXTURE_2D, tr->texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, tr->atlas_width, tr->atlas_height,
                 0, GL_RED, GL_UNSIGNED_BYTE, atlas);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    free(atlas);

    // Compile shaders
    GLuint vert = compile_shader(GL_VERTEX_SHADER, text_vertex_shader);
    if (!vert) return false;

    GLuint frag = compile_shader(GL_FRAGMENT_SHADER, text_fragment_shader);
    if (!frag) {
        glDeleteShader(vert);
        return false;
    }

    // Link program
    tr->shader_program = glCreateProgram();
    glAttachShader(tr->shader_program, vert);
    glAttachShader(tr->shader_program, frag);
    glLinkProgram(tr->shader_program);

    GLint success;
    glGetProgramiv(tr->shader_program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(tr->shader_program, sizeof(log), NULL, log);
        fprintf(stderr, "Text Shader link error: %s\n", log);
        glDeleteShader(vert);
        glDeleteShader(frag);
        glDeleteProgram(tr->shader_program);
        return false;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);

    // Get uniform locations
    tr->u_projection = glGetUniformLocation(tr->shader_program, "u_projection");
    tr->u_texture = glGetUniformLocation(tr->shader_program, "u_texture");
    tr->u_color = glGetUniformLocation(tr->shader_program, "u_color");

    // Create VAO/VBO for dynamic text
    glGenVertexArrays(1, &tr->vao);
    glGenBuffers(1, &tr->vbo);

    glBindVertexArray(tr->vao);
    glBindBuffer(GL_ARRAY_BUFFER, tr->vbo);
    // Reserve space for text (will be updated per draw call)
    glBufferData(GL_ARRAY_BUFFER, 6 * 4 * sizeof(float) * 256, NULL, GL_DYNAMIC_DRAW);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

    // UV attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    glBindVertexArray(0);

    printf("Text Renderer initialized (font size: %.0f)\n", font_size);
    return true;
}

void text_renderer_destroy(TextRenderer* tr) {
    if (tr->texture) glDeleteTextures(1, &tr->texture);
    if (tr->vao) glDeleteVertexArrays(1, &tr->vao);
    if (tr->vbo) glDeleteBuffers(1, &tr->vbo);
    if (tr->shader_program) glDeleteProgram(tr->shader_program);
    memset(tr, 0, sizeof(TextRenderer));
}

void text_renderer_begin(TextRenderer* tr, int screen_width, int screen_height) {
    // Orthographic projection (origin top-left, Y down)
    Mat4 projection = mat4_ortho(0, (float)screen_width, (float)screen_height, 0, -1, 1);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    glUseProgram(tr->shader_program);
    glUniformMatrix4fv(tr->u_projection, 1, GL_FALSE, projection.m);
    glUniform1i(tr->u_texture, 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tr->texture);
    glBindVertexArray(tr->vao);
}

void text_renderer_end(TextRenderer* tr) {
    (void)tr;
    glBindVertexArray(0);
    glUseProgram(0);
    glEnable(GL_DEPTH_TEST);
}

void text_draw(TextRenderer* tr, const char* text, float x, float y, UIColor color) {
    if (!text || !*text) return;

    glUniform4f(tr->u_color, color.r, color.g, color.b, color.a);

    // Build vertex data
    int len = strlen(text);
    if (len > 256) len = 256;

    float vertices[6 * 4 * 256];  // 6 verts per char, 4 floats per vert
    int vertex_count = 0;

    float cursor_x = x;
    float cursor_y = y + tr->ascent;  // Baseline

    for (int i = 0; i < len; i++) {
        int c = (unsigned char)text[i];
        if (c < FONT_ATLAS_FIRST_CHAR || c >= FONT_ATLAS_FIRST_CHAR + FONT_ATLAS_NUM_CHARS) {
            c = '?';  // Unknown character
        }

        CharInfo* ci = &tr->chars[c - FONT_ATLAS_FIRST_CHAR];

        float x0 = cursor_x + ci->x0;
        float y0 = cursor_y + ci->y0;
        float x1 = cursor_x + ci->x1;
        float y1 = cursor_y + ci->y1;

        // Two triangles (6 vertices)
        float* v = &vertices[vertex_count * 4];

        // Triangle 1
        v[0] = x0; v[1] = y0; v[2] = ci->u0; v[3] = ci->v0;
        v[4] = x1; v[5] = y0; v[6] = ci->u1; v[7] = ci->v0;
        v[8] = x1; v[9] = y1; v[10] = ci->u1; v[11] = ci->v1;

        // Triangle 2
        v[12] = x0; v[13] = y0; v[14] = ci->u0; v[15] = ci->v0;
        v[16] = x1; v[17] = y1; v[18] = ci->u1; v[19] = ci->v1;
        v[20] = x0; v[21] = y1; v[22] = ci->u0; v[23] = ci->v1;

        vertex_count += 6;
        cursor_x += ci->advance;
    }

    // Upload and draw
    glBindBuffer(GL_ARRAY_BUFFER, tr->vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertex_count * 4 * sizeof(float), vertices);
    glDrawArrays(GL_TRIANGLES, 0, vertex_count);
}

void text_draw_centered(TextRenderer* tr, const char* text, UIRect rect, UIColor color) {
    float width = text_measure_width(tr, text);
    float height = tr->font_size;

    float x = rect.x + (rect.width - width) / 2;
    float y = rect.y + (rect.height - height) / 2;

    text_draw(tr, text, x, y, color);
}

float text_measure_width(TextRenderer* tr, const char* text) {
    if (!text || !*text) return 0;

    float width = 0;
    while (*text) {
        int c = (unsigned char)*text;
        if (c >= FONT_ATLAS_FIRST_CHAR && c < FONT_ATLAS_FIRST_CHAR + FONT_ATLAS_NUM_CHARS) {
            width += tr->chars[c - FONT_ATLAS_FIRST_CHAR].advance;
        }
        text++;
    }
    return width;
}

float text_get_height(TextRenderer* tr) {
    return tr->font_size;
}
