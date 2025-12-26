#ifndef UI_TEXT_H
#define UI_TEXT_H

#include <stdbool.h>
#include <GL/glew.h>
#include "ui_render.h"

/*
 * Text Renderer using stb_truetype
 * =================================
 * Renders text in screen space using a TTF font.
 */

// Maximum characters in the font atlas
#define FONT_ATLAS_FIRST_CHAR 32   // Space
#define FONT_ATLAS_NUM_CHARS  95   // ASCII 32-126

typedef struct {
    float x0, y0, x1, y1;  // Bounding box in pixels (relative to baseline)
    float u0, v0, u1, v1;  // Texture coordinates
    float advance;          // Horizontal advance
} CharInfo;

typedef struct {
    GLuint texture;           // Font atlas texture
    int atlas_width;
    int atlas_height;
    float font_size;          // Size font was rendered at
    float ascent;             // Distance from baseline to top
    float descent;            // Distance from baseline to bottom (negative)
    float line_height;        // Recommended line spacing
    CharInfo chars[FONT_ATLAS_NUM_CHARS];

    // Shader for text rendering
    GLuint shader_program;
    GLuint vao;
    GLuint vbo;

    // Uniform locations
    GLint u_projection;
    GLint u_texture;
    GLint u_color;
} TextRenderer;

// Initialize text renderer with a TTF font file
bool text_renderer_init(TextRenderer* tr, const char* font_path, float font_size);

// Cleanup
void text_renderer_destroy(TextRenderer* tr);

// Begin text rendering (call after ui_renderer_begin)
void text_renderer_begin(TextRenderer* tr, int screen_width, int screen_height);

// End text rendering
void text_renderer_end(TextRenderer* tr);

// Draw text at position (screen pixels, origin top-left)
void text_draw(TextRenderer* tr, const char* text, float x, float y, UIColor color);

// Draw text centered horizontally within a rect
void text_draw_centered(TextRenderer* tr, const char* text, UIRect rect, UIColor color);

// Measure text width in pixels
float text_measure_width(TextRenderer* tr, const char* text);

// Get text height (line height)
float text_get_height(TextRenderer* tr);

#endif // UI_TEXT_H
