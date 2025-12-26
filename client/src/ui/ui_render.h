#ifndef UI_RENDER_H
#define UI_RENDER_H

#include <stdbool.h>
#include <GL/glew.h>
#include "../math/vec3.h"

/*
 * Simple 2D UI Renderer
 * =====================
 * Renders colored rectangles (panels) in screen space.
 * Uses orthographic projection for pixel-perfect 2D rendering.
 */

// Color with alpha
typedef struct {
    float r, g, b, a;
} UIColor;

// Predefined colors (from planning-ui-design.md)
#define UI_COLOR_SAFE       (UIColor){0.298f, 0.686f, 0.314f, 1.0f}  // #4CAF50
#define UI_COLOR_CAUTION    (UIColor){1.000f, 0.757f, 0.027f, 1.0f}  // #FFC107
#define UI_COLOR_DANGER     (UIColor){0.957f, 0.263f, 0.212f, 1.0f}  // #F44336
#define UI_COLOR_SELECTED   (UIColor){0.129f, 0.588f, 0.953f, 1.0f}  // #2196F3
#define UI_COLOR_DISABLED   (UIColor){0.620f, 0.620f, 0.620f, 1.0f}  // #9E9E9E
#define UI_COLOR_BG_DARK    (UIColor){0.102f, 0.102f, 0.180f, 0.9f}  // #1A1A2E
#define UI_COLOR_PANEL      (UIColor){0.086f, 0.129f, 0.243f, 0.95f} // #16213E
#define UI_COLOR_TEXT       (UIColor){1.000f, 1.000f, 1.000f, 1.0f}  // #FFFFFF
#define UI_COLOR_ACCENT     (UIColor){1.000f, 0.420f, 0.208f, 1.0f}  // #FF6B35
#define UI_COLOR_BLACK      (UIColor){0.0f, 0.0f, 0.0f, 1.0f}
#define UI_COLOR_WHITE      (UIColor){1.0f, 1.0f, 1.0f, 1.0f}

// Rectangle (in screen pixels, origin top-left)
typedef struct {
    float x, y;         // Top-left corner
    float width, height;
} UIRect;

// UI Renderer state
typedef struct {
    GLuint vao;
    GLuint vbo;
    GLuint shader_program;

    // Uniform locations
    GLint u_projection;
    GLint u_rect;
    GLint u_color;
    GLint u_border_color;
    GLint u_border_width;
    GLint u_corner_radius;

    // Screen dimensions (updated each frame)
    int screen_width;
    int screen_height;
} UIRenderer;

// Initialize the UI renderer
bool ui_renderer_init(UIRenderer* ui);

// Cleanup
void ui_renderer_destroy(UIRenderer* ui);

// Call at start of UI rendering (sets up orthographic projection)
void ui_renderer_begin(UIRenderer* ui, int screen_width, int screen_height);

// Call at end of UI rendering
void ui_renderer_end(UIRenderer* ui);

// Draw a filled rectangle
void ui_draw_rect(UIRenderer* ui, UIRect rect, UIColor color);

// Draw a rectangle with border
void ui_draw_rect_bordered(UIRenderer* ui, UIRect rect, UIColor fill_color,
                           UIColor border_color, float border_width);

// Draw a rectangle with rounded corners
void ui_draw_rect_rounded(UIRenderer* ui, UIRect rect, UIColor color, float corner_radius);

// Draw a rectangle with rounded corners and border
void ui_draw_panel(UIRenderer* ui, UIRect rect, UIColor fill_color,
                   UIColor border_color, float border_width, float corner_radius);

// Helper: Create UIRect
static inline UIRect ui_rect(float x, float y, float w, float h) {
    return (UIRect){x, y, w, h};
}

// Helper: Create UIColor
static inline UIColor ui_color(float r, float g, float b, float a) {
    return (UIColor){r, g, b, a};
}

// Helper: Create UIColor from hex (0xRRGGBB)
static inline UIColor ui_color_hex(unsigned int hex, float alpha) {
    return (UIColor){
        ((hex >> 16) & 0xFF) / 255.0f,
        ((hex >> 8) & 0xFF) / 255.0f,
        (hex & 0xFF) / 255.0f,
        alpha
    };
}

// Helper: Check if point is inside rect
static inline bool ui_rect_contains(UIRect rect, float x, float y) {
    return x >= rect.x && x < rect.x + rect.width &&
           y >= rect.y && y < rect.y + rect.height;
}

#endif // UI_RENDER_H
