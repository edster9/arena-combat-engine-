#ifndef TEXTURE_H
#define TEXTURE_H

#include <GL/glew.h>
#include <stdbool.h>

// Load a texture from a PNG file
// Returns the OpenGL texture ID, or 0 on failure
GLuint texture_load(const char* filepath);

// Free a loaded texture
void texture_destroy(GLuint texture);

#endif // TEXTURE_H
