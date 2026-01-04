#include "texture.h"
#include <stdio.h>

#define STB_IMAGE_IMPLEMENTATION
#include "../vendor/stb_image.h"

GLuint texture_load(const char* filepath) {
    int width, height, channels;

    // Flip vertically so (0,0) is bottom-left (OpenGL convention)
    stbi_set_flip_vertically_on_load(1);

    unsigned char* data = stbi_load(filepath, &width, &height, &channels, 0);
    if (!data) {
        fprintf(stderr, "Failed to load texture: %s\n", filepath);
        return 0;
    }

    GLenum format = GL_RGB;
    if (channels == 4) {
        format = GL_RGBA;
    } else if (channels == 1) {
        format = GL_RED;
    }

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);  // Pixel art style
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // Upload texture data
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);

    stbi_image_free(data);

    printf("Loaded texture: %s (%dx%d, %d channels)\n", filepath, width, height, channels);
    return texture;
}

void texture_destroy(GLuint texture) {
    if (texture) {
        glDeleteTextures(1, &texture);
    }
}
