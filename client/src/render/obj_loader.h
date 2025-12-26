#ifndef OBJ_LOADER_H
#define OBJ_LOADER_H

#include <stdbool.h>
#include <GL/glew.h>
#include "../math/vec3.h"

// A loaded mesh with GPU buffers ready for rendering
typedef struct LoadedMesh {
    GLuint vao;
    GLuint vbo;
    int vertex_count;      // Number of vertices to draw
    Vec3 bounds_min;       // Bounding box min
    Vec3 bounds_max;       // Bounding box max
    bool valid;
} LoadedMesh;

// Load an OBJ file and create GPU buffers
// Returns true on success, false on failure
bool obj_load(LoadedMesh* mesh, const char* filepath);

// Free GPU resources
void obj_destroy(LoadedMesh* mesh);

// Get the center of the mesh (for positioning)
Vec3 obj_get_center(LoadedMesh* mesh);

// Get the size of the mesh (for scaling)
Vec3 obj_get_size(LoadedMesh* mesh);

#endif // OBJ_LOADER_H
