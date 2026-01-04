#ifndef OBJ_LOADER_H
#define OBJ_LOADER_H

#include <stdbool.h>
#include <GL/glew.h>
#include "../math/vec3.h"

// A loaded mesh with GPU buffers ready for rendering
typedef struct LoadedMesh {
    GLuint vao;
    GLuint vbo;            // Position + normal data
    GLuint uv_vbo;         // UV coordinates (separate buffer)
    int vertex_count;      // Number of vertices to draw
    Vec3 bounds_min;       // Bounding box min
    Vec3 bounds_max;       // Bounding box max
    bool has_uvs;          // True if mesh has UV coordinates
    bool valid;
} LoadedMesh;

// Load an OBJ file and create GPU buffers
// Returns true on success, false on failure
bool obj_load(LoadedMesh* mesh, const char* filepath);

// Load specific groups from an OBJ file
// groups: array of group names to include (e.g., {"body", "spoiler"})
// num_groups: number of groups in the array
// If groups is NULL or num_groups is 0, loads all groups (same as obj_load)
bool obj_load_groups(LoadedMesh* mesh, const char* filepath, const char** groups, int num_groups);

// Free GPU resources
void obj_destroy(LoadedMesh* mesh);

// Get the center of the mesh (for positioning)
Vec3 obj_get_center(LoadedMesh* mesh);

// Get the size of the mesh (for scaling)
Vec3 obj_get_size(LoadedMesh* mesh);

#endif // OBJ_LOADER_H
