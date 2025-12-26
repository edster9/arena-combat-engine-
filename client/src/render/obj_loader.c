#include "obj_loader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

// Dynamic arrays for parsing
typedef struct {
    float* data;
    int count;
    int capacity;
} FloatArray;

typedef struct {
    int* data;
    int count;
    int capacity;
} IntArray;

static void float_array_init(FloatArray* arr) {
    arr->data = NULL;
    arr->count = 0;
    arr->capacity = 0;
}

static void float_array_push(FloatArray* arr, float value) {
    if (arr->count >= arr->capacity) {
        arr->capacity = arr->capacity == 0 ? 64 : arr->capacity * 2;
        arr->data = realloc(arr->data, arr->capacity * sizeof(float));
    }
    arr->data[arr->count++] = value;
}

static void float_array_free(FloatArray* arr) {
    free(arr->data);
    arr->data = NULL;
    arr->count = 0;
    arr->capacity = 0;
}

static void int_array_init(IntArray* arr) {
    arr->data = NULL;
    arr->count = 0;
    arr->capacity = 0;
}

static void int_array_push(IntArray* arr, int value) {
    if (arr->count >= arr->capacity) {
        arr->capacity = arr->capacity == 0 ? 64 : arr->capacity * 2;
        arr->data = realloc(arr->data, arr->capacity * sizeof(int));
    }
    arr->data[arr->count++] = value;
}

static void int_array_free(IntArray* arr) {
    free(arr->data);
    arr->data = NULL;
    arr->count = 0;
    arr->capacity = 0;
}

// Parse a face index like "1/2/3" or "1//3" or "1"
static void parse_face_vertex(const char* str, int* v, int* vt, int* vn) {
    *v = 0;
    *vt = 0;
    *vn = 0;

    // Parse vertex index
    *v = atoi(str);

    // Find first slash
    const char* slash1 = strchr(str, '/');
    if (!slash1) return;

    // Parse texture index (may be empty)
    if (slash1[1] != '/') {
        *vt = atoi(slash1 + 1);
    }

    // Find second slash
    const char* slash2 = strchr(slash1 + 1, '/');
    if (!slash2) return;

    // Parse normal index
    *vn = atoi(slash2 + 1);
}

bool obj_load(LoadedMesh* mesh, const char* filepath) {
    mesh->valid = false;
    mesh->vao = 0;
    mesh->vbo = 0;
    mesh->vertex_count = 0;
    mesh->bounds_min = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    mesh->bounds_max = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    FILE* file = fopen(filepath, "r");
    if (!file) {
        fprintf(stderr, "Failed to open OBJ file: %s\n", filepath);
        return false;
    }

    // Temporary storage for parsed data
    FloatArray positions, normals, texcoords;
    IntArray face_v, face_vt, face_vn;

    float_array_init(&positions);
    float_array_init(&normals);
    float_array_init(&texcoords);
    int_array_init(&face_v);
    int_array_init(&face_vt);
    int_array_init(&face_vn);

    char line[512];
    while (fgets(line, sizeof(line), file)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }

        // Vertex position (may have vertex colors: v x y z r g b)
        if (line[0] == 'v' && line[1] == ' ') {
            float x, y, z;
            sscanf(line + 2, "%f %f %f", &x, &y, &z);
            float_array_push(&positions, x);
            float_array_push(&positions, y);
            float_array_push(&positions, z);

            // Update bounds
            if (x < mesh->bounds_min.x) mesh->bounds_min.x = x;
            if (y < mesh->bounds_min.y) mesh->bounds_min.y = y;
            if (z < mesh->bounds_min.z) mesh->bounds_min.z = z;
            if (x > mesh->bounds_max.x) mesh->bounds_max.x = x;
            if (y > mesh->bounds_max.y) mesh->bounds_max.y = y;
            if (z > mesh->bounds_max.z) mesh->bounds_max.z = z;
        }
        // Vertex normal
        else if (line[0] == 'v' && line[1] == 'n' && line[2] == ' ') {
            float x, y, z;
            sscanf(line + 3, "%f %f %f", &x, &y, &z);
            float_array_push(&normals, x);
            float_array_push(&normals, y);
            float_array_push(&normals, z);
        }
        // Texture coordinate
        else if (line[0] == 'v' && line[1] == 't' && line[2] == ' ') {
            float u, v;
            sscanf(line + 3, "%f %f", &u, &v);
            float_array_push(&texcoords, u);
            float_array_push(&texcoords, v);
        }
        // Face (triangles only for now)
        else if (line[0] == 'f' && line[1] == ' ') {
            char v1[64], v2[64], v3[64], v4[64];
            int count = sscanf(line + 2, "%s %s %s %s", v1, v2, v3, v4);

            if (count >= 3) {
                int vi1, vt1, vn1;
                int vi2, vt2, vn2;
                int vi3, vt3, vn3;

                parse_face_vertex(v1, &vi1, &vt1, &vn1);
                parse_face_vertex(v2, &vi2, &vt2, &vn2);
                parse_face_vertex(v3, &vi3, &vt3, &vn3);

                // First triangle
                int_array_push(&face_v, vi1);
                int_array_push(&face_vt, vt1);
                int_array_push(&face_vn, vn1);
                int_array_push(&face_v, vi2);
                int_array_push(&face_vt, vt2);
                int_array_push(&face_vn, vn2);
                int_array_push(&face_v, vi3);
                int_array_push(&face_vt, vt3);
                int_array_push(&face_vn, vn3);

                // If quad, add second triangle
                if (count >= 4) {
                    int vi4, vt4, vn4;
                    parse_face_vertex(v4, &vi4, &vt4, &vn4);

                    int_array_push(&face_v, vi1);
                    int_array_push(&face_vt, vt1);
                    int_array_push(&face_vn, vn1);
                    int_array_push(&face_v, vi3);
                    int_array_push(&face_vt, vt3);
                    int_array_push(&face_vn, vn3);
                    int_array_push(&face_v, vi4);
                    int_array_push(&face_vt, vt4);
                    int_array_push(&face_vn, vn4);
                }
            }
        }
    }
    fclose(file);

    // Build interleaved vertex buffer (position + normal)
    int num_face_verts = face_v.count;
    if (num_face_verts == 0) {
        fprintf(stderr, "OBJ file has no faces: %s\n", filepath);
        float_array_free(&positions);
        float_array_free(&normals);
        float_array_free(&texcoords);
        int_array_free(&face_v);
        int_array_free(&face_vt);
        int_array_free(&face_vn);
        return false;
    }

    // 6 floats per vertex: x, y, z, nx, ny, nz
    float* vertex_data = malloc(num_face_verts * 6 * sizeof(float));
    if (!vertex_data) {
        fprintf(stderr, "Failed to allocate vertex buffer\n");
        float_array_free(&positions);
        float_array_free(&normals);
        float_array_free(&texcoords);
        int_array_free(&face_v);
        int_array_free(&face_vt);
        int_array_free(&face_vn);
        return false;
    }

    // Default normal if none specified
    float default_normal[3] = {0.0f, 1.0f, 0.0f};

    for (int i = 0; i < num_face_verts; i++) {
        int vi = face_v.data[i] - 1;  // OBJ indices are 1-based
        int vni = face_vn.data[i] - 1;

        // Position
        if (vi >= 0 && vi * 3 + 2 < positions.count) {
            vertex_data[i * 6 + 0] = positions.data[vi * 3 + 0];
            vertex_data[i * 6 + 1] = positions.data[vi * 3 + 1];
            vertex_data[i * 6 + 2] = positions.data[vi * 3 + 2];
        } else {
            vertex_data[i * 6 + 0] = 0.0f;
            vertex_data[i * 6 + 1] = 0.0f;
            vertex_data[i * 6 + 2] = 0.0f;
        }

        // Normal
        if (vni >= 0 && vni * 3 + 2 < normals.count) {
            vertex_data[i * 6 + 3] = normals.data[vni * 3 + 0];
            vertex_data[i * 6 + 4] = normals.data[vni * 3 + 1];
            vertex_data[i * 6 + 5] = normals.data[vni * 3 + 2];
        } else {
            vertex_data[i * 6 + 3] = default_normal[0];
            vertex_data[i * 6 + 4] = default_normal[1];
            vertex_data[i * 6 + 5] = default_normal[2];
        }
    }

    // Create VAO and VBO
    glGenVertexArrays(1, &mesh->vao);
    glGenBuffers(1, &mesh->vbo);

    glBindVertexArray(mesh->vao);

    glBindBuffer(GL_ARRAY_BUFFER, mesh->vbo);
    glBufferData(GL_ARRAY_BUFFER, num_face_verts * 6 * sizeof(float), vertex_data, GL_STATIC_DRAW);

    // Position attribute (location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attribute (location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    mesh->vertex_count = num_face_verts;
    mesh->valid = true;

    // Cleanup
    free(vertex_data);
    float_array_free(&positions);
    float_array_free(&normals);
    float_array_free(&texcoords);
    int_array_free(&face_v);
    int_array_free(&face_vt);
    int_array_free(&face_vn);

    printf("Loaded OBJ: %s (%d vertices)\n", filepath, mesh->vertex_count);
    return true;
}

void obj_destroy(LoadedMesh* mesh) {
    if (mesh->valid) {
        glDeleteVertexArrays(1, &mesh->vao);
        glDeleteBuffers(1, &mesh->vbo);
        mesh->vao = 0;
        mesh->vbo = 0;
        mesh->vertex_count = 0;
        mesh->valid = false;
    }
}

Vec3 obj_get_center(LoadedMesh* mesh) {
    return vec3(
        (mesh->bounds_min.x + mesh->bounds_max.x) * 0.5f,
        (mesh->bounds_min.y + mesh->bounds_max.y) * 0.5f,
        (mesh->bounds_min.z + mesh->bounds_max.z) * 0.5f
    );
}

Vec3 obj_get_size(LoadedMesh* mesh) {
    return vec3(
        mesh->bounds_max.x - mesh->bounds_min.x,
        mesh->bounds_max.y - mesh->bounds_min.y,
        mesh->bounds_max.z - mesh->bounds_min.z
    );
}
