#include "mesh.h"
#include <stdio.h>

// Vertex shader for lit boxes
static const char* box_vert_src =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "layout (location = 1) in vec3 aNormal;\n"
    "out vec3 fragNormal;\n"
    "out vec3 fragPos;\n"
    "uniform mat4 model;\n"
    "uniform mat4 view;\n"
    "uniform mat4 projection;\n"
    "void main() {\n"
    "    fragPos = vec3(model * vec4(aPos, 1.0));\n"
    "    fragNormal = mat3(transpose(inverse(model))) * aNormal;\n"
    "    gl_Position = projection * view * vec4(fragPos, 1.0);\n"
    "}\n";

// Fragment shader with simple directional lighting
static const char* box_frag_src =
    "#version 330 core\n"
    "in vec3 fragNormal;\n"
    "in vec3 fragPos;\n"
    "out vec4 FragColor;\n"
    "uniform vec3 lightDir;\n"
    "uniform vec3 objectColor;\n"
    "void main() {\n"
    "    vec3 norm = normalize(fragNormal);\n"
    "    vec3 light = normalize(-lightDir);\n"
    "    \n"
    "    // Ambient\n"
    "    float ambient = 0.3;\n"
    "    \n"
    "    // Diffuse\n"
    "    float diff = max(dot(norm, light), 0.0);\n"
    "    \n"
    "    // Combine\n"
    "    vec3 result = (ambient + diff * 0.7) * objectColor;\n"
    "    FragColor = vec4(result, 1.0);\n"
    "}\n";

// Unit box vertices with normals (position, normal)
static const float box_vertices[] = {
    // Front face (z = 0.5)
    -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
     0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
     0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
     0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
    -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
    -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

    // Back face (z = -0.5)
    -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
    -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
     0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
     0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
     0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

    // Left face (x = -0.5)
    -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
    -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
    -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
    -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
    -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
    -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

    // Right face (x = 0.5)
     0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
     0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
     0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
     0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
     0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
     0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

    // Top face (y = 0.5)
    -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
    -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
     0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
     0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
     0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
    -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,

    // Bottom face (y = -0.5)
    -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
     0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
     0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
     0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
};

static bool create_box_mesh(BoxMesh* mesh) {
    mesh->vertex_count = 36;

    glGenVertexArrays(1, &mesh->vao);
    glGenBuffers(1, &mesh->vbo);

    glBindVertexArray(mesh->vao);

    glBindBuffer(GL_ARRAY_BUFFER, mesh->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(box_vertices), box_vertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    return true;
}

bool box_renderer_init(BoxRenderer* r) {
    r->valid = false;

    if (!shader_create(&r->shader, box_vert_src, box_frag_src)) {
        fprintf(stderr, "Failed to create box shader\n");
        return false;
    }

    if (!create_box_mesh(&r->unit_box)) {
        shader_destroy(&r->shader);
        return false;
    }

    r->valid = true;
    return true;
}

void box_renderer_destroy(BoxRenderer* r) {
    if (r->valid) {
        shader_destroy(&r->shader);
        glDeleteVertexArrays(1, &r->unit_box.vao);
        glDeleteBuffers(1, &r->unit_box.vbo);
        r->valid = false;
    }
}

void box_renderer_begin(BoxRenderer* r, Mat4* view, Mat4* projection, Vec3 light_dir) {
    shader_use(&r->shader);
    shader_set_mat4(&r->shader, "view", view);
    shader_set_mat4(&r->shader, "projection", projection);
    shader_set_vec3(&r->shader, "lightDir", light_dir);
    glBindVertexArray(r->unit_box.vao);
}

void box_renderer_draw(BoxRenderer* r, Vec3 pos, Vec3 size, Vec3 color) {
    // Build model matrix: translate then scale
    Mat4 model = mat4_identity();

    // Scale
    model.m[0] = size.x;
    model.m[5] = size.y;
    model.m[10] = size.z;

    // Translate (applied after scale in column-major)
    model.m[12] = pos.x;
    model.m[13] = pos.y;
    model.m[14] = pos.z;

    shader_set_mat4(&r->shader, "model", &model);
    shader_set_vec3(&r->shader, "objectColor", color);

    glDrawArrays(GL_TRIANGLES, 0, r->unit_box.vertex_count);
}

void box_renderer_end(BoxRenderer* r) {
    (void)r;
    glBindVertexArray(0);
    glUseProgram(0);
}
