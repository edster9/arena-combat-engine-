#include "mesh.h"
#include <stdio.h>
#include <math.h>

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

// Vertex shader for textured meshes
static const char* textured_vert_src =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "layout (location = 1) in vec3 aNormal;\n"
    "layout (location = 2) in vec2 aTexCoord;\n"
    "out vec3 fragNormal;\n"
    "out vec3 fragPos;\n"
    "out vec2 texCoord;\n"
    "uniform mat4 model;\n"
    "uniform mat4 view;\n"
    "uniform mat4 projection;\n"
    "void main() {\n"
    "    fragPos = vec3(model * vec4(aPos, 1.0));\n"
    "    fragNormal = mat3(transpose(inverse(model))) * aNormal;\n"
    "    texCoord = aTexCoord;\n"
    "    gl_Position = projection * view * vec4(fragPos, 1.0);\n"
    "}\n";

// Fragment shader for textured meshes with lighting
static const char* textured_frag_src =
    "#version 330 core\n"
    "in vec3 fragNormal;\n"
    "in vec3 fragPos;\n"
    "in vec2 texCoord;\n"
    "out vec4 FragColor;\n"
    "uniform vec3 lightDir;\n"
    "uniform sampler2D textureSampler;\n"
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
    "    // Sample texture\n"
    "    vec3 texColor = texture(textureSampler, texCoord).rgb;\n"
    "    \n"
    "    // Combine lighting with texture\n"
    "    vec3 result = (ambient + diff * 0.7) * texColor;\n"
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

    // Cache uniform locations for color shader
    r->u_model = glGetUniformLocation(r->shader.program, "model");
    r->u_view = glGetUniformLocation(r->shader.program, "view");
    r->u_projection = glGetUniformLocation(r->shader.program, "projection");
    r->u_lightDir = glGetUniformLocation(r->shader.program, "lightDir");
    r->u_objectColor = glGetUniformLocation(r->shader.program, "objectColor");

    // Create textured shader
    if (!shader_create(&r->textured_shader, textured_vert_src, textured_frag_src)) {
        fprintf(stderr, "Failed to create textured shader\n");
        shader_destroy(&r->shader);
        return false;
    }

    // Cache uniform locations for textured shader
    r->ut_model = glGetUniformLocation(r->textured_shader.program, "model");
    r->ut_view = glGetUniformLocation(r->textured_shader.program, "view");
    r->ut_projection = glGetUniformLocation(r->textured_shader.program, "projection");
    r->ut_lightDir = glGetUniformLocation(r->textured_shader.program, "lightDir");
    r->ut_texture = glGetUniformLocation(r->textured_shader.program, "textureSampler");

    if (!create_box_mesh(&r->unit_box)) {
        shader_destroy(&r->shader);
        shader_destroy(&r->textured_shader);
        return false;
    }

    r->valid = true;
    return true;
}

void box_renderer_destroy(BoxRenderer* r) {
    if (r->valid) {
        shader_destroy(&r->shader);
        shader_destroy(&r->textured_shader);
        glDeleteVertexArrays(1, &r->unit_box.vao);
        glDeleteBuffers(1, &r->unit_box.vbo);
        r->valid = false;
    }
}

void box_renderer_begin(BoxRenderer* r, Mat4* view, Mat4* projection, Vec3 light_dir) {
    // Cache frame data for textured shader
    r->cached_view = *view;
    r->cached_projection = *projection;
    r->cached_light_dir = light_dir;

    shader_use(&r->shader);
    // Use cached uniform locations (faster than glGetUniformLocation every frame)
    glUniformMatrix4fv(r->u_view, 1, GL_FALSE, view->m);
    glUniformMatrix4fv(r->u_projection, 1, GL_FALSE, projection->m);
    glUniform3f(r->u_lightDir, light_dir.x, light_dir.y, light_dir.z);
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

    // Use cached uniform locations
    glUniformMatrix4fv(r->u_model, 1, GL_FALSE, model.m);
    glUniform3f(r->u_objectColor, color.x, color.y, color.z);

    glDrawArrays(GL_TRIANGLES, 0, r->unit_box.vertex_count);
}

void box_renderer_draw_rotated(BoxRenderer* r, Vec3 pos, Vec3 size, float rotation_y, Vec3 color) {
    // Build model matrix: translate * rotate * scale
    float c = cosf(rotation_y);
    float s = sinf(rotation_y);

    Mat4 model = mat4_identity();

    // Combined transform (scale, rotate Y, translate) in column-major order
    // This is: T * Ry * S where operations apply right-to-left
    model.m[0] = c * size.x;
    model.m[1] = 0;
    model.m[2] = -s * size.x;
    model.m[3] = 0;

    model.m[4] = 0;
    model.m[5] = size.y;
    model.m[6] = 0;
    model.m[7] = 0;

    model.m[8] = s * size.z;
    model.m[9] = 0;
    model.m[10] = c * size.z;
    model.m[11] = 0;

    model.m[12] = pos.x;
    model.m[13] = pos.y;
    model.m[14] = pos.z;
    model.m[15] = 1;

    // Use cached uniform locations
    glUniformMatrix4fv(r->u_model, 1, GL_FALSE, model.m);
    glUniform3f(r->u_objectColor, color.x, color.y, color.z);

    glDrawArrays(GL_TRIANGLES, 0, r->unit_box.vertex_count);
}

void box_renderer_draw_rotated_matrix(BoxRenderer* r, Vec3 pos, Vec3 size, const float* rot_matrix, Vec3 color) {
    // Build model matrix: translate * rotate * scale
    // rot_matrix is 3x3 row-major, OpenGL needs 4x4 column-major
    // Combined: model = T * R * S
    Mat4 model = mat4_identity();

    // Column 0: R * scale_x (first column of rotation * size.x)
    model.m[0] = rot_matrix[0] * size.x;
    model.m[1] = rot_matrix[3] * size.x;
    model.m[2] = rot_matrix[6] * size.x;
    model.m[3] = 0;

    // Column 1: R * scale_y (second column of rotation * size.y)
    model.m[4] = rot_matrix[1] * size.y;
    model.m[5] = rot_matrix[4] * size.y;
    model.m[6] = rot_matrix[7] * size.y;
    model.m[7] = 0;

    // Column 2: R * scale_z (third column of rotation * size.z)
    model.m[8] = rot_matrix[2] * size.z;
    model.m[9] = rot_matrix[5] * size.z;
    model.m[10] = rot_matrix[8] * size.z;
    model.m[11] = 0;

    // Column 3: translation
    model.m[12] = pos.x;
    model.m[13] = pos.y;
    model.m[14] = pos.z;
    model.m[15] = 1;

    // Use cached uniform locations
    glUniformMatrix4fv(r->u_model, 1, GL_FALSE, model.m);
    glUniform3f(r->u_objectColor, color.x, color.y, color.z);

    glDrawArrays(GL_TRIANGLES, 0, r->unit_box.vertex_count);
}

void box_renderer_draw_mesh(BoxRenderer* r, GLuint vao, int vertex_count,
                            Vec3 pos, float scale, float rotation_y, Vec3 color) {
    // Build model matrix: translate * rotate * scale
    // For Y-axis rotation: [cos, 0, sin, 0], [0, 1, 0, 0], [-sin, 0, cos, 0], [0, 0, 0, 1]
    float c = cosf(rotation_y);
    float s = sinf(rotation_y);

    Mat4 model = mat4_identity();

    // Combined transform (scale, rotate Y, translate) in column-major order
    // This is: T * Ry * S where operations apply right-to-left
    model.m[0] = c * scale;
    model.m[1] = 0;
    model.m[2] = -s * scale;
    model.m[3] = 0;

    model.m[4] = 0;
    model.m[5] = scale;
    model.m[6] = 0;
    model.m[7] = 0;

    model.m[8] = s * scale;
    model.m[9] = 0;
    model.m[10] = c * scale;
    model.m[11] = 0;

    model.m[12] = pos.x;
    model.m[13] = pos.y;
    model.m[14] = pos.z;
    model.m[15] = 1;

    // Use cached uniform locations
    glUniformMatrix4fv(r->u_model, 1, GL_FALSE, model.m);
    glUniform3f(r->u_objectColor, color.x, color.y, color.z);

    // Bind the loaded mesh's VAO and draw
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, vertex_count);
    // Rebind the box VAO for subsequent box_renderer_draw calls
    glBindVertexArray(r->unit_box.vao);
}

void box_renderer_draw_mesh_matrix(BoxRenderer* r, GLuint vao, int vertex_count,
                                   Vec3 pos, Vec3 scale, const float* rot_matrix,
                                   Vec3 pre_translate, Vec3 color) {
    // Build model matrix: T * R * S * T_pre
    // Where T_pre centers the mesh, S scales it, R rotates it, and T positions it
    // rot_matrix is column-major 3x3: [0-2]=X axis, [3-5]=Y axis, [6-8]=Z axis

    // Scale each rotation column
    float sx = scale.x, sy = scale.y, sz = scale.z;

    // First apply pre-translation and scale
    float px = pre_translate.x * sx;
    float py = pre_translate.y * sy;
    float pz = pre_translate.z * sz;

    // Then rotate the pre-translation
    float rpx = rot_matrix[0] * px + rot_matrix[3] * py + rot_matrix[6] * pz;
    float rpy = rot_matrix[1] * px + rot_matrix[4] * py + rot_matrix[7] * pz;
    float rpz = rot_matrix[2] * px + rot_matrix[5] * py + rot_matrix[8] * pz;

    Mat4 model = mat4_identity();

    // Rotation * Scale (column-major layout)
    model.m[0] = rot_matrix[0] * sx;
    model.m[1] = rot_matrix[1] * sx;
    model.m[2] = rot_matrix[2] * sx;
    model.m[3] = 0;

    model.m[4] = rot_matrix[3] * sy;
    model.m[5] = rot_matrix[4] * sy;
    model.m[6] = rot_matrix[5] * sy;
    model.m[7] = 0;

    model.m[8] = rot_matrix[6] * sz;
    model.m[9] = rot_matrix[7] * sz;
    model.m[10] = rot_matrix[8] * sz;
    model.m[11] = 0;

    // Translation (includes rotated pre-translation)
    model.m[12] = pos.x + rpx;
    model.m[13] = pos.y + rpy;
    model.m[14] = pos.z + rpz;
    model.m[15] = 1;

    glUniformMatrix4fv(r->u_model, 1, GL_FALSE, model.m);
    glUniform3f(r->u_objectColor, color.x, color.y, color.z);

    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, vertex_count);
    glBindVertexArray(r->unit_box.vao);
}

void box_renderer_draw_mesh_rotated(BoxRenderer* r, GLuint vao, int vertex_count,
                                    Vec3 pos, float scale, const float* rot_matrix,
                                    Vec3 pre_translate, Vec3 color) {
    // Build model matrix: T * R * S * T_pre
    // rot_matrix is 3x3 ROW-MAJOR (like chassis), convert to OpenGL column-major

    // Apply pre-translation and scale
    float px = pre_translate.x * scale;
    float py = pre_translate.y * scale;
    float pz = pre_translate.z * scale;

    // Rotate the pre-translation (row-major: multiply by columns)
    float rpx = rot_matrix[0] * px + rot_matrix[1] * py + rot_matrix[2] * pz;
    float rpy = rot_matrix[3] * px + rot_matrix[4] * py + rot_matrix[5] * pz;
    float rpz = rot_matrix[6] * px + rot_matrix[7] * py + rot_matrix[8] * pz;

    Mat4 model = mat4_identity();

    // Convert row-major to column-major: transpose while scaling
    // Column 0 = Row 0 * scale
    model.m[0] = rot_matrix[0] * scale;
    model.m[1] = rot_matrix[3] * scale;
    model.m[2] = rot_matrix[6] * scale;
    model.m[3] = 0;

    // Column 1 = Row 1 * scale
    model.m[4] = rot_matrix[1] * scale;
    model.m[5] = rot_matrix[4] * scale;
    model.m[6] = rot_matrix[7] * scale;
    model.m[7] = 0;

    // Column 2 = Row 2 * scale
    model.m[8] = rot_matrix[2] * scale;
    model.m[9] = rot_matrix[5] * scale;
    model.m[10] = rot_matrix[8] * scale;
    model.m[11] = 0;

    // Translation (includes rotated pre-translation)
    model.m[12] = pos.x + rpx;
    model.m[13] = pos.y + rpy;
    model.m[14] = pos.z + rpz;
    model.m[15] = 1;

    glUniformMatrix4fv(r->u_model, 1, GL_FALSE, model.m);
    glUniform3f(r->u_objectColor, color.x, color.y, color.z);

    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, vertex_count);
    glBindVertexArray(r->unit_box.vao);
}

void box_renderer_draw_mesh_textured(BoxRenderer* r, GLuint vao, int vertex_count,
                                     Vec3 pos, float scale, const float* rot_matrix,
                                     Vec3 pre_translate, GLuint texture) {
    // Switch to textured shader
    shader_use(&r->textured_shader);

    // Set view/projection/light from cached frame data
    glUniformMatrix4fv(r->ut_view, 1, GL_FALSE, r->cached_view.m);
    glUniformMatrix4fv(r->ut_projection, 1, GL_FALSE, r->cached_projection.m);
    glUniform3f(r->ut_lightDir, r->cached_light_dir.x, r->cached_light_dir.y, r->cached_light_dir.z);

    // Build model matrix: T * R * S * T_pre
    // rot_matrix is 3x3 ROW-MAJOR (like chassis), convert to OpenGL column-major

    // Apply pre-translation and scale
    float px = pre_translate.x * scale;
    float py = pre_translate.y * scale;
    float pz = pre_translate.z * scale;

    // Rotate the pre-translation (row-major: multiply by columns)
    float rpx = rot_matrix[0] * px + rot_matrix[1] * py + rot_matrix[2] * pz;
    float rpy = rot_matrix[3] * px + rot_matrix[4] * py + rot_matrix[5] * pz;
    float rpz = rot_matrix[6] * px + rot_matrix[7] * py + rot_matrix[8] * pz;

    Mat4 model = mat4_identity();

    // Convert row-major to column-major: transpose while scaling
    model.m[0] = rot_matrix[0] * scale;
    model.m[1] = rot_matrix[3] * scale;
    model.m[2] = rot_matrix[6] * scale;
    model.m[3] = 0;

    model.m[4] = rot_matrix[1] * scale;
    model.m[5] = rot_matrix[4] * scale;
    model.m[6] = rot_matrix[7] * scale;
    model.m[7] = 0;

    model.m[8] = rot_matrix[2] * scale;
    model.m[9] = rot_matrix[5] * scale;
    model.m[10] = rot_matrix[8] * scale;
    model.m[11] = 0;

    // Translation (includes rotated pre-translation)
    model.m[12] = pos.x + rpx;
    model.m[13] = pos.y + rpy;
    model.m[14] = pos.z + rpz;
    model.m[15] = 1;

    glUniformMatrix4fv(r->ut_model, 1, GL_FALSE, model.m);

    // Bind texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform1i(r->ut_texture, 0);

    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, vertex_count);

    // Switch back to color shader for subsequent draws
    shader_use(&r->shader);
    glBindVertexArray(r->unit_box.vao);
}

void box_renderer_end(BoxRenderer* r) {
    (void)r;
    glBindVertexArray(0);
    glUseProgram(0);
}
