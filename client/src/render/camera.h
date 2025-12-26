#ifndef CAMERA_H
#define CAMERA_H

#include "../math/vec3.h"
#include "../math/mat4.h"
#include "../platform/platform.h"

typedef struct FlyCamera {
    Vec3 position;
    float yaw;      // Horizontal rotation (radians)
    float pitch;    // Vertical rotation (radians)

    float move_speed;
    float fast_speed;
    float mouse_sensitivity;

    float fov;      // Field of view (radians)
    float near;
    float far;
} FlyCamera;

// Initialize camera with defaults
void camera_init(FlyCamera* cam);

// Update camera from input
void camera_update(FlyCamera* cam, InputState* input, float dt);

// Get direction vectors
Vec3 camera_forward(FlyCamera* cam);
Vec3 camera_right(FlyCamera* cam);
Vec3 camera_up(FlyCamera* cam);

// Get matrices
Mat4 camera_view_matrix(FlyCamera* cam);
Mat4 camera_projection_matrix(FlyCamera* cam, float aspect);

// Generate a picking ray from screen coordinates (0,0 = top-left)
// ray_dir is returned normalized
void camera_screen_to_ray(FlyCamera* cam, int screen_x, int screen_y,
                          int screen_width, int screen_height,
                          Vec3* ray_origin, Vec3* ray_dir);

#endif // CAMERA_H
