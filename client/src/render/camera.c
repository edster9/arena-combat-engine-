#include "camera.h"
#include <math.h>

#define PI 3.14159265358979323846f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

void camera_init(FlyCamera* cam) {
    cam->position = vec3(0, 5, 10);
    cam->yaw = 0;
    cam->pitch = 0;

    cam->move_speed = 10.0f;
    cam->fast_speed = 30.0f;
    cam->mouse_sensitivity = 0.002f;

    cam->fov = 60.0f * DEG_TO_RAD;
    cam->near = 0.1f;
    cam->far = 1000.0f;
}

Vec3 camera_forward(FlyCamera* cam) {
    return (Vec3){
        sinf(cam->yaw) * cosf(cam->pitch),
        sinf(cam->pitch),
        -cosf(cam->yaw) * cosf(cam->pitch)
    };
}

Vec3 camera_right(FlyCamera* cam) {
    return (Vec3){
        cosf(cam->yaw),
        0,
        sinf(cam->yaw)
    };
}

Vec3 camera_up(FlyCamera* cam) {
    (void)cam;  // World up for simplicity (no roll)
    return vec3(0, 1, 0);
}

void camera_update(FlyCamera* cam, InputState* input, float dt) {
    // Mouse look only when captured (right-click held)
    // Skip frames until we see mouse movement, then skip that first movement
    // (SDL relative mode sends a big jump on first real input)
    if (input->mouse_captured) {
        bool has_movement = (input->mouse_dx != 0 || input->mouse_dy != 0);

        if (input->mouse_capture_just_started) {
            // Wait for first movement, then skip it
            if (has_movement) {
                input->mouse_capture_just_started = false;
            }
            // Don't apply any mouse input while waiting
        } else {
            cam->yaw -= input->mouse_dx * cam->mouse_sensitivity;
            cam->pitch += input->mouse_dy * cam->mouse_sensitivity;  // Inverted Y
        }
    }

    // Clamp pitch to avoid gimbal lock
    float max_pitch = 89.0f * DEG_TO_RAD;
    if (cam->pitch > max_pitch) cam->pitch = max_pitch;
    if (cam->pitch < -max_pitch) cam->pitch = -max_pitch;

    // Movement
    float speed = cam->move_speed;
    if (input->keys[KEY_LSHIFT] || input->keys[KEY_RSHIFT]) {
        speed = cam->fast_speed;
    }

    Vec3 forward = camera_forward(cam);
    Vec3 right = camera_right(cam);
    Vec3 move = vec3_zero();

    if (input->keys[KEY_W]) {
        move = vec3_add(move, forward);
    }
    if (input->keys[KEY_S]) {
        move = vec3_sub(move, forward);
    }
    if (input->keys[KEY_D]) {
        move = vec3_add(move, right);
    }
    if (input->keys[KEY_A]) {
        move = vec3_sub(move, right);
    }
    if (input->keys[KEY_E] || input->keys[KEY_SPACE]) {
        move.y += 1.0f;
    }
    if (input->keys[KEY_Q] || input->keys[KEY_LCTRL]) {
        move.y -= 1.0f;
    }

    // Normalize and apply speed
    float len = vec3_length(move);
    if (len > 0.001f) {
        move = vec3_scale(move, speed * dt / len);
        cam->position = vec3_add(cam->position, move);
    }

    // Scroll to adjust move speed
    if (input->scroll_y != 0) {
        cam->move_speed += input->scroll_y * 2.0f;
        if (cam->move_speed < 1.0f) cam->move_speed = 1.0f;
        if (cam->move_speed > 100.0f) cam->move_speed = 100.0f;
        cam->fast_speed = cam->move_speed * 3.0f;
    }
}

Mat4 camera_view_matrix(FlyCamera* cam) {
    Vec3 forward = camera_forward(cam);
    Vec3 target = vec3_add(cam->position, forward);
    return mat4_look_at(cam->position, target, vec3_up());
}

Mat4 camera_projection_matrix(FlyCamera* cam, float aspect) {
    return mat4_perspective(cam->fov, aspect, cam->near, cam->far);
}

void camera_screen_to_ray(FlyCamera* cam, int screen_x, int screen_y,
                          int screen_width, int screen_height,
                          Vec3* ray_origin, Vec3* ray_dir) {
    // Convert screen coords to normalized device coords (-1 to 1)
    float ndc_x = (2.0f * screen_x) / screen_width - 1.0f;
    float ndc_y = 1.0f - (2.0f * screen_y) / screen_height;  // Flip Y

    // Calculate ray direction in view space
    // Using the FOV and aspect ratio
    float aspect = (float)screen_width / (float)screen_height;
    float half_fov_tan = tanf(cam->fov * 0.5f);

    // View space ray direction (camera looks down -Z)
    Vec3 view_ray = vec3(
        ndc_x * half_fov_tan * aspect,
        ndc_y * half_fov_tan,
        -1.0f
    );

    // Transform to world space using camera orientation
    Vec3 forward = camera_forward(cam);
    Vec3 right = camera_right(cam);
    Vec3 up = vec3(0, 1, 0);

    // Build camera basis (right-handed, looking down -Z in view space)
    // world_dir = right * view_ray.x + up * view_ray.y + forward * view_ray.z
    Vec3 world_dir = vec3(
        right.x * view_ray.x + up.x * view_ray.y - forward.x * view_ray.z,
        right.y * view_ray.x + up.y * view_ray.y - forward.y * view_ray.z,
        right.z * view_ray.x + up.z * view_ray.y - forward.z * view_ray.z
    );

    *ray_origin = cam->position;
    *ray_dir = vec3_normalize(world_dir);
}
