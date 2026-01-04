/*
 * Arena - Vehicular Combat Game
 *
 * Branch: one-phase-physics-scripts
 * Transitioning to physics-based execution with Lua Reflex Scripts.
 * Turn-based kinematic system is DISABLED pending new implementation.
 *
 * See docs/proposals/ for the new architecture.
 */

// ============================================================================
// TURN-BASED SYSTEM: ENABLED
// Turn-based mode uses physics pause + Lua script execution.
// GUI maneuver declaration sends events to scripts for execution.
// Set to 0 for freestyle-only mode.
// ============================================================================
#define TURN_MODE_ENABLED 1

#include "platform/platform.h"
#include "math/vec3.h"
#include "math/mat4.h"
#include "render/camera.h"
#include "render/floor.h"
#include "render/mesh.h"
#include "render/obj_loader.h"
#include "render/texture.h"
#include "game/entity.h"
#include "render/line_render.h"
#include "ui/ui_render.h"
#include "ui/ui_text.h"
#include "physics/jolt_physics.h"
#include "game/config_loader.h"
#include "game/equipment_loader.h"
#include "game/maneuver.h"
#include "script/reflex_script.h"

#include <GL/glew.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// PI constant for rotation
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define WINDOW_TITLE "Arena"

// Arena dimensions
#define ARENA_SIZE 60.0f
#define WALL_HEIGHT 4.0f
#define WALL_THICKNESS 1.0f

// Car dimensions (for model scaling and ghost preview)
#define CAR_LENGTH 3.5f

// Vehicle mesh components (loaded separately for proper wheel positioning)
typedef struct {
    char type_name[64];    // Vehicle type name (e.g., "sports_car", "pickup_truck")
    char chassis_id[64];   // Chassis ID (e.g., "compact", "pickup")
    LoadedMesh body;       // Body + spoiler (no wheels)
    LoadedMesh wheel;      // Single wheel mesh (reused for all 4)
    Vec3 wheel_center;     // Center of wheel mesh (for positioning)
    float body_scale;      // Scale for body mesh
    float wheel_scale;     // Scale for wheel mesh
    GLuint texture;        // Body texture (0 = no texture, use color)
    bool loaded;           // True if meshes loaded successfully
} VehicleMesh;

// Support multiple vehicle types
#define MAX_VEHICLE_TYPES 8
static VehicleMesh g_vehicle_meshes[MAX_VEHICLE_TYPES];
static VehicleJSON g_vehicle_configs[MAX_VEHICLE_TYPES];
static int g_vehicle_type_count = 0;

// Map physics vehicle ID to vehicle type index
static int g_vehicle_type_map[MAX_PHYSICS_VEHICLES] = {0};

// Legacy compatibility: point to first vehicle mesh
static VehicleMesh* g_vehicle_mesh = &g_vehicle_meshes[0];

// Find vehicle type index by name, returns -1 if not found
static int find_vehicle_type(const char* type_name) {
    for (int i = 0; i < g_vehicle_type_count; i++) {
        if (strcmp(g_vehicle_meshes[i].type_name, type_name) == 0) {
            return i;
        }
    }
    return -1;
}

// Helper to read chassis ID from vehicle JSON file
static bool read_chassis_id_from_json(const char* filepath, char* chassis_id_out, int max_len) {
    FILE* f = fopen(filepath, "rb");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* data = (char*)malloc(size + 1);
    fread(data, 1, size, f);
    data[size] = '\0';
    fclose(f);

    // Simple parse for chassis field
    const char* chassis_start = strstr(data, "\"chassis\"");
    if (chassis_start) {
        // Find the value after the colon
        const char* colon = strchr(chassis_start, ':');
        if (colon) {
            // Skip whitespace
            const char* value = colon + 1;
            while (*value && (*value == ' ' || *value == '\t' || *value == '\n')) value++;
            // If it's a string (direct ID)
            if (*value == '"') {
                value++;
                const char* end = strchr(value, '"');
                if (end) {
                    int len = (int)(end - value);
                    if (len >= max_len) len = max_len - 1;
                    strncpy(chassis_id_out, value, len);
                    chassis_id_out[len] = '\0';
                    free(data);
                    return true;
                }
            }
        }
    }
    free(data);
    return false;
}

// Load a vehicle type (config + mesh), returns type index or -1 on failure
static int load_vehicle_type(const char* type_name) {
    // Check if already loaded
    int existing = find_vehicle_type(type_name);
    if (existing >= 0) return existing;

    // Check capacity
    if (g_vehicle_type_count >= MAX_VEHICLE_TYPES) {
        fprintf(stderr, "Too many vehicle types (max %d)\n", MAX_VEHICLE_TYPES);
        return -1;
    }

    int idx = g_vehicle_type_count;
    VehicleMesh* mesh = &g_vehicle_meshes[idx];
    VehicleJSON* config = &g_vehicle_configs[idx];

    // Initialize mesh struct
    memset(mesh, 0, sizeof(VehicleMesh));

    // Build vehicle config path
    char config_path[256];
    snprintf(config_path, sizeof(config_path), "../../assets/data/vehicles/%s.json", type_name);

    // Read chassis ID from JSON first (before full config load)
    char chassis_id[64] = {0};
    if (!read_chassis_id_from_json(config_path, chassis_id, sizeof(chassis_id))) {
        fprintf(stderr, "Could not find chassis in: %s\n", config_path);
        return -1;
    }

    // Store chassis ID
    strncpy(mesh->chassis_id, chassis_id, sizeof(mesh->chassis_id) - 1);

    // Load vehicle config
    if (!config_load_vehicle(config_path, config)) {
        fprintf(stderr, "Failed to load vehicle config: %s\n", config_path);
        return -1;
    }

    // Store type name
    strncpy(mesh->type_name, type_name, sizeof(mesh->type_name) - 1);
    mesh->type_name[sizeof(mesh->type_name) - 1] = '\0';

    // Get chassis equipment to find model path
    const ChassisEquipment* chassis = equipment_find_chassis(chassis_id);
    if (!chassis) {
        fprintf(stderr, "Chassis '%s' not found for vehicle '%s'\n", chassis_id, type_name);
        return -1;
    }

    // Build model paths
    char body_path[256];
    char wheel_path[256];
    snprintf(body_path, sizeof(body_path), "../../assets/%s", chassis->model);
    // Use standard wheel for now (could be per-tire later)
    snprintf(wheel_path, sizeof(wheel_path), "../../assets/models/wheels/wheel_standard.obj");

    printf("Loading vehicle type '%s':\n", type_name);
    printf("  Config: %s\n", config_path);
    printf("  Body:   %s\n", body_path);
    printf("  Wheel:  %s\n", wheel_path);

    // Load meshes
    bool body_loaded = obj_load(&mesh->body, body_path);
    bool wheel_loaded = obj_load(&mesh->wheel, wheel_path);

    if (body_loaded && wheel_loaded) {
        // Calculate body scale
        Vec3 body_size = obj_get_size(&mesh->body);
        float body_length = fmaxf(body_size.x, body_size.z);
        mesh->body_scale = (body_length > 0.001f) ? (CAR_LENGTH / body_length) : 1.0f;

        // Calculate wheel scale
        Vec3 wheel_size = obj_get_size(&mesh->wheel);
        float wheel_radius_model = fmaxf(wheel_size.x, wheel_size.y) * 0.5f;
        mesh->wheel_scale = 0.32f / wheel_radius_model;

        // Store wheel center
        mesh->wheel_center = obj_get_center(&mesh->wheel);
        mesh->loaded = true;

        // Update config dimensions from mesh if not set
        float scale = mesh->body_scale;
        float mesh_length = body_size.z * scale;
        float mesh_width = body_size.x * scale;
        float mesh_height = body_size.y * scale;

        if (config->chassis_length <= 0.01f) {
            config->chassis_length = mesh_length;
            config->physics.chassis_length = mesh_length;
        }
        if (config->chassis_width <= 0.01f) {
            config->chassis_width = mesh_width;
            config->physics.chassis_width = mesh_width;
        }
        if (config->chassis_height <= 0.01f) {
            config->chassis_height = mesh_height;
            config->physics.chassis_height = mesh_height;
        }

        printf("  Body:  %.2f x %.2f x %.2f, scale: %.2f\n",
               body_size.x, body_size.y, body_size.z, mesh->body_scale);
        printf("  Wheel: %.2f x %.2f x %.2f, scale: %.2f\n",
               wheel_size.x, wheel_size.y, wheel_size.z, mesh->wheel_scale);
        printf("  Physics dims: %.2fm x %.2fm x %.2fm\n",
               config->chassis_length, config->chassis_width, config->chassis_height);

        // Load texture if specified and mesh has UVs
        mesh->texture = 0;
        if (config->texture[0] != '\0' && mesh->body.has_uvs) {
            char texture_path[256];
            snprintf(texture_path, sizeof(texture_path), "../../assets/%s", config->texture);
            mesh->texture = texture_load(texture_path);
            if (mesh->texture) {
                printf("  Texture: %s (loaded)\n", config->texture);
            } else {
                printf("  Texture: %s (failed to load)\n", config->texture);
            }
        } else if (config->texture[0] != '\0' && !mesh->body.has_uvs) {
            printf("  Texture: %s (mesh has no UVs, skipping)\n", config->texture);
        }
    } else {
        if (!body_loaded) printf("  Warning: Could not load body mesh\n");
        if (!wheel_loaded) printf("  Warning: Could not load wheel mesh\n");
        mesh->loaded = false;
    }

    g_vehicle_type_count++;
    return idx;
}

// Get mesh for a physics vehicle
static VehicleMesh* get_vehicle_mesh(int physics_id) {
    if (physics_id < 0 || physics_id >= MAX_PHYSICS_VEHICLES) return g_vehicle_mesh;
    int type_idx = g_vehicle_type_map[physics_id];
    if (type_idx < 0 || type_idx >= g_vehicle_type_count) return g_vehicle_mesh;
    return &g_vehicle_meshes[type_idx];
}

// Get config for a physics vehicle
static VehicleJSON* get_vehicle_config(int physics_id) {
    if (physics_id < 0 || physics_id >= MAX_PHYSICS_VEHICLES) return &g_vehicle_configs[0];
    int type_idx = g_vehicle_type_map[physics_id];
    if (type_idx < 0 || type_idx >= g_vehicle_type_count) return &g_vehicle_configs[0];
    return &g_vehicle_configs[type_idx];
}

// Game mode is determined by physics pause state:
// - physics.paused = true  → Turn Based mode (planning turns)
// - physics.paused = false → Freestyle mode (free driving)
// Toggle with TAB key

// Planning UI state (for turn-based mode)
typedef enum {
    SPEED_BRAKE = 0,
    SPEED_HOLD = 1,
    SPEED_ACCEL = 2
} SpeedChoice;

typedef struct {
    SpeedChoice speed_choice;
    int current_speed;       // Current speed in mph (display only)

    // Snapshot speed when paused (for phase calculation)
    int snapshot_speed;      // Speed at moment of pause

    // Declared maneuver for this turn (default: MANEUVER_NONE = straight)
    ManeuverType maneuver;            // What maneuver for this turn
    ManeuverDirection direction;      // Direction (left/right)
    int bend_angle;                   // Bend angle (15,30,45)

    // Turn execution tracking
    bool turn_executing;              // True while a turn is being executed
    int turn_vehicle_id;              // Vehicle ID for the executing turn
} PlanningState;

// Physics state for freestyle mode
typedef struct {
    float velocity;          // Current speed in game units/sec
    float angular_velocity;  // Turning rate in radians/sec
    float steering;          // Current steering input (-1 to 1)
} CarPhysics;

// Physics constants
#define CAR_ACCEL 15.0f          // Acceleration in units/sec^2
#define CAR_BRAKE 25.0f          // Braking deceleration
#define CAR_FRICTION 3.0f        // Natural deceleration (drag)
#define CAR_MAX_SPEED 40.0f      // Max speed in units/sec (~90 mph in game scale)
#define CAR_TURN_RATE 2.5f       // Max turn rate in radians/sec
#define CAR_MIN_TURN_SPEED 2.0f  // Minimum speed to turn

// Calculate next speed based on choice
static int calculate_next_speed(int current_speed, SpeedChoice choice) {
    switch (choice) {
        case SPEED_BRAKE: return current_speed > 0 ? current_speed - 5 : 0;
        case SPEED_ACCEL: return current_speed + 5;
        case SPEED_HOLD:
        default: return current_speed;
    }
}

// Calculate movement distance in game units for a given speed
// tabletop: distance per turn = speed / 10 inches
// Our scale: 1 game unit = 1 inch
static float calculate_move_distance(int speed_mph) {
    return (float)speed_mph / 10.0f;
}

// Calculate end position for a straight-line move
static Vec3 calculate_end_position(Vec3 start, float rotation_y, float distance) {
    // Forward direction based on rotation
    float dx = sinf(rotation_y) * distance;
    float dz = cosf(rotation_y) * distance;
    return vec3(start.x + dx, start.y, start.z + dz);
}

// Get active phases for a given speed (tabletop rules)
// Returns bitmask: bit 0 = P1, bit 1 = P2, ..., bit 4 = P5
// 0 mph = P3 only (for starting from stop - ACCEL only, STRAIGHT only)
// 10 mph = P3 only (0b00100)
// 20 mph = P2, P4 (0b01010)
// 30 mph = P1, P3, P5 (0b10101)
// 40 mph = P1, P2, P4, P5 (0b11011)
// 50+ mph = all phases (0b11111)
static int get_active_phases(int speed_mph) {
    if (speed_mph < 5)  return 0b00100;     // 0-4 mph: P3 only (starting from stop)
    if (speed_mph < 15) return 0b00100;     // 5-14 mph (10 mph range): P3
    if (speed_mph < 25) return 0b01010;     // 15-24 mph (20 mph range): P2, P4
    if (speed_mph < 35) return 0b10101;     // 25-34 mph (30 mph range): P1, P3, P5
    if (speed_mph < 45) return 0b11011;     // 35-44 mph (40 mph range): P1, P2, P4, P5
    return 0b11111;                         // 50+ mph: all phases
}

// Check if point is inside rect
static bool point_in_rect(float px, float py, UIRect rect) {
    return px >= rect.x && px <= rect.x + rect.width &&
           py >= rect.y && py <= rect.y + rect.height;
}

// Convert ManeuverType to script-compatible name (for Lua maneuver module)
static const char* maneuver_to_script_name(ManeuverType type, int bend_angle) {
    switch (type) {
        case MANEUVER_NONE:
        case MANEUVER_STRAIGHT: return "straight";
        case MANEUVER_DRIFT: return "drift";
        case MANEUVER_STEEP_DRIFT: return "steep_drift";
        case MANEUVER_BEND:
            switch (bend_angle) {
                case 15: return "bend_15";
                case 30: return "bend_30";
                case 45: return "bend_45";
                case 60: return "bend_60";
                case 75: return "bend_75";
                case 90: return "bend_90";
                default: return "bend_45";
            }
        case MANEUVER_SWERVE: return "swerve";
        case MANEUVER_CONTROLLED_SKID: return "controlled_skid";
        case MANEUVER_T_STOP: return "t_stop";
        case MANEUVER_PIVOT: return "pivot";
        case MANEUVER_BOOTLEGGER: return "bootlegger";
        default: return "straight";
    }
}

// Draw arena walls using config
static void draw_arena_walls(BoxRenderer* r, ArenaConfig* cfg) {
    Vec3 wall_color = vec3(0.5f, 0.45f, 0.4f);  // Concrete grey-brown
    float half = cfg->size / 2.0f;
    float y = cfg->wall_height / 2.0f;
    float t = cfg->wall_thickness;

    // North wall (positive Z)
    box_renderer_draw(r,
        vec3(0, y, half + t/2),
        vec3(cfg->size + t*2, cfg->wall_height, t),
        wall_color);

    // South wall (negative Z)
    box_renderer_draw(r,
        vec3(0, y, -half - t/2),
        vec3(cfg->size + t*2, cfg->wall_height, t),
        wall_color);

    // East wall (positive X)
    box_renderer_draw(r,
        vec3(half + t/2, y, 0),
        vec3(t, cfg->wall_height, cfg->size),
        wall_color);

    // West wall (negative X)
    box_renderer_draw(r,
        vec3(-half - t/2, y, 0),
        vec3(t, cfg->wall_height, cfg->size),
        wall_color);
}

// Draw obstacles from scene config (boxes only - ramps drawn separately)
static void draw_obstacles(BoxRenderer* r, SceneJSON* scene) {
    for (int i = 0; i < scene->obstacle_count; i++) {
        SceneObstacle* o = &scene->obstacles[i];
        if (strcmp(o->type, "ramp") == 0) continue;  // Ramps drawn as wireframes
        box_renderer_draw(r, o->position, o->size, o->color);
    }
}

// Draw ramps as solid wedge shapes using triangles
static void draw_ramps(BoxRenderer* r, SceneJSON* scene) {
    for (int i = 0; i < scene->obstacle_count; i++) {
        SceneObstacle* o = &scene->obstacles[i];
        if (strcmp(o->type, "ramp") != 0) continue;

        // Ramp: wedge shape with size.x=width, size.y=height (at high end), size.z=length
        // Low end at -Z, high end at +Z (before rotation)
        // Position is at center-bottom
        float hw = o->size.x * 0.5f;  // half width
        float h = o->size.y;          // full height
        float hl = o->size.z * 0.5f;  // half length

        // Apply Y rotation
        float cos_r = cosf(o->rotation_y);
        float sin_r = sinf(o->rotation_y);

        // Transform local point to world
        auto transform = [&](float lx, float ly, float lz) -> Vec3 {
            return vec3(
                o->position.x + lx * cos_r - lz * sin_r,
                o->position.y + ly,
                o->position.z + lx * sin_r + lz * cos_r
            );
        };

        // Draw as series of boxes to approximate the ramp surface visually
        // We'll draw thin boxes along the slope for a reasonable visual
        const int SLICES = 8;
        for (int s = 0; s < SLICES; s++) {
            float t0 = (float)s / SLICES;
            float t1 = (float)(s + 1) / SLICES;

            // Height at each slice position
            float h0 = h * t0;
            float h1 = h * t1;

            // Z position along the ramp
            float z0 = -hl + (hl * 2.0f) * t0;
            float z1 = -hl + (hl * 2.0f) * t1;

            // Center of this slice segment
            float slice_z = (z0 + z1) * 0.5f;
            float slice_h = (h0 + h1) * 0.5f;
            float slice_depth = (z1 - z0);

            // Transform the slice center
            Vec3 slice_pos = transform(0, slice_h * 0.5f, slice_z);

            // Create rotation matrix for this slice (Y rotation only)
            float rot_matrix[9] = {
                cos_r,  0, sin_r,
                0,      1, 0,
                -sin_r, 0, cos_r
            };

            Vec3 slice_size = vec3(o->size.x, slice_h, slice_depth);
            box_renderer_draw_rotated_matrix(r, slice_pos, slice_size, rot_matrix, o->color);
        }
    }
}

// Draw physics vehicles as solid primitives (box chassis only, wheels drawn separately)
// team_for_vehicle maps physics vehicle id -> team (TEAM_RED, TEAM_BLUE, etc.)
static void draw_physics_vehicles(BoxRenderer* r, PhysicsWorld* pw, int selected_id, Team* team_for_vehicle) {
    // Team colors
    Vec3 red_color = vec3(0.8f, 0.2f, 0.2f);         // Red team
    Vec3 red_selected = vec3(1.0f, 0.4f, 0.4f);      // Red selected
    Vec3 blue_color = vec3(0.2f, 0.4f, 0.9f);        // Blue team
    Vec3 blue_selected = vec3(0.4f, 0.6f, 1.0f);     // Blue selected
    Vec3 front_color = vec3(0.9f, 0.9f, 0.2f);       // Yellow front indicator

    for (int i = 0; i < pw->vehicle_count; i++) {
        if (!pw->vehicles[i].active) continue;

        // Get team-based color
        Team team = team_for_vehicle ? team_for_vehicle[i] : TEAM_RED;
        Vec3 base_color = (team == TEAM_BLUE) ? blue_color : red_color;
        Vec3 sel_color = (team == TEAM_BLUE) ? blue_selected : red_selected;
        Vec3 color = (i == selected_id) ? sel_color : base_color;

        // Get chassis position and full rotation from physics
        Vec3 pos;
        float rot_matrix[9];  // 3x3 row-major rotation matrix
        physics_vehicle_get_position(pw, i, &pos);
        physics_vehicle_get_rotation_matrix(pw, i, rot_matrix);

        // Get vehicle config for dimensions
        VehicleConfig* cfg = &pw->vehicles[i].config;

        // Get the correct mesh for this vehicle type
        VehicleMesh* vmesh = get_vehicle_mesh(i);

        // Draw chassis - use loaded mesh if available, otherwise box
        if (vmesh->loaded) {
            // Use full rotation matrix for proper roll/pitch during flips
            // Pre-translate to offset mesh center to match physics center
            Vec3 body_center = obj_get_center(&vmesh->body);
            Vec3 pre_translate = vec3(-body_center.x, -body_center.y, -body_center.z);

            if (vmesh->texture && vmesh->body.has_uvs) {
                // Use textured rendering
                box_renderer_draw_mesh_textured(r, vmesh->body.vao, vmesh->body.vertex_count,
                                                pos, vmesh->body_scale, rot_matrix,
                                                pre_translate, vmesh->texture);
            } else {
                // Fallback to color-based rendering
                box_renderer_draw_mesh_rotated(r, vmesh->body.vao, vmesh->body.vertex_count,
                                               pos, vmesh->body_scale, rot_matrix,
                                               pre_translate, color);
            }
            // Wheels rendered separately by draw_vehicle_wheels_mesh()
        } else {
            Vec3 chassis_size = vec3(cfg->chassis_width, cfg->chassis_height, cfg->chassis_length);
            box_renderer_draw_rotated_matrix(r, pos, chassis_size, rot_matrix, color);

            // Draw front indicator (yellow bar at front of car) - only for box mode
            float front_offset = cfg->chassis_length * 0.35f;
            float indicator_height = 0.3f;
            float local_y = cfg->chassis_height * 0.5f + indicator_height * 0.5f;
            Vec3 front_pos = vec3(
                pos.x + rot_matrix[2] * front_offset + rot_matrix[1] * local_y,
                pos.y + rot_matrix[5] * front_offset + rot_matrix[4] * local_y,
                pos.z + rot_matrix[8] * front_offset + rot_matrix[7] * local_y
            );
            Vec3 front_size = vec3(cfg->chassis_width * 0.7f, indicator_height, 0.4f);
            box_renderer_draw_rotated_matrix(r, front_pos, front_size, rot_matrix, front_color);
        }
    }
}

// Draw wheels as cylinders with rotating spokes (batched - efficient)
// Uses Jolt wheel transform for correct display during rollovers
static void draw_vehicle_wheels(LineRenderer* lr, PhysicsWorld* pw) {
    Vec3 rim_color = vec3(0.3f, 0.3f, 0.35f);     // Dark grey rim
    Vec3 spoke_color = vec3(0.8f, 0.8f, 0.8f);    // Light grey spokes

    for (int i = 0; i < pw->vehicle_count; i++) {
        if (!pw->vehicles[i].active) continue;

        VehicleConfig* cfg = &pw->vehicles[i].config;
        WheelState wheels[4];
        physics_vehicle_get_wheel_states(pw, i, wheels);

        for (int w = 0; w < 4; w++) {
            Vec3 center = wheels[w].position;
            float radius = cfg->use_per_wheel_config ? cfg->wheel_radii[w] : cfg->wheel_radius;
            float width = cfg->use_per_wheel_config ? cfg->wheel_widths[w] : cfg->wheel_width;
            float halfWidth = width * 0.5f;
            float spin = wheels[w].rotation;

            // Get wheel orientation from Jolt rotation matrix
            // Jolt stores column-major: [0-2]=X, [3-5]=Y, [6-8]=Z
            // Wheel Y axis is the axle, circle is in X-Z plane
            float* rot = wheels[w].rot_matrix;

            // Local X axis (in world coords) - horizontal extent of wheel circle
            Vec3 axis_x = vec3(rot[0], rot[1], rot[2]);
            // Local Y axis (in world coords) - axle direction (for width offset)
            Vec3 axis_y = vec3(rot[3], rot[4], rot[5]);
            // Local Z axis (in world coords) - vertical extent of wheel circle
            Vec3 axis_z = vec3(rot[6], rot[7], rot[8]);

            // Offset for inner and outer rim circles
            Vec3 inner_offset = vec3(axis_y.x * halfWidth, axis_y.y * halfWidth, axis_y.z * halfWidth);
            Vec3 outer_offset = vec3(-axis_y.x * halfWidth, -axis_y.y * halfWidth, -axis_y.z * halfWidth);

            // Draw cylinder: two rim circles + connecting lines
            const int RIM_SEGMENTS = 12;
            for (int s = 0; s < RIM_SEGMENTS; s++) {
                float a1 = (float)s / RIM_SEGMENTS * 2.0f * (float)M_PI;
                float a2 = (float)(s + 1) / RIM_SEGMENTS * 2.0f * (float)M_PI;

                // Points on rim circle using local X and Z axes
                float cos_a1 = cosf(a1) * radius;
                float sin_a1 = sinf(a1) * radius;
                float cos_a2 = cosf(a2) * radius;
                float sin_a2 = sinf(a2) * radius;

                // Inner rim circle (closer to chassis)
                Vec3 inner1 = vec3(
                    center.x + inner_offset.x + axis_x.x * cos_a1 + axis_z.x * sin_a1,
                    center.y + inner_offset.y + axis_x.y * cos_a1 + axis_z.y * sin_a1,
                    center.z + inner_offset.z + axis_x.z * cos_a1 + axis_z.z * sin_a1
                );
                Vec3 inner2 = vec3(
                    center.x + inner_offset.x + axis_x.x * cos_a2 + axis_z.x * sin_a2,
                    center.y + inner_offset.y + axis_x.y * cos_a2 + axis_z.y * sin_a2,
                    center.z + inner_offset.z + axis_x.z * cos_a2 + axis_z.z * sin_a2
                );
                line_renderer_draw_line(lr, inner1, inner2, rim_color, 1.0f);

                // Outer rim circle (away from chassis)
                Vec3 outer1 = vec3(
                    center.x + outer_offset.x + axis_x.x * cos_a1 + axis_z.x * sin_a1,
                    center.y + outer_offset.y + axis_x.y * cos_a1 + axis_z.y * sin_a1,
                    center.z + outer_offset.z + axis_x.z * cos_a1 + axis_z.z * sin_a1
                );
                Vec3 outer2 = vec3(
                    center.x + outer_offset.x + axis_x.x * cos_a2 + axis_z.x * sin_a2,
                    center.y + outer_offset.y + axis_x.y * cos_a2 + axis_z.y * sin_a2,
                    center.z + outer_offset.z + axis_x.z * cos_a2 + axis_z.z * sin_a2
                );
                line_renderer_draw_line(lr, outer1, outer2, rim_color, 1.0f);

                // Connect inner and outer every 3rd segment (cleaner look)
                if (s % 3 == 0) {
                    line_renderer_draw_line(lr, inner1, outer1, rim_color, 1.0f);
                }
            }

            // Draw 4 rotating spokes on outer face
            for (int s = 0; s < 4; s++) {
                float spoke_angle = spin + (float)s * (float)M_PI * 0.5f;
                float spoke_cos = cosf(spoke_angle) * radius * 0.85f;
                float spoke_sin = sinf(spoke_angle) * radius * 0.85f;

                Vec3 spoke_center = vec3(
                    center.x + outer_offset.x,
                    center.y + outer_offset.y,
                    center.z + outer_offset.z
                );
                Vec3 spoke_end = vec3(
                    center.x + outer_offset.x + axis_x.x * spoke_cos + axis_z.x * spoke_sin,
                    center.y + outer_offset.y + axis_x.y * spoke_cos + axis_z.y * spoke_sin,
                    center.z + outer_offset.z + axis_x.z * spoke_cos + axis_z.z * spoke_sin
                );
                line_renderer_draw_line(lr, spoke_center, spoke_end, spoke_color, 1.0f);
            }
        }
    }
}

// Draw wheels using loaded mesh at physics wheel positions
// Uses Jolt wheel transform for correct display during rollovers
static void draw_vehicle_wheels_mesh(BoxRenderer* r, PhysicsWorld* pw) {
    Vec3 wheel_color = vec3(0.2f, 0.2f, 0.22f);  // Dark tire color

    for (int i = 0; i < pw->vehicle_count; i++) {
        if (!pw->vehicles[i].active) continue;

        // Get the correct mesh for this vehicle type
        VehicleMesh* vmesh = get_vehicle_mesh(i);
        if (!vmesh->loaded) continue;

        // Pre-translation to center the wheel mesh at origin
        Vec3 center_offset = vec3(-vmesh->wheel_center.x,
                                  -vmesh->wheel_center.y,
                                  -vmesh->wheel_center.z);

        VehicleConfig* cfg = &pw->vehicles[i].config;
        WheelState wheels[4];
        physics_vehicle_get_wheel_states(pw, i, wheels);

        for (int w = 0; w < 4; w++) {
            Vec3 center = wheels[w].position;
            float radius = cfg->use_per_wheel_config ? cfg->wheel_radii[w] : cfg->wheel_radius;

            // Scale wheel mesh to match physics wheel radius
            float wheel_scale = vmesh->wheel_scale * (radius / 0.32f);
            Vec3 scale = vec3(wheel_scale, wheel_scale, wheel_scale);

            // Wheel rotation matrix from physics (column-major)
            // Jolt: [0-2]=X (right), [3-5]=Y (axle), [6-8]=Z (up)
            float* rot = wheels[w].rot_matrix;

            // Kenney wheel mesh has axle along X, but Jolt expects axle along Y
            // Apply 90° rotation around Z to fix: swap X<->Y axes
            // New X = old Y, New Y = -old X
            // Also flip for left-side wheels
            bool is_left = (w == 0 || w == 2);  // FL=0, RL=2
            float flip = is_left ? -1.0f : 1.0f;

            // Corrected rotation: R_jolt * R_90z
            // This swaps axes so mesh axle (X) aligns with Jolt axle (Y)
            float corrected_rot[9] = {
                rot[3] * flip, rot[4] * flip, rot[5] * flip,  // New X = Jolt Y (axle) * flip
                -rot[0], -rot[1], -rot[2],                     // New Y = -Jolt X
                rot[6], rot[7], rot[8]                         // Z unchanged
            };

            box_renderer_draw_mesh_matrix(r, vmesh->wheel.vao,
                                          vmesh->wheel.vertex_count,
                                          center, scale, corrected_rot,
                                          center_offset, wheel_color);
        }
    }
}

// Create vehicles from scene config
static void create_vehicles_from_scene(EntityManager* em, SceneJSON* scene, float car_scale) {
    for (int i = 0; i < scene->vehicle_count; i++) {
        SceneVehicle* sv = &scene->vehicles[i];

        Team team = TEAM_RED;
        if (strcmp(sv->team, "blue") == 0) team = TEAM_BLUE;

        Entity* e = entity_manager_create(em, ENTITY_VEHICLE, team);
        e->position = sv->position;
        e->rotation_y = sv->rotation;
        e->scale = car_scale;
    }

    printf("Created %d vehicles from scene config\n", em->count);
}

// Global verbose flag for debug output
static bool g_verbose = false;

int main(int argc, char* argv[]) {
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0) {
            g_verbose = true;
        }
    }

    printf("=== Arena ===\n");
    printf("Press F1 for controls help\n");
    if (g_verbose) printf("Verbose mode enabled\n");
    printf("\n");

    // Initialize platform
    Platform platform;
    if (!platform_init(&platform, WINDOW_TITLE, WINDOW_WIDTH, WINDOW_HEIGHT)) {
        fprintf(stderr, "Failed to initialize platform\n");
        return 1;
    }

    // Initialize camera - position to see both cars from an angle
    FlyCamera camera;
    camera_init(&camera);
    camera.position = vec3(50, 25, 0);  // Side view, elevated
    camera.yaw = -1.57f;                // Look toward center (-90 degrees)
    camera.pitch = -0.4f;               // Look slightly down

    // Floor will be initialized after scene config is loaded
    Floor arena_floor;

    // Initialize box renderer for walls and obstacles
    BoxRenderer box_renderer;
    if (!box_renderer_init(&box_renderer)) {
        fprintf(stderr, "Failed to initialize box renderer\n");
        platform_shutdown(&platform);
        return 1;
    }

    // Load equipment data first (required for vehicle config parsing)
    if (!equipment_load_all("../../assets/data/equipment")) {
        fprintf(stderr, "Warning: Equipment data not loaded - using defaults\n");
    }

    // Load scene config
    SceneJSON scene_config;
    bool scene_ok = config_load_scene("../../assets/config/scenes/showdown.json", &scene_config);
    if (!scene_ok) {
        fprintf(stderr, "\n*** ERROR: Scene config failed to load! ***\n");
        fprintf(stderr, "*** Check assets/config/scenes/showdown.json ***\n\n");
    }

    // Load all vehicle types from scene
    bool vehicle_ok = false;
    if (scene_ok) {
        printf("\n=== Loading Vehicle Types ===\n");
        for (int i = 0; i < scene_config.vehicle_count; i++) {
            const char* type_name = scene_config.vehicles[i].type;
            int type_idx = load_vehicle_type(type_name);
            if (type_idx >= 0) {
                vehicle_ok = true;  // At least one vehicle loaded
            }
        }
        printf("=== Loaded %d vehicle type(s) ===\n\n", g_vehicle_type_count);
    }

    if (!vehicle_ok) {
        fprintf(stderr, "\n*** ERROR: No vehicle configs loaded! ***\n");
        fprintf(stderr, "*** Arena will be empty - no vehicles! ***\n\n");
    }

    // For legacy compatibility, point to first vehicle config
    VehicleJSON& vehicle_config = g_vehicle_configs[0];

    // Initialize entity manager and create vehicles from scene (only if scene loaded)
    EntityManager entities;
    entity_manager_init(&entities);
    if (scene_ok && vehicle_ok) {
        float entity_scale = g_vehicle_mesh->loaded ? g_vehicle_mesh->body_scale : 1.0f;
        create_vehicles_from_scene(&entities, &scene_config, entity_scale);
    }

    // Initialize ODE physics
    PhysicsWorld physics;
    if (!physics_init(&physics)) {
        fprintf(stderr, "Failed to initialize physics\n");
        // Continue anyway, physics just won't work
    }

    // Initialize Reflex Script Engine (Lua/Sol3)
    // Scripts are loaded per-vehicle in the vehicle creation loop below
    ReflexScriptEngine* script_engine = reflex_create();

    // Set up ground plane from scene config
    physics_set_ground(&physics, scene_config.arena.ground_y);

    // Initialize floor with arena size from config (1 unit grid = tabletop scale)
    if (!floor_init(&arena_floor, scene_config.arena.size, 1.0f)) {
        fprintf(stderr, "Failed to initialize floor\n");
        physics_destroy(&physics);
        box_renderer_destroy(&box_renderer);
        platform_shutdown(&platform);
        return 1;
    }

    // Add arena walls to physics
    physics_add_arena_walls(&physics, scene_config.arena.size,
                            scene_config.arena.wall_height,
                            scene_config.arena.wall_thickness);

    // Add obstacles to physics from scene config
    for (int i = 0; i < scene_config.obstacle_count; i++) {
        SceneObstacle* o = &scene_config.obstacles[i];
        if (strcmp(o->type, "ramp") == 0) {
            physics_add_ramp_obstacle(&physics, o->position, o->size, o->rotation_y);
        } else {
            physics_add_box_obstacle(&physics, o->position, o->size);
        }
    }

    // Create physics vehicles for each entity using per-vehicle type configs
    // Only if vehicle configs loaded successfully
    int entity_to_physics[MAX_ENTITIES];  // Maps entity id -> physics vehicle id
    memset(entity_to_physics, -1, sizeof(entity_to_physics));

    if (vehicle_ok) {
        int scene_vehicle_idx = 0;  // Track which scene vehicle we're creating
        for (int i = 0; i < entities.count; i++) {
            Entity* e = &entities.entities[i];
            if (e->active && e->type == ENTITY_VEHICLE) {
                // Get vehicle type from scene config
                const char* type_name = scene_config.vehicles[scene_vehicle_idx].type;
                int type_idx = find_vehicle_type(type_name);
                if (type_idx < 0) type_idx = 0;  // Fallback to first type

                VehicleJSON* vconfig = &g_vehicle_configs[type_idx];
                VehicleConfig vehicle_cfg = config_vehicle_to_physics(vconfig);

                int phys_id = physics_create_vehicle(&physics, e->position, e->rotation_y, &vehicle_cfg);
                entity_to_physics[e->id] = phys_id;

                // Map physics ID to vehicle type for rendering
                g_vehicle_type_map[phys_id] = type_idx;

                printf("Created vehicle %d: type='%s' (type_idx=%d)\n", phys_id, type_name, type_idx);

                // Attach scripts to this vehicle (each vehicle gets its own isolated script instance)
                if (script_engine) {
                    for (int si = 0; si < vconfig->script_count; si++) {
                        VehicleScript* vs = &vconfig->scripts[si];
                        if (!vs->enabled) continue;

                        // Build config arrays
                        const char* keys[MAX_SCRIPT_OPTIONS];
                        float values[MAX_SCRIPT_OPTIONS];
                        for (int opt = 0; opt < vs->option_count; opt++) {
                            keys[opt] = vs->options[opt].key;
                            values[opt] = vs->options[opt].value;
                        }

                        // Attach script with config (creates isolated instance for this vehicle)
                        reflex_attach_script(script_engine, phys_id, vs->name, vs->path,
                                             keys, values, vs->option_count);
                    }
                }

                scene_vehicle_idx++;
            }
        }
    }

    // Initialize UI renderer
    UIRenderer ui_renderer;
    if (!ui_renderer_init(&ui_renderer)) {
        fprintf(stderr, "Failed to initialize UI renderer\n");
        // Continue anyway, just won't have UI
    }

    // Initialize text renderer
    TextRenderer text_renderer;
    bool has_text = text_renderer_init(&text_renderer, "../../assets/fonts/Roboto-Bold.ttf", 18.0f);
    if (!has_text) {
        fprintf(stderr, "Failed to initialize text renderer\n");
        // Continue anyway, just won't have text
    }

    // Initialize line renderer for ghost path
    LineRenderer line_renderer;
    bool has_lines = line_renderer_init(&line_renderer);
    if (!has_lines) {
        fprintf(stderr, "Failed to initialize line renderer\n");
    }

    // Light direction (sun from upper-right-front)
    Vec3 light_dir = vec3_normalize(vec3(0.5f, -1.0f, 0.3f));

    // Input state
    InputState input;
    memset(&input, 0, sizeof(InputState));

    // OpenGL setup
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);

    // Planning state
    PlanningState planning = {
        .speed_choice = SPEED_HOLD,
        .current_speed = 0,  // Starting from standstill
        .snapshot_speed = 0,
        .maneuver = MANEUVER_NONE,
        .direction = MANEUVER_LEFT,
        .bend_angle = 0,
        .turn_executing = false,
        .turn_vehicle_id = -1
    };

    // Debug flags
    bool show_cars = true;       // Toggle with 'H' key
    bool debug_ghost = false;    // Toggle with 'G' key for debug output
    bool show_physics_debug = false;  // Toggle with 'P' key for physics shapes
    bool show_help = false;      // Toggle with 'F1' key for help overlay

    // Chase camera mode (C to toggle) - spherical orbit around car
    bool chase_camera = false;
    float chase_distance = 10.0f;       // Distance from car (radius of orbit sphere)
    float chase_azimuth = 0.0f;         // Horizontal orbit angle (radians)
    float chase_elevation = 0.5f;       // Vertical orbit angle (radians, 0=level, PI/2=overhead)

    // Game mode is now unified with physics pause state:
    // - physics.paused = true  → Turn Based mode
    // - physics.paused = false → Freestyle mode
    // Toggle with TAB key (F key removed)

    // Scripted turn state (physics-based maneuver execution via Lua scripts)
    // When active, scripts apply controls to achieve maneuver goals over 1 second
    struct {
        bool active;              // Is a scripted turn in progress?
        int vehicle_id;           // Which vehicle is executing the turn
        float elapsed;            // Time elapsed in turn (0 to 1.0)
        float duration;           // Turn duration (always 1.0s)
        const char* maneuver;     // Current maneuver type name
        const char* direction;    // "left", "right", or "center"
    } scripted_turn = {false, -1, 0.0f, 1.0f, nullptr, nullptr};

    // Steering state (accessible for status bar display)
    bool discrete_steering = false;  // Start in gradual mode (easier to drive)
    int steering_level = 0;         // -5 to +5 for discrete mode

    // Manual throttle override (for debugging TCS/drivetrain)
    // -1 = disabled (use UP key), 0-6 = manual levels: 5%, 10%, 15%, 20%, 30%, 40%, 50%
    int manual_throttle_level = -1;
    float manual_throttle_values[7] = {0.05f, 0.10f, 0.15f, 0.20f, 0.30f, 0.40f, 0.50f};
    float current_steering = 0.0f;  // Current steering value

    // Physics state for each car (indexed by entity id)
    CarPhysics car_physics[MAX_ENTITIES];
    memset(car_physics, 0, sizeof(car_physics));

    // Timing
    double last_time = platform_get_time();
    int frame_count = 0;
    double fps_timer = 0;

    // Main loop
    while (!platform.should_quit) {
        // Timing
        double current_time = platform_get_time();
        float dt = (float)(current_time - last_time);
        last_time = current_time;

        // FPS counter
        frame_count++;
        fps_timer += dt;
        if (fps_timer >= 1.0) {
            char title[128];
            snprintf(title, sizeof(title), "%s | FPS: %d | Pos: (%.1f, %.1f, %.1f)",
                     WINDOW_TITLE, frame_count,
                     camera.position.x, camera.position.y, camera.position.z);
            platform_set_title(&platform, title);
            frame_count = 0;
            fps_timer = 0;
        }

        // Input
        platform_poll_events(&platform, &input);

        // Toggle mouse capture with middle click (for camera orbit/look)
        if (input.mouse_pressed[MOUSE_MIDDLE]) {
            platform_capture_mouse(&platform, &input, true);
        }
        if (input.mouse_released[MOUSE_MIDDLE]) {
            platform_capture_mouse(&platform, &input, false);
        }

        // Number keys 1-3 select vehicles and enable chase camera
        {
            int key_to_vehicle[3] = {KEY_1, KEY_2, KEY_3};

            for (int v = 0; v < 3; v++) {
                if (input.keys_pressed[key_to_vehicle[v]]) {
                    // Find entity that maps to physics vehicle v
                    int target_phys_id = v;
                    if (target_phys_id < physics.vehicle_count && physics.vehicles[target_phys_id].active) {
                        // Find the entity with this physics id
                        for (int i = 0; i < entities.count; i++) {
                            Entity* e = &entities.entities[i];
                            if (e->active && e->id < MAX_ENTITIES && entity_to_physics[e->id] == target_phys_id) {
                                entity_manager_select(&entities, e->id);
                                printf("Selected vehicle %d\n", v + 1);

                                // Also enable chase camera
                                if (!chase_camera) {
                                    chase_camera = true;
                                    // Initialize chase camera from current position
                                    Vec3 car_pos;
                                    physics_vehicle_get_position(&physics, target_phys_id, &car_pos);
                                    Vec3 offset = vec3_sub(camera.position, car_pos);
                                    chase_distance = 10.0f;
                                    chase_azimuth = atan2f(offset.x, offset.z);
                                    float horiz_dist = sqrtf(offset.x * offset.x + offset.z * offset.z);
                                    chase_elevation = atan2f(offset.y, horiz_dist);
                                    if (chase_elevation < 0.175f) chase_elevation = 0.175f;
                                    if (chase_elevation > 1.22f) chase_elevation = 1.22f;
                                    printf("Chase camera ON\n");
                                }
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }

        // Keys 4-0 set manual throttle levels (for debugging TCS/drivetrain)
        // 4=5%, 5=10%, 6=15%, 7=20%, 8=30%, 9=40%, 0=50%
        // UP key when manual throttle is active will clear it (return to full throttle)
        {
            int throttle_keys[7] = {KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0};
            const char* throttle_labels[7] = {"5%", "10%", "15%", "20%", "30%", "40%", "50%"};

            for (int t = 0; t < 7; t++) {
                if (input.keys_pressed[throttle_keys[t]]) {
                    manual_throttle_level = t;
                    printf("Manual throttle: %s (%.0f%%)\n", throttle_labels[t], manual_throttle_values[t] * 100.0f);
                    break;
                }
            }
        }

        // Fullscreen toggle
        if (input.keys_pressed[KEY_F11]) {
            platform_toggle_fullscreen(&platform);
        }

        // Help overlay toggle
        if (input.keys_pressed[KEY_F1]) {
            show_help = !show_help;
        }

        // Quit on ESC
        if (input.keys_pressed[KEY_ESCAPE]) {
            platform.should_quit = true;
        }

        // Toggle car visibility with H
        if (input.keys_pressed[KEY_H]) {
            show_cars = !show_cars;
            printf("Cars %s\n", show_cars ? "visible" : "hidden");
        }

        // Toggle ghost debug with G
        if (input.keys_pressed[KEY_G]) {
            debug_ghost = !debug_ghost;
            printf("Ghost debug %s\n", debug_ghost ? "ON" : "OFF");
        }

        // Toggle physics debug with P
        if (input.keys_pressed[KEY_P]) {
            show_physics_debug = !show_physics_debug;
            printf("Physics debug %s\n", show_physics_debug ? "ON" : "OFF");
        }

        // Debug: flip selected vehicle upside down with F
        if (input.keys_pressed[KEY_F]) {
            Entity* sel = entity_manager_get_selected(&entities);
            if (sel && sel->id < MAX_ENTITIES && entity_to_physics[sel->id] >= 0) {
                int phys_id = entity_to_physics[sel->id];
                physics_vehicle_flip(&physics, phys_id);
            } else {
                printf("No vehicle selected to flip\n");
            }
        }

        // Toggle chase camera with C (requires selected vehicle)
        if (input.keys_pressed[KEY_C]) {
            Entity* sel = entity_manager_get_selected(&entities);
            if (sel && sel->id < MAX_ENTITIES && entity_to_physics[sel->id] >= 0) {
                chase_camera = !chase_camera;
                printf("Chase camera %s\n", chase_camera ? "ON" : "OFF");

                // Initialize chase camera from current camera position
                if (chase_camera) {
                    int phys_id = entity_to_physics[sel->id];
                    Vec3 car_pos;
                    physics_vehicle_get_position(&physics, phys_id, &car_pos);

                    // Calculate spherical coords from camera to car
                    Vec3 offset = vec3_sub(camera.position, car_pos);
                    // Start at a close distance, not the current camera distance
                    chase_distance = 10.0f;

                    // Azimuth: horizontal angle (atan2 of x/z offset)
                    chase_azimuth = atan2f(offset.x, offset.z);

                    // Elevation: vertical angle from horizontal
                    float horiz_dist = sqrtf(offset.x * offset.x + offset.z * offset.z);
                    chase_elevation = atan2f(offset.y, horiz_dist);
                    if (chase_elevation < 0.175f) chase_elevation = 0.175f;
                    if (chase_elevation > 1.22f) chase_elevation = 1.22f;

                    if (g_verbose) {
                        printf("Chase init: cam=(%.1f,%.1f,%.1f) car=(%.1f,%.1f,%.1f)\n",
                               camera.position.x, camera.position.y, camera.position.z,
                               car_pos.x, car_pos.y, car_pos.z);
                        printf("  offset=(%.1f,%.1f,%.1f) dist=%.1f az=%.3f el=%.3f\n",
                               offset.x, offset.y, offset.z, chase_distance, chase_azimuth, chase_elevation);
                    }
                }
            } else {
                printf("Select a vehicle first (C)\n");
            }
        }

        // Reload vehicle config and recreate all vehicles with R
        if (input.keys_pressed[KEY_R]) {
            printf("Reloading configs...\n");
            // Reload equipment first (suspension.json, tires.json, etc.)
            equipment_load_all("../../assets/data/equipment");

            // Collect old spawn info and vehicle types before destroying
            int old_count = physics.vehicle_count;
            Vec3 spawn_positions[MAX_PHYSICS_VEHICLES];
            float spawn_rotations[MAX_PHYSICS_VEHICLES];
            int old_type_map[MAX_PHYSICS_VEHICLES];
            for (int i = 0; i < old_count; i++) {
                spawn_positions[i] = physics.vehicles[i].spawn_position;
                spawn_rotations[i] = physics.vehicles[i].spawn_rotation;
                old_type_map[i] = g_vehicle_type_map[i];
            }

            // Reload all vehicle type configs (but don't reload meshes - they're already in GPU)
            bool reload_ok = true;
            for (int t = 0; t < g_vehicle_type_count; t++) {
                VehicleMesh* mesh = &g_vehicle_meshes[t];
                VehicleJSON* config = &g_vehicle_configs[t];

                char config_path[256];
                snprintf(config_path, sizeof(config_path), "../../assets/data/vehicles/%s.json", mesh->type_name);

                VehicleJSON new_config;
                if (config_load_vehicle(config_path, &new_config)) {
                    *config = new_config;

                    // Override chassis dimensions from mesh bounds
                    if (mesh->loaded) {
                        Vec3 body_size = obj_get_size(&mesh->body);
                        float scale = mesh->body_scale;
                        if (config->chassis_length <= 0.01f) {
                            config->chassis_length = body_size.z * scale;
                            config->physics.chassis_length = config->chassis_length;
                        }
                        if (config->chassis_width <= 0.01f) {
                            config->chassis_width = body_size.x * scale;
                            config->physics.chassis_width = config->chassis_width;
                        }
                        if (config->chassis_height <= 0.01f) {
                            config->chassis_height = body_size.y * scale;
                            config->physics.chassis_height = config->chassis_height;
                        }
                    }
                    printf("Reloaded config: %s\n", mesh->type_name);
                } else {
                    fprintf(stderr, "Failed to reload: %s\n", mesh->type_name);
                    reload_ok = false;
                }
            }

            if (reload_ok) {
                // Destroy all vehicles
                for (int i = 0; i < old_count; i++) {
                    if (physics.vehicles[i].active) {
                        physics_destroy_vehicle(&physics, i);
                    }
                }
                physics.vehicle_count = 0;

                // Recreate all vehicles with their correct type configs
                for (int i = 0; i < old_count; i++) {
                    int type_idx = old_type_map[i];
                    VehicleJSON* vconfig = &g_vehicle_configs[type_idx];
                    VehicleConfig vehicle_cfg = config_vehicle_to_physics(vconfig);

                    int phys_id = physics_create_vehicle(&physics, spawn_positions[i], spawn_rotations[i], &vehicle_cfg);
                    g_vehicle_type_map[phys_id] = type_idx;
                }
                printf("All %d vehicles recreated with reloaded configs\n", old_count);

                // Unpause physics so vehicles can settle
                if (physics_is_paused(&physics)) {
                    physics_unpause(&physics);
                }
            } else {
                fprintf(stderr, "RELOAD FAILED: Some configs have errors - keeping current vehicles\n");
            }
        }

        // Reload scene config with L
        if (input.keys_pressed[KEY_L]) {
            printf("Reloading scene config...\n");
            config_load_scene("../../assets/config/scenes/showdown.json", &scene_config);
            printf("Scene config reloaded (arena/obstacles - restart for full effect)\n");
        }

        // Reload scripts with F5 (hot reload for development)
        if (input.keys_pressed[KEY_F5]) {
            printf("Reloading scripts...\n");
            int count = reflex_reload_all_scripts(script_engine);
            if (count >= 0) {
                printf("Scripts reloaded: %d vehicle scripts\n", count);
            } else {
                printf("Script reload failed!\n");
            }
        }

        // Start acceleration test with T (requires selected vehicle)
        if (input.keys_pressed[KEY_T]) {
            Entity* sel = entity_manager_get_selected(&entities);
            if (sel && sel->id < MAX_ENTITIES && entity_to_physics[sel->id] >= 0) {
                physics_vehicle_start_accel_test(&physics, entity_to_physics[sel->id]);
            } else {
                printf("Select a vehicle first (T)\n");
            }
        }

        // Pass input to scripts (X, Y handled in Lua now)
        {
            Entity* sel = entity_manager_get_selected(&entities);
            int selected_vehicle_id = -1;
            if (sel && sel->id < MAX_ENTITIES && entity_to_physics[sel->id] >= 0) {
                selected_vehicle_id = entity_to_physics[sel->id];
            }
            reflex_on_input(script_engine, input.keys_pressed, 512, selected_vehicle_id);
        }

        // Cruise control: V = toggle cruise (hold current speed or cancel)
        if (input.keys_pressed[KEY_V]) {
            Entity* sel = entity_manager_get_selected(&entities);
            if (sel && sel->id < MAX_ENTITIES && entity_to_physics[sel->id] >= 0) {
                int phys_id = entity_to_physics[sel->id];
                if (physics_vehicle_cruise_active(&physics, phys_id)) {
                    physics_vehicle_cruise_cancel(&physics, phys_id);
                } else {
                    physics_vehicle_cruise_hold(&physics, phys_id);
                }
            } else {
                printf("Select a vehicle first (V)\n");
            }
        }

        // Cruise control: ] = snap UP to next 10 mph
        if (input.keys_pressed[KEY_RIGHTBRACKET]) {
            Entity* sel = entity_manager_get_selected(&entities);
            if (sel && sel->id < MAX_ENTITIES && entity_to_physics[sel->id] >= 0) {
                physics_vehicle_cruise_snap_up(&physics, entity_to_physics[sel->id]);
            } else {
                printf("Select a vehicle first (])\n");
            }
        }

        // Cruise control: [ = snap DOWN to previous 10 mph
        if (input.keys_pressed[KEY_LEFTBRACKET]) {
            Entity* sel = entity_manager_get_selected(&entities);
            if (sel && sel->id < MAX_ENTITIES && entity_to_physics[sel->id] >= 0) {
                physics_vehicle_cruise_snap_down(&physics, entity_to_physics[sel->id]);
            } else {
                printf("Select a vehicle first ([)\n");
            }
        }

        // ========== PHYSICS PAUSE ==========
        // TAB = toggle physics pause (TURN MODE DISABLED)
#if TURN_MODE_ENABLED
        // When pausing: snapshot speed and calculate active phases for turn declaration
        if (input.keys_pressed[KEY_TAB]) {
            if (physics_is_paused(&physics)) {
                physics_unpause(&physics);
            } else {
                physics_pause(&physics);
#else
        // Turn mode disabled - TAB does nothing
        if (false && input.keys_pressed[KEY_TAB]) {
            if (false) {
            } else {
#endif

                // Snapshot speed and calculate active phases when pausing
                Entity* sel_for_pause = entity_manager_get_selected(&entities);
                int pause_phys_id = (sel_for_pause && sel_for_pause->id < MAX_ENTITIES)
                                    ? entity_to_physics[sel_for_pause->id] : -1;

                if (pause_phys_id >= 0) {
                    // Get current speed in m/s and convert to mph
                    float vel_ms = 0.0f;
                    physics_vehicle_get_velocity(&physics, pause_phys_id, &vel_ms);
                    planning.snapshot_speed = (int)(fabsf(vel_ms) * 2.237f);  // m/s to mph

                    // Reset maneuver for new turn
                    planning.maneuver = MANEUVER_NONE;
                    planning.direction = MANEUVER_LEFT;
                    planning.bend_angle = 0;

                    printf("[Turn] Paused at %d mph\n", planning.snapshot_speed);
                }
            }
        }

        // ========== MANEUVER CANCEL KEY ==========
        // Key 0 = cancel maneuver (kept for emergency abort)
        // Direct maneuver keys 1-9 removed - use GUI declaration instead
        if (physics_is_paused(&physics) && input.keys_pressed[KEY_0]) {
            Entity* sel_for_cancel = entity_manager_get_selected(&entities);
            int cancel_phys_id = (sel_for_cancel && sel_for_cancel->id < MAX_ENTITIES)
                              ? entity_to_physics[sel_for_cancel->id] : -1;
            if (cancel_phys_id >= 0) {
                physics_vehicle_cancel_maneuver(&physics, cancel_phys_id);
            }
        }

        // Left click handling (UI buttons first, then 3D picking)
        if (input.mouse_pressed[MOUSE_LEFT] && !input.mouse_captured) {
            float mx = (float)input.mouse_x;
            float my = (float)input.mouse_y;
            bool ui_clicked = false;

            // Speed button rects (must match rendering positions)
            UIRect brake_btn = ui_rect(platform.width - 305, 100, 80, 50);
            UIRect hold_btn = ui_rect(platform.width - 210, 100, 80, 50);
            UIRect accel_btn = ui_rect(platform.width - 115, 100, 80, 50);

            // Check speed button clicks - all three choices available at any speed
            if (point_in_rect(mx, my, brake_btn)) {
                planning.speed_choice = SPEED_BRAKE;
                ui_clicked = true;
            } else if (point_in_rect(mx, my, hold_btn)) {
                planning.speed_choice = SPEED_HOLD;
                ui_clicked = true;
            } else if (point_in_rect(mx, my, accel_btn)) {
                planning.speed_choice = SPEED_ACCEL;
                ui_clicked = true;
            }

            // At 0 mph, maneuvers are limited to STRAIGHT
            bool at_stop = (planning.snapshot_speed < 5);

            // Maneuver selector click handling (only when paused AND not at 0 mph)
            // At 0 mph, only STRAIGHT is allowed (no maneuver selection)
            bool can_edit_maneuver = physics_is_paused(&physics) && !at_stop;

            if (can_edit_maneuver && !ui_clicked) {
                // Row 1: Maneuver type buttons (5 buttons: STR, DFT, STP, BND, SWV)
                // Y=200 matches rendering in draw section
                ManeuverType maneuver_values[] = {MANEUVER_NONE, MANEUVER_DRIFT, MANEUVER_STEEP_DRIFT, MANEUVER_BEND, MANEUVER_SWERVE};
                for (int m = 0; m < 5; m++) {
                    UIRect btn = ui_rect(platform.width - 305 + m * 60, 200, 56, 35);
                    if (point_in_rect(mx, my, btn)) {
                        planning.maneuver = maneuver_values[m];
                        // Reset direction to LEFT when changing maneuver type
                        planning.direction = MANEUVER_LEFT;
                        // Reset bend angle to 15 when selecting BEND or SWERVE
                        if (maneuver_values[m] == MANEUVER_BEND || maneuver_values[m] == MANEUVER_SWERVE) {
                            planning.bend_angle = 15;
                        }
                        ui_clicked = true;
                        break;
                    }
                }

                // Row 2: Direction buttons (LEFT / RIGHT)
                // Y=245 matches rendering in draw section
                ManeuverType sel_type = planning.maneuver;
                bool needs_direction = (sel_type == MANEUVER_DRIFT || sel_type == MANEUVER_STEEP_DRIFT || sel_type == MANEUVER_BEND || sel_type == MANEUVER_SWERVE);

                if (needs_direction && !ui_clicked) {
                    UIRect left_btn = ui_rect(platform.width - 305, 245, 145, 35);
                    UIRect right_btn = ui_rect(platform.width - 155, 245, 145, 35);

                    if (point_in_rect(mx, my, left_btn)) {
                        planning.direction = MANEUVER_LEFT;
                        ui_clicked = true;
                    } else if (point_in_rect(mx, my, right_btn)) {
                        planning.direction = MANEUVER_RIGHT;
                        ui_clicked = true;
                    }
                }

                // Row 3: Bend angle buttons (15, 30, 45) - for BEND and SWERVE
                // Y=290 matches rendering in draw section
                if ((sel_type == MANEUVER_BEND || sel_type == MANEUVER_SWERVE) && !ui_clicked) {
                    int angles[] = {15, 30, 45};
                    for (int a = 0; a < 3; a++) {
                        UIRect btn = ui_rect(platform.width - 305 + a * 100, 290, 95, 35);
                        if (point_in_rect(mx, my, btn)) {
                            planning.bend_angle = angles[a];
                            ui_clicked = true;
                            break;
                        }
                    }
                }
            }

            // Check Execute button click (moved down to Y=465)
            UIRect execute_btn = ui_rect(platform.width - 315, 465, 300, 50);
            if (point_in_rect(mx, my, execute_btn) && physics_is_paused(&physics)) {
                Entity* selected = entity_manager_get_selected(&entities);
                int exec_phys_id = (selected && selected->id < MAX_ENTITIES)
                                   ? entity_to_physics[selected->id] : -1;

                if (exec_phys_id < 0) {
                    printf("[Turn] No vehicle selected!\n");
                    ui_clicked = true;
                } else {
                    // At 0 mph, only STRAIGHT is allowed (no maneuvers)
                    bool starting_from_stop = (planning.snapshot_speed < 5);

                    // Get the maneuver for this turn (default to straight if none selected or at stop)
                    ManeuverType maneuver_type = planning.maneuver;
                    ManeuverDirection maneuver_dir = planning.direction;
                    int bend_angle = planning.bend_angle;

                    if (starting_from_stop || maneuver_type == MANEUVER_NONE) {
                        maneuver_type = MANEUVER_STRAIGHT;
                    }

                    // Check if cruise was active before executing
                    bool cruise_was_active = physics_vehicle_cruise_active(&physics, exec_phys_id);

                    // Calculate target speed
                    int target_speed_mph = calculate_next_speed(planning.snapshot_speed, planning.speed_choice);
                    if (target_speed_mph < 0) target_speed_mph = 0;

                    // Send execute_maneuver event to Lua
                    const char* maneuver_name = maneuver_to_script_name(maneuver_type, bend_angle);
                    const char* direction_str = (maneuver_dir == MANEUVER_LEFT) ? "left" : "right";

                    ScriptEventData event_data = reflex_event_data_create();
                    reflex_event_data_add_string(&event_data, "type", maneuver_name);
                    reflex_event_data_add_string(&event_data, "direction", direction_str);
                    reflex_event_data_add_float(&event_data, "duration", 1.0f);

                    // Speed change info
                    const char* speed_change_names[] = {"decelerate", "maintain", "accelerate"};
                    reflex_event_data_add_string(&event_data, "speed_change", speed_change_names[planning.speed_choice]);
                    reflex_event_data_add_float(&event_data, "target_speed", (float)target_speed_mph);

                    reflex_send_event(script_engine, exec_phys_id, "execute_maneuver", &event_data);

                    // Apply speed change via cruise control
                    bool should_set_cruise = (planning.speed_choice != SPEED_HOLD) || cruise_was_active;
                    if (should_set_cruise) {
                        float target_speed_ms = target_speed_mph / 2.237f;  // mph to m/s
                        physics_vehicle_cruise_set(&physics, exec_phys_id, target_speed_ms);
                    }

                    physics_unpause(&physics);

                    // Track that a turn is executing
                    planning.turn_executing = true;
                    planning.turn_vehicle_id = exec_phys_id;

                    // Log what we're executing
                    const char* speed_names[] = {"BRAKE", "HOLD", "ACCEL"};
                    printf("[Turn] Executing %s %s at %d mph -> %s to %d mph\n",
                           direction_str, maneuver_name,
                           planning.snapshot_speed, speed_names[planning.speed_choice], target_speed_mph);

                    // Update snapshot for next turn
                    planning.snapshot_speed = target_speed_mph;

                    // Reset maneuver for next turn
                    planning.maneuver = MANEUVER_NONE;
                    planning.direction = MANEUVER_LEFT;
                    planning.bend_angle = 0;

                    ui_clicked = true;
                }
            }

            // If no UI was clicked, do 3D picking
            if (!ui_clicked) {
                Vec3 ray_origin, ray_dir;
                camera_screen_to_ray(&camera, input.mouse_x, input.mouse_y,
                                     platform.width, platform.height,
                                     &ray_origin, &ray_dir);

                int hit_id = entity_manager_pick(&entities, ray_origin, ray_dir);
                if (hit_id >= 0) {
                    entity_manager_select(&entities, hit_id);
                    Entity* selected = entity_manager_get_selected(&entities);
                    if (selected) {
                        printf("Selected: %s team vehicle at (%.1f, %.1f)\n",
                               selected->team == TEAM_RED ? "Red" :
                               selected->team == TEAM_BLUE ? "Blue" :
                               selected->team == TEAM_YELLOW ? "Yellow" : "Green",
                               selected->position.x, selected->position.z);
                    }
                } else {
                    entity_manager_deselect_all(&entities);
                }
            }
        }

        // Update camera
        if (!chase_camera) {
            // Normal free camera mode
            camera_update(&camera, &input, dt);
        }

        // Apply vehicle controls only in freestyle mode (not paused)
        if (!physics_is_paused(&physics)) {
            Entity* selected = entity_manager_get_selected(&entities);
            if (selected && selected->id < MAX_ENTITIES) {
                int phys_id = entity_to_physics[selected->id];
                if (phys_id >= 0) {
                    // Get input (arrow keys for car control, space for brake)
                    float throttle = 0.0f;
                    float reverse = 0.0f;
                    float brake = 0.0f;

                    // Steering modes: discrete (tabletop style) or gradual (analog style)
                    const float steering_rate = 2.5f;  // Full lock in ~0.4 seconds (gradual)

                    // M key toggles steering mode
                    if (input.keys_pressed[KEY_M]) {
                        discrete_steering = !discrete_steering;
                        // Reset steering when switching modes
                        steering_level = 0;
                        current_steering = 0.0f;
                        printf("Steering mode: %s\n", discrete_steering ? "DISCRETE (tap arrows)" : "GRADUAL (hold arrows)");
                    }

                    if (discrete_steering) {
                        // Discrete mode: tabletop D ratings (D1=15°, D2=30°, D3=45°)
                        // Level 0 = straight, ±1 = D1, ±2 = D2, ±3 = D3 (full lock)
                        // Steering holds position until manually changed
                        if (input.keys_pressed[KEY_LEFT]) {
                            steering_level--;
                            if (steering_level < -3) steering_level = -3;
                        }
                        if (input.keys_pressed[KEY_RIGHT]) {
                            steering_level++;
                            if (steering_level > 3) steering_level = 3;
                        }
                        // Both arrows = center immediately
                        if (input.keys_pressed[KEY_LEFT] && input.keys_pressed[KEY_RIGHT]) {
                            steering_level = 0;
                        }
                        // Map levels to steering: D1=0.333, D2=0.667, D3=1.0
                        current_steering = steering_level * 0.333f;
                    } else {
                        // Gradual mode: hold arrows to steer, auto-center when released
                        if (input.keys[KEY_LEFT]) {
                            current_steering -= steering_rate * dt;
                            if (current_steering < -1.0f) current_steering = -1.0f;
                        } else if (input.keys[KEY_RIGHT]) {
                            current_steering += steering_rate * dt;
                            if (current_steering > 1.0f) current_steering = 1.0f;
                        } else {
                            // Auto-center when no arrows held
                            if (current_steering > 0.0f) {
                                current_steering -= steering_rate * dt;
                                if (current_steering < 0.0f) current_steering = 0.0f;
                            } else if (current_steering < 0.0f) {
                                current_steering += steering_rate * dt;
                                if (current_steering > 0.0f) current_steering = 0.0f;
                            }
                        }
                    }

                    // Manual throttle mode (keys 4-0 set fixed throttle levels)
                    if (manual_throttle_level >= 0) {
                        // Manual throttle active - apply fixed level
                        throttle = manual_throttle_values[manual_throttle_level];
                        // UP key clears manual throttle (return to full)
                        if (input.keys_pressed[KEY_UP]) {
                            manual_throttle_level = -1;
                            printf("Manual throttle: OFF (full throttle)\n");
                        }
                    } else if (input.keys[KEY_UP]) {
                        // Normal throttle - Shift = half throttle for less wheelspin
                        throttle = (input.keys[KEY_LSHIFT] || input.keys[KEY_RSHIFT]) ? 0.5f : 1.0f;
                    }
                    if (input.keys[KEY_DOWN]) {
                        reverse = (input.keys[KEY_LSHIFT] || input.keys[KEY_RSHIFT]) ? 0.5f : 1.0f;
                    }
                    if (input.keys[KEY_SPACE]) brake = 1.0f;

                    // Apply controls to physics vehicle
                    physics_vehicle_set_throttle(&physics, phys_id, throttle);
                    physics_vehicle_set_reverse(&physics, phys_id, reverse);
                    physics_vehicle_set_brake(&physics, phys_id, brake);
                    physics_vehicle_set_steering(&physics, phys_id, current_steering);
                }
            }
        }

        // Update reflex scripts (ABS, traction control, AI)
        // Scripts run before physics so they can modify controls
        if (script_engine) {
            for (int i = 0; i < physics.vehicle_count; i++) {
                if (physics.vehicles[i].active) {
                    reflex_update_vehicle(script_engine, &physics, i, dt);
                }
            }
        }

        // Always step physics simulation (gravity, collisions, etc. always active)
        physics_step(&physics, dt);

        // Check if GUI-triggered turn has completed and re-pause physics
        if (planning.turn_executing && planning.turn_vehicle_id >= 0) {
            // Poll Lua to check if turn is still active
            bool turn_still_active = reflex_is_turn_active(script_engine, planning.turn_vehicle_id);

            if (!turn_still_active) {
                // Turn completed - pause physics for next turn declaration
                physics_pause(&physics);
                planning.turn_executing = false;

                // Get current speed for next turn snapshot
                float vel_ms = 0.0f;
                physics_vehicle_get_velocity(&physics, planning.turn_vehicle_id, &vel_ms);
                planning.snapshot_speed = (int)(fabsf(vel_ms) * 2.237f);

                // Reset maneuver for next turn
                planning.maneuver = MANEUVER_NONE;
                planning.direction = MANEUVER_LEFT;
                planning.bend_angle = 0;

                printf("[Turn] Turn complete - paused at %d mph\n", planning.snapshot_speed);
                planning.turn_vehicle_id = -1;
            }
        }

        // Update scripted turn timer
        if (scripted_turn.active) {
            scripted_turn.elapsed += dt;

            // Check for turn completion
            if (scripted_turn.elapsed >= scripted_turn.duration) {
                // End the turn and get result
                ManeuverResult result = reflex_end_turn(script_engine, scripted_turn.vehicle_id);

                printf("[Turn] Completed: %s %s\n", scripted_turn.maneuver, scripted_turn.direction);
                printf("[Turn] Result: %s (success=%s)\n",
                       result.level, result.success ? "yes" : "no");
                printf("[Turn] Heading: target=%.1f, achieved=%.1f, error=%.1f deg\n",
                       result.heading_target, result.heading_achieved, result.heading_error);

                // Reset turn state
                scripted_turn.active = false;
                scripted_turn.vehicle_id = -1;
                scripted_turn.elapsed = 0.0f;
            }
        }

        // Sync physics state back to entities
        for (int i = 0; i < entities.count; i++) {
            Entity* e = &entities.entities[i];
            if (!e->active || e->type != ENTITY_VEHICLE) continue;

            int phys_id = entity_to_physics[e->id];
            if (phys_id >= 0) {
                Vec3 pos;
                float rot_y;
                physics_vehicle_get_position(&physics, phys_id, &pos);
                physics_vehicle_get_rotation(&physics, phys_id, &rot_y);
                e->position = pos;
                e->rotation_y = rot_y;

                // Store velocity for display
                // During kinematic maneuver, interpolate between start and target speed
                float speed_ms;
                if (physics_vehicle_maneuver_active(&physics, phys_id)) {
                    const ManeuverAutopilot* ap = physics_vehicle_get_autopilot(&physics, phys_id);
                    if (ap) {
                        // Interpolate speed during turn (ACCEL/BRAKE changes speed during turn)
                        float start = ap->start_speed_ms;
                        float target = ap->target_speed_ms > 0.0f ? ap->target_speed_ms : start;
                        speed_ms = start + (target - start) * ap->progress;
                    } else {
                        speed_ms = 0.0f;
                    }
                } else {
                    physics_vehicle_get_velocity(&physics, phys_id, &speed_ms);
                }
                if (e->id < MAX_ENTITIES) {
                    car_physics[e->id].velocity = speed_ms;
                }
            }
        }

        // Chase camera: spherical orbit around selected vehicle
        if (chase_camera) {
            Entity* sel = entity_manager_get_selected(&entities);
            if (sel && sel->id < MAX_ENTITIES) {
                int phys_id = entity_to_physics[sel->id];
                if (phys_id >= 0) {
                    // RMB drag controls spherical orbit (azimuth and elevation)
                    // Skip first frame after capture (SDL sends large delta on capture)
                    if (input.mouse_captured) {
                        bool has_movement = (input.mouse_dx != 0 || input.mouse_dy != 0);

                        if (input.mouse_capture_just_started) {
                            // Wait for first movement, then skip it
                            if (has_movement) {
                                input.mouse_capture_just_started = false;
                                if (g_verbose) {
                                    printf("Chase drag: SKIPPED capture start dx=%d dy=%d\n",
                                           input.mouse_dx, input.mouse_dy);
                                }
                            }
                        } else if (has_movement) {
                            float sensitivity = 0.005f;
                            chase_azimuth += input.mouse_dx * sensitivity;
                            chase_elevation -= input.mouse_dy * sensitivity;
                            // Clamp elevation: 10-70 degrees (0.175 to 1.22 radians)
                            if (chase_elevation < 0.175f) chase_elevation = 0.175f;
                            if (chase_elevation > 1.22f) chase_elevation = 1.22f;
                        }
                    }

                    // Scroll wheel adjusts chase distance (zoom)
                    if (input.scroll_y != 0) {
                        chase_distance -= input.scroll_y * 2.0f;
                        if (chase_distance < 5.0f) chase_distance = 5.0f;
                        if (chase_distance > 100.0f) chase_distance = 100.0f;
                    }

                    // Get car position directly (no smoothing needed with orbit control)
                    Vec3 car_pos;
                    physics_vehicle_get_position(&physics, phys_id, &car_pos);

                    // Spherical coordinates: camera orbits on a sphere around the car
                    // elevation=0 is level, elevation=PI/2 is directly overhead
                    float cos_elev = cosf(chase_elevation);
                    float sin_elev = sinf(chase_elevation);
                    camera.position.x = car_pos.x + cos_elev * sinf(chase_azimuth) * chase_distance;
                    camera.position.y = car_pos.y + sin_elev * chase_distance;
                    camera.position.z = car_pos.z + cos_elev * cosf(chase_azimuth) * chase_distance;

                    // Look at car - compute yaw/pitch to point at target
                    // camera_forward: x = sin(yaw)*cos(pitch), z = -cos(yaw)*cos(pitch)
                    // So for direction (dx, dy, dz): yaw = atan2(dx, -dz), pitch = atan2(dy, horiz)
                    Vec3 to_target = vec3_sub(car_pos, camera.position);
                    camera.yaw = atan2f(to_target.x, -to_target.z);
                    float horiz_dist = sqrtf(to_target.x * to_target.x + to_target.z * to_target.z);
                    camera.pitch = atan2f(to_target.y, horiz_dist);
                }
            } else {
                // No vehicle selected, disable chase camera
                chase_camera = false;
                printf("Chase camera OFF (no vehicle selected)\n");
            }
        }

        // Render
        glViewport(0, 0, platform.width, platform.height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set up matrices
        float aspect = (float)platform.width / (float)platform.height;
        Mat4 projection = camera_projection_matrix(&camera, aspect);
        Mat4 view = camera_view_matrix(&camera);

        // Draw floor with procedural grid (modern OpenGL shader)
        floor_render(&arena_floor, &view, &projection, camera.position);

        // Draw walls, obstacles, ramps, and cars with lighting
        box_renderer_begin(&box_renderer, &view, &projection, light_dir);
        draw_arena_walls(&box_renderer, &scene_config.arena);
        draw_obstacles(&box_renderer, &scene_config);
        draw_ramps(&box_renderer, &scene_config);
        if (show_cars) {
            // Get selected physics vehicle id for highlighting
            Entity* sel = entity_manager_get_selected(&entities);
            int selected_phys_id = (sel && sel->id < MAX_ENTITIES) ? entity_to_physics[sel->id] : -1;

            // Build team mapping for physics vehicles (physics_id -> team)
            Team team_for_vehicle[MAX_PHYSICS_VEHICLES];
            for (int i = 0; i < MAX_PHYSICS_VEHICLES; i++) {
                team_for_vehicle[i] = TEAM_RED;  // Default
            }
            for (int i = 0; i < entities.count; i++) {
                Entity* e = &entities.entities[i];
                if (e->active && e->id < MAX_ENTITIES) {
                    int phys_id = entity_to_physics[e->id];
                    if (phys_id >= 0 && phys_id < MAX_PHYSICS_VEHICLES) {
                        team_for_vehicle[phys_id] = e->team;
                    }
                }
            }

            // Render physics vehicles as solid primitives (physics-first approach)
            draw_physics_vehicles(&box_renderer, &physics, selected_phys_id, team_for_vehicle);

            // Draw wheel meshes at physics wheel positions (if any mesh loaded)
            if (g_vehicle_type_count > 0) {
                draw_vehicle_wheels_mesh(&box_renderer, &physics);
            }
        }
        box_renderer_end(&box_renderer);

        // All line rendering in one batch (wheels, ghost path, debug)
        if (has_lines) {
            line_renderer_begin(&line_renderer, &view, &projection);

            // Draw wheels with rotating spokes (only if no meshes loaded - fallback)
            if (show_cars && g_vehicle_type_count == 0) {
                draw_vehicle_wheels(&line_renderer, &physics);
            }

            // Physics debug visualization (press P to toggle)
            if (show_physics_debug) {
                physics_debug_draw(&physics, &line_renderer);
            }

            line_renderer_end(&line_renderer);
        }

        // Draw UI test panels
        ui_renderer_begin(&ui_renderer, platform.width, platform.height);

        // Right side panel (where controls will go)
        ui_draw_panel(&ui_renderer,
            ui_rect(platform.width - 320, 10, 310, platform.height - 20),
            UI_COLOR_PANEL, UI_COLOR_SELECTED, 2.0f, 8.0f);

        // Header bar
        ui_draw_panel(&ui_renderer,
            ui_rect(platform.width - 315, 15, 300, 40),
            UI_COLOR_BG_DARK, UI_COLOR_ACCENT, 1.0f, 4.0f);

        // Speed control section
        // In freestyle mode, show controls as disabled
        bool turn_mode = physics_is_paused(&physics);
        UIColor section_border = turn_mode ? ui_color(0.3f, 0.3f, 0.4f, 1.0f) : ui_color(0.2f, 0.2f, 0.25f, 1.0f);

        ui_draw_panel(&ui_renderer,
            ui_rect(platform.width - 315, 65, 300, 100),
            UI_COLOR_BG_DARK, section_border, 1.0f, 4.0f);

        // Three speed buttons (highlight selected, grey out in freestyle mode only)
        {
            float sel_border = 3.0f;
            float norm_border = 1.0f;
            bool disabled = !turn_mode;  // Disable all in freestyle mode

            // BRAKE button
            ui_draw_panel(&ui_renderer,
                ui_rect(platform.width - 305, 100, 80, 50),
                disabled ? UI_COLOR_DISABLED : UI_COLOR_DANGER, UI_COLOR_WHITE,
                (!disabled && planning.speed_choice == SPEED_BRAKE) ? sel_border : norm_border, 4.0f);
            // HOLD button
            ui_draw_panel(&ui_renderer,
                ui_rect(platform.width - 210, 100, 80, 50),
                disabled ? UI_COLOR_DISABLED : UI_COLOR_SELECTED, UI_COLOR_WHITE,
                (!disabled && planning.speed_choice == SPEED_HOLD) ? sel_border : norm_border, 4.0f);
            // ACCEL button
            ui_draw_panel(&ui_renderer,
                ui_rect(platform.width - 115, 100, 80, 50),
                disabled ? UI_COLOR_DISABLED : UI_COLOR_SAFE, UI_COLOR_WHITE,
                (!disabled && planning.speed_choice == SPEED_ACCEL) ? sel_border : norm_border, 4.0f);
        }

        // Maneuver section (single maneuver per turn)
        ui_draw_panel(&ui_renderer,
            ui_rect(platform.width - 315, 175, 300, 280),
            UI_COLOR_BG_DARK, section_border, 1.0f, 4.0f);

        // Maneuver selector (only when physics paused AND not at 0 mph)
        // At 0 mph, only STRAIGHT is allowed (starting from stop)
        bool at_stop_for_selector = (planning.snapshot_speed < 5);
        bool show_maneuver_selector = physics_is_paused(&physics) && !at_stop_for_selector;

        if (show_maneuver_selector) {
            // Row 1: Maneuver type buttons (5 buttons: STR, DFT, STP, BND, SWV)
            ManeuverType maneuver_values[] = {MANEUVER_NONE, MANEUVER_DRIFT, MANEUVER_STEEP_DRIFT, MANEUVER_BEND, MANEUVER_SWERVE};
            int current_maneuver = planning.maneuver;

            for (int m = 0; m < 5; m++) {
                bool is_sel = (current_maneuver == maneuver_values[m]);
                UIColor btn_color = is_sel ? UI_COLOR_SELECTED : ui_color(0.2f, 0.2f, 0.3f, 1.0f);
                UIColor btn_border = is_sel ? UI_COLOR_WHITE : ui_color(0.4f, 0.4f, 0.5f, 1.0f);
                ui_draw_panel(&ui_renderer,
                    ui_rect(platform.width - 305 + m * 60, 200, 56, 35),
                    btn_color, btn_border, is_sel ? 2.0f : 1.0f, 4.0f);
            }

            // Row 2: Direction buttons (only if maneuver needs direction)
            ManeuverType sel_type = (ManeuverType)current_maneuver;
            bool needs_direction = (sel_type == MANEUVER_DRIFT || sel_type == MANEUVER_STEEP_DRIFT || sel_type == MANEUVER_BEND || sel_type == MANEUVER_SWERVE);

            if (needs_direction) {
                ManeuverDirection current_dir = planning.direction;
                bool left_sel = (current_dir == MANEUVER_LEFT);
                bool right_sel = (current_dir == MANEUVER_RIGHT);

                ui_draw_panel(&ui_renderer,
                    ui_rect(platform.width - 305, 245, 145, 35),
                    left_sel ? UI_COLOR_CAUTION : ui_color(0.2f, 0.2f, 0.3f, 1.0f),
                    left_sel ? UI_COLOR_WHITE : ui_color(0.4f, 0.4f, 0.5f, 1.0f),
                    left_sel ? 2.0f : 1.0f, 4.0f);
                ui_draw_panel(&ui_renderer,
                    ui_rect(platform.width - 155, 245, 145, 35),
                    right_sel ? UI_COLOR_CAUTION : ui_color(0.2f, 0.2f, 0.3f, 1.0f),
                    right_sel ? UI_COLOR_WHITE : ui_color(0.4f, 0.4f, 0.5f, 1.0f),
                    right_sel ? 2.0f : 1.0f, 4.0f);
            }

            // Row 3: Bend angle buttons (for BEND or SWERVE)
            if (sel_type == MANEUVER_BEND || sel_type == MANEUVER_SWERVE) {
                int angles[] = {15, 30, 45};
                int current_angle = planning.bend_angle;

                for (int a = 0; a < 3; a++) {
                    bool angle_sel = (current_angle == angles[a]);
                    ui_draw_panel(&ui_renderer,
                        ui_rect(platform.width - 305 + a * 100, 290, 95, 35),
                        angle_sel ? UI_COLOR_SAFE : ui_color(0.2f, 0.2f, 0.3f, 1.0f),
                        angle_sel ? UI_COLOR_WHITE : ui_color(0.4f, 0.4f, 0.5f, 1.0f),
                        angle_sel ? 2.0f : 1.0f, 4.0f);
                }
            }
        }

        // Execute button (moved down to accommodate maneuver selector)
        // Disabled in freestyle mode
        ui_draw_panel(&ui_renderer,
            ui_rect(platform.width - 315, 465, 300, 50),
            turn_mode ? UI_COLOR_ACCENT : UI_COLOR_DISABLED, UI_COLOR_WHITE, 2.0f, 4.0f);

        // Bottom status bar (two rows) - full width
        ui_draw_panel(&ui_renderer,
            ui_rect(10, platform.height - 70, platform.width - 340, 60),
            UI_COLOR_PANEL, UI_COLOR_SELECTED, 1.0f, 4.0f);

        // ===== TOP-LEFT HUD OVERLAY (gauges and slip bars) =====
        // Get physics data for HUD
        Entity* hud_sel = entity_manager_get_selected(&entities);
        int hud_phys_id = -1;
        float hud_throttle = 0.0f;
        float hud_brake = 0.0f;
        float hud_slip[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float hud_speed_mph = 0.0f;
        float hud_rpm = 0.0f;
        float hud_redline = 6000.0f;
        int hud_gear = 0;

        if (hud_sel && hud_sel->id < MAX_ENTITIES) {
            hud_phys_id = entity_to_physics[hud_sel->id];
            if (hud_phys_id >= 0 && physics.vehicles[hud_phys_id].active) {
                hud_throttle = physics.vehicles[hud_phys_id].throttle;
                hud_brake = physics.vehicles[hud_phys_id].brake;

                // Get wheel slip
                WheelState ws[4];
                physics_vehicle_get_wheel_states(&physics, hud_phys_id, ws);
                for (int w = 0; w < 4; w++) {
                    hud_slip[w] = fabsf(ws[w].longitudinal_slip);
                    if (hud_slip[w] > 1.0f) hud_slip[w] = 1.0f;
                }

                // Get speed
                hud_speed_mph = fabsf(car_physics[hud_sel->id].velocity) * 2.237f;

                // Get RPM and gear
                int raw_gear = 0;
                bool is_matchbox = false;
                physics_vehicle_get_drivetrain_info(&physics, hud_phys_id, &hud_gear, &hud_rpm, &raw_gear, &is_matchbox);
            }
        }

        // HUD position (top-left, offset from edge)
        float hud_x = 15.0f;
        float hud_y = 15.0f;
        float label_w = 35.0f;   // Width for labels (SPD, THR, BRK)
        float gauge_w = 160.0f;  // Bar width
        float gauge_h = 14.0f;   // Bar height
        float row_h = 20.0f;     // Row spacing

        // Semi-transparent background panel (taller to fit all elements)
        ui_draw_panel(&ui_renderer,
            ui_rect(hud_x - 5, hud_y - 5, label_w + gauge_w + 50, 150),
            ui_color(0.05f, 0.08f, 0.15f, 0.85f), ui_color(0.2f, 0.3f, 0.4f, 0.8f), 1.0f, 6.0f);

        float bar_x = hud_x + label_w;  // Bars start after labels

        // Row 0: Speed gauge (0-120 mph scale)
        float row0_y = hud_y;
        float speed_pct = hud_speed_mph / 120.0f;
        if (speed_pct > 1.0f) speed_pct = 1.0f;
        ui_draw_rect(&ui_renderer, ui_rect(bar_x, row0_y, gauge_w, gauge_h), ui_color(0.12f, 0.12f, 0.18f, 1.0f));
        if (speed_pct > 0.01f) {
            ui_draw_rect(&ui_renderer, ui_rect(bar_x, row0_y, gauge_w * speed_pct, gauge_h), UI_COLOR_SELECTED);
        }

        // Row 1: RPM gauge (0-redline scale, red zone at 85%+)
        float row1_y = row0_y + row_h;
        float rpm_pct = hud_rpm / hud_redline;
        if (rpm_pct > 1.0f) rpm_pct = 1.0f;
        ui_draw_rect(&ui_renderer, ui_rect(bar_x, row1_y, gauge_w, gauge_h), ui_color(0.12f, 0.12f, 0.18f, 1.0f));
        ui_draw_rect(&ui_renderer, ui_rect(bar_x + gauge_w * 0.85f, row1_y, gauge_w * 0.15f, gauge_h), ui_color(0.3f, 0.1f, 0.1f, 1.0f));
        if (rpm_pct > 0.01f) {
            UIColor rpm_color = rpm_pct > 0.85f ? UI_COLOR_DANGER : UI_COLOR_SAFE;
            ui_draw_rect(&ui_renderer, ui_rect(bar_x, row1_y, gauge_w * rpm_pct, gauge_h), rpm_color);
        }

        // Row 2: Throttle bar
        float row2_y = row1_y + row_h;
        ui_draw_rect(&ui_renderer, ui_rect(bar_x, row2_y, gauge_w, gauge_h), ui_color(0.12f, 0.12f, 0.18f, 1.0f));
        if (hud_throttle > 0.01f) {
            ui_draw_rect(&ui_renderer, ui_rect(bar_x, row2_y, gauge_w * hud_throttle, gauge_h), UI_COLOR_SAFE);
        }

        // Row 3: Brake bar
        float row3_y = row2_y + row_h;
        ui_draw_rect(&ui_renderer, ui_rect(bar_x, row3_y, gauge_w, gauge_h), ui_color(0.12f, 0.12f, 0.18f, 1.0f));
        if (hud_brake > 0.01f) {
            ui_draw_rect(&ui_renderer, ui_rect(bar_x, row3_y, gauge_w * hud_brake, gauge_h), UI_COLOR_DANGER);
        }

        // Row 4: Wheel slip bars (4 horizontal bars)
        float row4_y = row3_y + row_h + 4;
        float slip_bar_w = 45.0f;
        float slip_bar_h = 20.0f;
        float slip_gap = 6.0f;

        for (int w = 0; w < 4; w++) {
            float wx = hud_x + w * (slip_bar_w + slip_gap);
            float slip = hud_slip[w];

            // Background
            ui_draw_rect(&ui_renderer, ui_rect(wx, row4_y, slip_bar_w, slip_bar_h), ui_color(0.12f, 0.12f, 0.18f, 1.0f));

            // Color based on slip amount
            UIColor slip_color;
            if (slip > 0.15f) {
                slip_color = UI_COLOR_DANGER;
            } else if (slip > 0.05f) {
                slip_color = UI_COLOR_CAUTION;
            } else {
                slip_color = UI_COLOR_SAFE;
            }

            // Fill (horizontal bar showing slip %)
            if (slip > 0.01f) {
                float fill = slip;
                if (fill > 1.0f) fill = 1.0f;
                ui_draw_rect(&ui_renderer, ui_rect(wx, row4_y, slip_bar_w * fill, slip_bar_h), slip_color);
            }
        }

        ui_renderer_end(&ui_renderer);

        // Draw text labels
        if (has_text) {
            text_renderer_begin(&text_renderer, platform.width, platform.height);

            // Header text (changes based on mode)
            const char* header_text = !physics_is_paused(&physics) ? "FREESTYLE MODE" : "TURN PLANNING";
            text_draw_centered(&text_renderer, header_text,
                ui_rect(platform.width - 315, 15, 300, 40), UI_COLOR_WHITE);

            // Speed section label with current speed
            {
                char speed_text[32];
                snprintf(speed_text, sizeof(speed_text), "Current: %d mph", planning.current_speed);
                text_draw(&text_renderer, "SPEED", platform.width - 305, 70, UI_COLOR_WHITE);
                text_draw(&text_renderer, speed_text, platform.width - 200, 70, UI_COLOR_DISABLED);
            }

            // Speed button labels
            text_draw_centered(&text_renderer, "BRAKE",
                ui_rect(platform.width - 305, 100, 80, 50), UI_COLOR_WHITE);
            text_draw_centered(&text_renderer, "HOLD",
                ui_rect(platform.width - 210, 100, 80, 50), UI_COLOR_WHITE);
            text_draw_centered(&text_renderer, "ACCEL",
                ui_rect(platform.width - 115, 100, 80, 50), UI_COLOR_WHITE);

            // Maneuver section label with snapshot speed
            {
                char maneuver_label[64];
                if (physics_is_paused(&physics) && planning.snapshot_speed > 0) {
                    snprintf(maneuver_label, sizeof(maneuver_label), "MANEUVER (%d mph)", planning.snapshot_speed);
                } else {
                    snprintf(maneuver_label, sizeof(maneuver_label), "MANEUVER");
                }
                text_draw(&text_renderer, maneuver_label, platform.width - 305, 180, UI_COLOR_WHITE);
            }

            // Maneuver selector labels (only when paused, not at 0 mph)
            bool at_stop_labels = (planning.snapshot_speed < 5);
            bool show_selector_labels = physics_is_paused(&physics) && !at_stop_labels;

            if (show_selector_labels) {
                // Row 1: Maneuver type button labels (5 buttons)
                const char* type_labels[] = {"STR", "DFT", "STP", "BND", "SWV"};
                for (int m = 0; m < 5; m++) {
                    UIRect btn = ui_rect(platform.width - 305 + m * 60, 200, 56, 35);
                    text_draw_centered(&text_renderer, type_labels[m], btn, UI_COLOR_WHITE);
                }

                // Row 2: Direction labels (only if maneuver needs direction)
                ManeuverType sel_type = planning.maneuver;
                bool needs_direction = (sel_type == MANEUVER_DRIFT || sel_type == MANEUVER_STEEP_DRIFT || sel_type == MANEUVER_BEND || sel_type == MANEUVER_SWERVE);

                if (needs_direction) {
                    text_draw_centered(&text_renderer, "LEFT",
                        ui_rect(platform.width - 305, 245, 145, 35), UI_COLOR_WHITE);
                    text_draw_centered(&text_renderer, "RIGHT",
                        ui_rect(platform.width - 155, 245, 145, 35), UI_COLOR_WHITE);
                }

                // Row 3: Bend angle labels (if BEND or SWERVE selected)
                if (sel_type == MANEUVER_BEND || sel_type == MANEUVER_SWERVE) {
                    const char* angle_labels[] = {"15", "30", "45"};
                    for (int a = 0; a < 3; a++) {
                        UIRect btn = ui_rect(platform.width - 305 + a * 100, 290, 95, 35);
                        text_draw_centered(&text_renderer, angle_labels[a], btn, UI_COLOR_WHITE);
                    }
                }
            }

            // Execute button label (moved down to Y=465)
            text_draw_centered(&text_renderer, "EXECUTE TURN",
                ui_rect(platform.width - 315, 465, 300, 50), UI_COLOR_WHITE);

            // Top-left HUD labels (must match bar positions!)
            {
                float hud_x = 15.0f;
                float hud_y = 15.0f;
                float row_h = 20.0f;      // Must match bar code
                float gauge_w = 160.0f;   // Must match bar code

                // Row 0: Speed label and value
                float row0_y = hud_y;
                char speed_str[16];
                snprintf(speed_str, sizeof(speed_str), "%d", (int)hud_speed_mph);
                text_draw(&text_renderer, "SPD", (int)hud_x, (int)(row0_y), UI_COLOR_DISABLED);
                text_draw(&text_renderer, speed_str, (int)(hud_x + 200), (int)(row0_y), UI_COLOR_WHITE);

                // Row 1: Gear + RPM
                float row1_y = row0_y + row_h;
                char gear_str[8];
                if (hud_gear < 0) {
                    snprintf(gear_str, sizeof(gear_str), "R");
                } else if (hud_gear == 0) {
                    snprintf(gear_str, sizeof(gear_str), "N");
                } else {
                    snprintf(gear_str, sizeof(gear_str), "%d", hud_gear);
                }
                char rpm_label[16];
                snprintf(rpm_label, sizeof(rpm_label), "[%s]", gear_str);
                text_draw(&text_renderer, rpm_label, (int)hud_x, (int)(row1_y), UI_COLOR_ACCENT);
                char rpm_str[16];
                snprintf(rpm_str, sizeof(rpm_str), "%.0f", hud_rpm);
                text_draw(&text_renderer, rpm_str, (int)(hud_x + 200), (int)(row1_y), UI_COLOR_WHITE);

                // Row 2: THR label
                float row2_y = row1_y + row_h;
                text_draw(&text_renderer, "THR", (int)hud_x, (int)(row2_y), UI_COLOR_DISABLED);

                // Row 3: BRK label
                float row3_y = row2_y + row_h;
                text_draw(&text_renderer, "BRK", (int)hud_x, (int)(row3_y), UI_COLOR_DISABLED);

                // Row 4: Wheel slip labels
                float row4_y = row3_y + row_h + 4;  // Must match bar code
                float slip_bar_w = 45.0f;           // Must match bar code
                float slip_gap = 6.0f;              // Must match bar code
                const char* wheel_labels[] = {"FL", "FR", "RL", "RR"};
                for (int w = 0; w < 4; w++) {
                    float wx = hud_x + w * (slip_bar_w + slip_gap);
                    text_draw(&text_renderer, wheel_labels[w], (int)(wx + 12), (int)(row4_y + 3), UI_COLOR_DISABLED);
                }
            }

            // Status bar text - two rows with clear labels
            // Row 1: Mode, Vehicle, Speed, Target (common to both modes)
            // Row 2: Mode-specific details
            {
                const char* team_name = "No Vehicle";
                Entity* sel = entity_manager_get_selected(&entities);
                int phys_id = -1;
                if (sel) {
                    team_name = sel->team == TEAM_RED ? "Red Team" :
                                sel->team == TEAM_BLUE ? "Blue Team" : "Other";
                    if (sel->id < MAX_ENTITIES) {
                        phys_id = entity_to_physics[sel->id];
                    }
                }

                // Two-row layout with fixed column positions
                int row1_y = platform.height - 58;  // Top row
                int row2_y = platform.height - 38;  // Bottom row
                char col_buf[48];

                // Column positions (same for both modes)
                int col1 = 20;    // Mode
                int col2 = 120;   // Vehicle
                int col3 = 240;   // Speed
                int col4 = 400;   // Target/Action
                int col5 = 560;   // Handling

                // Get common data
                float vel = 0.0f;
                if (sel && sel->id < MAX_ENTITIES) {
                    vel = car_physics[sel->id].velocity;
                }
                int display_mph = (int)(fabsf(vel) * 2.237f);

                int hs = 0, hc = 0;
                if (phys_id >= 0) {
                    physics_vehicle_get_handling(&physics, phys_id, &hs, &hc);
                }

                // ===== ROW 1: Common info =====
                // Mode indicator
                const char* mode_label = physics_is_paused(&physics) ? "TURN MODE" : "FREE MODE";
                text_draw(&text_renderer, mode_label, col1, row1_y,
                          physics_is_paused(&physics) ? UI_COLOR_CAUTION : UI_COLOR_SAFE);

                // Vehicle
                text_draw(&text_renderer, team_name, col2, row1_y, UI_COLOR_WHITE);

                // Speed
                snprintf(col_buf, sizeof(col_buf), "Speed: %d mph", display_mph);
                text_draw(&text_renderer, col_buf, col3, row1_y, UI_COLOR_WHITE);

                // Target/Action (mode-dependent but same position)
                if (!physics_is_paused(&physics)) {
                    // Freestyle: show cruise target
                    bool cruise_on = phys_id >= 0 && physics_vehicle_cruise_active(&physics, phys_id);
                    if (cruise_on) {
                        float target_ms = physics_vehicle_cruise_target(&physics, phys_id);
                        int target_mph = (int)(target_ms * 2.237f);
                        snprintf(col_buf, sizeof(col_buf), "Cruise: %d mph", target_mph);
                        text_draw(&text_renderer, col_buf, col4, row1_y, UI_COLOR_ACCENT);
                    } else {
                        text_draw(&text_renderer, "Cruise: OFF", col4, row1_y, UI_COLOR_DISABLED);
                    }
                } else {
                    // Turn mode: show next action
                    const char* action_names[] = {"-5 mph (Brake)", "Hold Speed", "+5 mph (Accel)"};
                    snprintf(col_buf, sizeof(col_buf), "Action: %s", action_names[planning.speed_choice]);
                    text_draw(&text_renderer, col_buf, col4, row1_y, UI_COLOR_ACCENT);
                }

                // Handling
                snprintf(col_buf, sizeof(col_buf), "Handling: %d/%d", hs, hc);
                text_draw(&text_renderer, col_buf, col5, row1_y, UI_COLOR_WHITE);

                // ===== ROW 2: Mode-specific details =====
                if (!physics_is_paused(&physics)) {
                    // Freestyle mode: Steering, Traction, Drift status
                    // Steering
                    if (discrete_steering) {
                        if (steering_level == 0) {
                            snprintf(col_buf, sizeof(col_buf), "Steering: Center");
                        } else {
                            int d_level = steering_level < 0 ? -steering_level : steering_level;
                            const char* dir = steering_level < 0 ? "Left" : "Right";
                            snprintf(col_buf, sizeof(col_buf), "Steering: %s D%d", dir, d_level);
                        }
                    } else {
                        snprintf(col_buf, sizeof(col_buf), "Steering: Analog");
                    }
                    text_draw(&text_renderer, col_buf, col1, row2_y, UI_COLOR_WHITE);

                    // Traction (moved to col2)
                    float traction = 0.0f;
                    if (phys_id >= 0) {
                        float force_n;
                        physics_vehicle_get_traction_info(&physics, phys_id, &force_n, &traction);
                    }
                    int wheels_contact = (int)(traction * 4.0f + 0.5f);
                    snprintf(col_buf, sizeof(col_buf), "Traction: %d/4", wheels_contact);
                    text_draw(&text_renderer, col_buf, col2, row2_y, UI_COLOR_WHITE);

                    // Gear and RPM (col3 area)
                    int gear = 0;
                    float rpm = 0.0f;
                    int raw_gear = 0;
                    bool is_matchbox = false;
                    if (phys_id >= 0) {
                        physics_vehicle_get_drivetrain_info(&physics, phys_id, &gear, &rpm, &raw_gear, &is_matchbox);
                    }

                    if (is_matchbox) {
                        // Matchbox mode - drivetrain is fake, show N/A
                        snprintf(col_buf, sizeof(col_buf), "Gear: N/A");
                    } else if (gear < 0) {
                        snprintf(col_buf, sizeof(col_buf), "Gear: R RPM: %.0f", rpm);
                    } else if (gear == 0) {
                        snprintf(col_buf, sizeof(col_buf), "Gear: N RPM: %.0f", rpm);
                    } else {
                        snprintf(col_buf, sizeof(col_buf), "Gear: %d RPM: %.0f", gear, rpm);
                    }
                    text_draw(&text_renderer, col_buf, col3, row2_y, UI_COLOR_WHITE);

                    // Throttle (col4) - shows actual physics throttle (reflects TCS adjustments)
                    float phys_throttle = 0.0f;
                    if (phys_id >= 0 && physics.vehicles[phys_id].active) {
                        phys_throttle = physics.vehicles[phys_id].throttle;
                    }
                    int throttle_pct = (int)(phys_throttle * 100.0f + 0.5f);
                    snprintf(col_buf, sizeof(col_buf), "Throttle: %d%%", throttle_pct);
                    text_draw(&text_renderer, col_buf, col4, row2_y, UI_COLOR_WHITE);

                    // Slip percentage (col5) - shows max wheel slip for debugging TCS
                    float max_slip = 0.0f;
                    if (phys_id >= 0) {
                        WheelState wheel_states[4];
                        physics_vehicle_get_wheel_states(&physics, phys_id, wheel_states);
                        for (int w = 0; w < 4; w++) {
                            float slip = fabsf(wheel_states[w].longitudinal_slip);
                            if (slip > max_slip) max_slip = slip;
                        }
                    }
                    int slip_pct = (int)(max_slip * 100.0f + 0.5f);
                    if (slip_pct > 15) {
                        snprintf(col_buf, sizeof(col_buf), "Slip: %d%%", slip_pct);
                        text_draw(&text_renderer, col_buf, col5, row2_y, UI_COLOR_DANGER);
                    } else if (slip_pct > 5) {
                        snprintf(col_buf, sizeof(col_buf), "Slip: %d%%", slip_pct);
                        text_draw(&text_renderer, col_buf, col5, row2_y, UI_COLOR_CAUTION);
                    } else {
                        snprintf(col_buf, sizeof(col_buf), "Slip: %d%%", slip_pct);
                        text_draw(&text_renderer, col_buf, col5, row2_y, UI_COLOR_SAFE);
                    }
                } else {
                    // Turn mode: Maneuver info (single maneuver per turn)
                    snprintf(col_buf, sizeof(col_buf), "Paused at: %d mph", planning.snapshot_speed);
                    text_draw(&text_renderer, col_buf, col1, row2_y, UI_COLOR_WHITE);

                    // Show declared maneuver
                    ManeuverType m = planning.maneuver;
                    ManeuverDirection d = planning.direction;
                    const char* dir_str = (d == MANEUVER_LEFT) ? "Left" : "Right";

                    if (m == MANEUVER_NONE || m == MANEUVER_STRAIGHT) {
                        snprintf(col_buf, sizeof(col_buf), "Maneuver: Straight");
                    } else if (m == MANEUVER_DRIFT) {
                        snprintf(col_buf, sizeof(col_buf), "Maneuver: Drift %s", dir_str);
                    } else if (m == MANEUVER_STEEP_DRIFT) {
                        snprintf(col_buf, sizeof(col_buf), "Maneuver: Steep %s", dir_str);
                    } else if (m == MANEUVER_BEND) {
                        snprintf(col_buf, sizeof(col_buf), "Maneuver: Bend %d %s",
                                 planning.bend_angle, dir_str);
                    } else if (m == MANEUVER_SWERVE) {
                        snprintf(col_buf, sizeof(col_buf), "Maneuver: Swerve %d %s",
                                 planning.bend_angle, dir_str);
                    } else {
                        snprintf(col_buf, sizeof(col_buf), "Maneuver: Straight");
                    }
                    text_draw(&text_renderer, col_buf, col3, row2_y, UI_COLOR_WHITE);
                }
            }

            text_renderer_end(&text_renderer);
        }

        // Help overlay (renders on top of everything)
        if (show_help) {
            // Semi-transparent background panel - upper left, auto-height
            float help_w = 320.0f;
            float help_h = platform.height - 30.0f;  // Full window height minus margins
            float help_x = 15.0f;
            float help_y = 15.0f;

            ui_renderer_begin(&ui_renderer, platform.width, platform.height);
            ui_draw_panel(&ui_renderer,
                ui_rect(help_x, help_y, help_w, help_h),
                ui_color(0.05f, 0.05f, 0.1f, 0.7f),
                ui_color(0.3f, 0.5f, 0.8f, 0.5f), 1.0f, 6.0f);
            ui_renderer_end(&ui_renderer);

            if (has_text) {
                text_renderer_begin(&text_renderer, platform.width, platform.height);

                float tx = help_x + 15;
                float ty = help_y + 12;
                float line_h = 24.0f;

                text_draw(&text_renderer, "CONTROLS (F1)", tx, ty, UI_COLOR_ACCENT);
                ty += line_h * 1.3f;

                text_draw(&text_renderer, "CAMERA", tx, ty, UI_COLOR_CAUTION);
                ty += line_h;
                text_draw(&text_renderer, "  MMB+drag  Look", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  WASD      Move", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  E/Space   Up", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  Q/Ctrl    Down", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  Shift     Fast", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  C         Chase cam", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  MMB+drag  Orbit (chase)", tx, ty, UI_COLOR_WHITE);
                ty += line_h * 1.3f;

                text_draw(&text_renderer, "SELECTION", tx, ty, UI_COLOR_CAUTION);
                ty += line_h;
                text_draw(&text_renderer, "  LMB       Click select", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  1-0       Select vehicle + chase", tx, ty, UI_COLOR_WHITE);
                ty += line_h * 1.3f;

                text_draw(&text_renderer, "TURN MODE (TAB)", tx, ty, UI_COLOR_CAUTION);
                ty += line_h;
                text_draw(&text_renderer, "  TAB       Pause/Unpause", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  (Use GUI to declare maneuvers)", tx, ty, UI_COLOR_DISABLED);
                ty += line_h * 1.3f;

                text_draw(&text_renderer, "GAMEPLAY", tx, ty, UI_COLOR_CAUTION);
                ty += line_h;
                text_draw(&text_renderer, "  Up/Down   Throttle/Reverse", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  L/R       Steer (tap=D1-D3)", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  M         Steer mode toggle", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  V         Cruise on/off", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  ]         Cruise +10mph", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  [         Cruise -10mph", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  Space     Brake", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  T         Accel Test", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  TAB       Turn Based mode", tx, ty, UI_COLOR_WHITE);
                ty += line_h * 1.3f;

                text_draw(&text_renderer, "DEBUG", tx, ty, UI_COLOR_CAUTION);
                ty += line_h;
                text_draw(&text_renderer, "  P         Physics", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  H         Hide cars", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  G         Ghost", tx, ty, UI_COLOR_WHITE);
                ty += line_h * 1.3f;

                text_draw(&text_renderer, "SYSTEM", tx, ty, UI_COLOR_CAUTION);
                ty += line_h;
                text_draw(&text_renderer, "  F11       Fullscreen", tx, ty, UI_COLOR_WHITE);
                ty += line_h;
                text_draw(&text_renderer, "  ESC       Quit", tx, ty, UI_COLOR_WHITE);

                text_renderer_end(&text_renderer);
            }
        }

        // Swap buffers
        platform_swap_buffers(&platform);
    }

    // Cleanup
    if (script_engine) reflex_destroy(script_engine);
    physics_destroy(&physics);
    if (has_lines) line_renderer_destroy(&line_renderer);
    if (has_text) text_renderer_destroy(&text_renderer);
    ui_renderer_destroy(&ui_renderer);
    // Destroy all vehicle meshes and textures
    for (int t = 0; t < g_vehicle_type_count; t++) {
        obj_destroy(&g_vehicle_meshes[t].body);
        obj_destroy(&g_vehicle_meshes[t].wheel);
        if (g_vehicle_meshes[t].texture) {
            texture_destroy(g_vehicle_meshes[t].texture);
        }
    }
    box_renderer_destroy(&box_renderer);
    floor_destroy(&arena_floor);
    platform_shutdown(&platform);
    printf("Goodbye!\n");

    return 0;
}
