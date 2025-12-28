/*
 * Jolt Physics Integration
 * Vehicle physics with WheeledVehicleController
 */

#ifndef JOLT_PHYSICS_H
#define JOLT_PHYSICS_H

#include <stdbool.h>
#include "../math/vec3.h"

// Maximum vehicles in physics world
#define MAX_PHYSICS_VEHICLES 8

// Wheel indices
#define WHEEL_FL 0  // Front Left
#define WHEEL_FR 1  // Front Right
#define WHEEL_RL 2  // Rear Left
#define WHEEL_RR 3  // Rear Right

// Vehicle configuration
typedef struct {
    float chassis_mass;      // kg
    float chassis_length;    // meters (Z axis - forward)
    float chassis_width;     // meters (X axis - side to side)
    float chassis_height;    // meters (Y axis - up)

    // Per-wheel configuration
    // Order: FL=0, FR=1, RL=2, RR=3
    Vec3 wheel_positions[4]; // Position relative to chassis center
    float wheel_radii[4];    // Radius per wheel (meters)
    float wheel_widths[4];   // Width per wheel (meters)
    float wheel_masses[4];   // Mass per wheel (kg)
    bool wheel_steering[4];  // Which wheels can steer
    bool wheel_driven[4];    // Which wheels are powered
    float wheel_steer_angles[4]; // Max steer angle per wheel (radians)

    // Per-wheel suspension (Jolt style: frequency/damping)
    float wheel_suspension_frequency[4];  // Spring frequency in Hz (1.0-2.0 typical)
    float wheel_suspension_damping[4];    // Damping ratio (0.0-1.0, 0.5 typical)
    float wheel_suspension_travel[4];     // Max travel in meters

    // Legacy single-wheel values (used as fallback if per-wheel not set)
    float wheel_mass;        // kg per wheel
    float wheel_radius;      // meters
    float wheel_width;       // meters

    // Default suspension values (used when per-wheel not specified)
    float suspension_frequency;  // Spring frequency in Hz
    float suspension_damping;    // Damping ratio
    float suspension_travel;     // Max suspension travel in meters

    float max_steer_angle;   // Max steering angle in radians

    // Engine/drivetrain
    float engine_max_torque; // Nm
    float engine_max_rpm;    // Max RPM

    // Transmission/gearbox
    #define MAX_CONFIG_GEARS 8
    float gear_ratios[MAX_CONFIG_GEARS];
    int gear_count;
    float reverse_ratios[MAX_CONFIG_GEARS];
    int reverse_count;
    float differential_ratio;
    bool use_config_transmission;  // If true, use these values instead of hardcoded

    // Tire properties
    float tire_friction;     // Friction coefficient (mu), higher = more grip

    // Legacy values for compatibility
    float max_motor_force;   // Will be converted to torque
    float max_brake_force;   // Brake force (Newtons)

    // Car Wars acceleration targets (for tuning/testing)
    int power_factors;            // Total power factors from power plant(s)
    float target_accel_ms2;       // Target acceleration in m/s² (from PF/weight ratio)
    float target_0_60_seconds;    // Target 0-60 time in seconds
    char vehicle_name[64];        // Vehicle name for test output

    // Car Wars linear acceleration mode
    // When enabled, bypasses engine simulation and applies constant F = m * a
    bool use_linear_accel;        // Enable direct force application
    float linear_accel_force;     // Force in Newtons (mass * target_accel)
    float top_speed_ms;           // Top speed limit in m/s (from Car Wars tables)

    // Wheel mount points (legacy - used if wheel_positions all zero)
    float wheelbase;         // Distance between front and rear axles
    float track_width;       // Distance between left and right wheels
    float wheel_mount_height;// How far below chassis center the suspension mounts

    // Flag to indicate per-wheel config is active
    bool use_per_wheel_config;
} VehicleConfig;

// Per-wheel state (for rendering)
typedef struct {
    Vec3 position;
    float rotation;          // Wheel spin angle
    float steer_angle;       // Current steering angle
    float suspension_compression; // 0 = fully extended, 1 = fully compressed
    float rot_matrix[12];    // Rotation matrix (3x4, column-major)
} WheelState;

// Opaque physics types (implementation hidden)
struct PhysicsWorldImpl;
struct PhysicsVehicleImpl;

// Physics vehicle handle
typedef struct {
    int id;
    bool active;

    struct PhysicsVehicleImpl* impl;

    // Current state
    WheelState wheel_states[4];
    float steering;          // Current steering input (-1 to 1)
    float throttle;          // Current throttle (0 to 1)
    float reverse;           // Current reverse (0 to 1)
    float brake;             // Current brake (0 to 1)

    // Spawn state (for respawn)
    Vec3 spawn_position;
    float spawn_rotation;

    // Config
    VehicleConfig config;

    // Acceleration test state
    bool accel_test_active;
    bool accel_test_timing_started;  // True once throttle is applied
    float accel_test_elapsed;    // Time since timing started (seconds)
    float accel_test_print_timer; // Timer for periodic output
    Vec3 accel_test_start_pos;
    float accel_test_last_speed;
} PhysicsVehicle;

// Physics world
typedef struct {
    struct PhysicsWorldImpl* impl;

    PhysicsVehicle vehicles[MAX_PHYSICS_VEHICLES];
    int vehicle_count;

    float step_size;         // Physics timestep
    float accumulator;       // Time accumulator for fixed timestep
} PhysicsWorld;

#ifdef __cplusplus
extern "C" {
#endif

// World management
bool physics_init(PhysicsWorld* pw);
void physics_destroy(PhysicsWorld* pw);
void physics_step(PhysicsWorld* pw, float dt);

// Ground/arena setup
void physics_set_ground(PhysicsWorld* pw, float y_level);
void physics_add_box_obstacle(PhysicsWorld* pw, Vec3 pos, Vec3 size);
void physics_add_arena_walls(PhysicsWorld* pw, float arena_size, float wall_height, float wall_thickness);

// Vehicle management
int physics_create_vehicle(PhysicsWorld* pw, Vec3 position, float rotation_y, const VehicleConfig* config);
void physics_destroy_vehicle(PhysicsWorld* pw, int vehicle_id);

// Vehicle control
void physics_vehicle_set_steering(PhysicsWorld* pw, int vehicle_id, float steering);  // -1 to 1
void physics_vehicle_set_throttle(PhysicsWorld* pw, int vehicle_id, float throttle);  // 0 to 1
void physics_vehicle_set_reverse(PhysicsWorld* pw, int vehicle_id, float reverse);    // 0 to 1
void physics_vehicle_set_brake(PhysicsWorld* pw, int vehicle_id, float brake);        // 0 to 1
void physics_vehicle_respawn(PhysicsWorld* pw, int vehicle_id);  // Reset to spawn position
void physics_vehicle_start_accel_test(PhysicsWorld* pw, int vehicle_id);  // Start 0-60 acceleration test

// Get vehicle state (for rendering)
void physics_vehicle_get_position(PhysicsWorld* pw, int vehicle_id, Vec3* pos);
void physics_vehicle_get_rotation(PhysicsWorld* pw, int vehicle_id, float* rotation_y);
void physics_vehicle_get_rotation_matrix(PhysicsWorld* pw, int vehicle_id, float* rot_matrix);
void physics_vehicle_get_velocity(PhysicsWorld* pw, int vehicle_id, float* speed_ms);
void physics_vehicle_get_wheel_states(PhysicsWorld* pw, int vehicle_id, WheelState* wheels);

// Debug visualization - call between line_renderer_begin/end
struct LineRenderer;  // Forward declare
void physics_debug_draw(PhysicsWorld* pw, struct LineRenderer* lr);

#ifdef __cplusplus
}
#endif

#endif // JOLT_PHYSICS_H
