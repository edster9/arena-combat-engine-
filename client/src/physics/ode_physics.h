/*
 * ODE Physics Integration
 * Vehicle physics with independent wheel suspension
 */

#ifndef ODE_PHYSICS_H
#define ODE_PHYSICS_H

#include <stdbool.h>
#include "../math/vec3.h"

// Forward declare ODE types (avoid including ode.h in header)
typedef struct dxWorld* dWorldID;
typedef struct dxSpace* dSpaceID;
typedef struct dxBody* dBodyID;
typedef struct dxGeom* dGeomID;
typedef struct dxJoint* dJointID;
typedef struct dxJointGroup* dJointGroupID;

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
    float chassis_length;    // meters (X axis)
    float chassis_width;     // meters (Z axis)
    float chassis_height;    // meters (Y axis)

    // Per-wheel configuration (v2 format)
    // Order: FL=0, FR=1, RL=2, RR=3
    Vec3 wheel_positions[4]; // Position relative to chassis center
    float wheel_radii[4];    // Radius per wheel (meters)
    float wheel_widths[4];   // Width per wheel (meters)
    float wheel_masses[4];   // Mass per wheel (kg)
    bool wheel_steering[4];  // Which wheels can steer
    bool wheel_driven[4];    // Which wheels are powered
    float wheel_steer_angles[4]; // Max steer angle per wheel (radians)

    // Per-wheel suspension (v2 format, set per-axle in JSON)
    float wheel_suspension_erp[4];
    float wheel_suspension_cfm[4];
    float wheel_suspension_travel[4];

    // Legacy single-wheel values (used as fallback if per-wheel not set)
    float wheel_mass;        // kg per wheel
    float wheel_radius;      // meters
    float wheel_width;       // meters

    // Default suspension values (used when per-wheel not specified)
    float suspension_erp;    // Error reduction (0.1-0.8, higher = stiffer)
    float suspension_cfm;    // Constraint force mixing (softness)
    float suspension_travel; // Max suspension travel in meters

    float max_steer_angle;   // Max steering angle in radians
    float max_motor_force;   // Max engine force (Newtons)
    float max_brake_force;   // Max brake force (Newtons)

    // Tire properties
    float tire_friction;     // Friction coefficient (mu), higher = more grip
    float tire_slip;         // Slip allowed (lower = more grip, higher = more drift)

    // Wheel mount points (legacy - used if wheel_positions all zero)
    float wheelbase;         // Distance between front and rear axles (Z axis)
    float track_width;       // Distance between left and right wheels (X axis)
    float wheel_mount_height;// How far below chassis center the suspension mounts

    // Flag to indicate v2 per-wheel config is active
    bool use_per_wheel_config;
} VehicleConfig;

// Per-wheel state (for rendering)
typedef struct {
    Vec3 position;
    float rotation;          // Wheel spin angle
    float steer_angle;       // Current steering angle (front wheels)
    float suspension_compression; // 0 = fully extended, 1 = fully compressed
    float rot_matrix[12];    // ODE rotation matrix (3x4, column-major)
} WheelState;

// Physics vehicle handle
typedef struct {
    int id;
    bool active;

    // ODE bodies
    dBodyID chassis;
    dBodyID wheels[4];

    // ODE joints (Hinge2 = suspension + steering)
    dJointID suspensions[4];

    // ODE geometry for collision
    dGeomID chassis_geom;
    dGeomID wheel_geoms[4];

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
} PhysicsVehicle;

// Physics world
typedef struct {
    dWorldID world;
    dSpaceID space;
    dJointGroupID contact_group;
    dGeomID ground;

    PhysicsVehicle vehicles[MAX_PHYSICS_VEHICLES];
    int vehicle_count;

    float step_size;         // Physics timestep
    float accumulator;       // Time accumulator for fixed timestep
} PhysicsWorld;

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

// Get vehicle state (for rendering)
void physics_vehicle_get_position(PhysicsWorld* pw, int vehicle_id, Vec3* pos);
void physics_vehicle_get_rotation(PhysicsWorld* pw, int vehicle_id, float* rotation_y);
void physics_vehicle_get_rotation_matrix(PhysicsWorld* pw, int vehicle_id, float* rot_matrix);  // 9 floats (3x3 row-major)
void physics_vehicle_get_velocity(PhysicsWorld* pw, int vehicle_id, float* speed_ms);
void physics_vehicle_get_wheel_states(PhysicsWorld* pw, int vehicle_id, WheelState* wheels);

// Default vehicle config (good starting point)
VehicleConfig physics_default_vehicle_config(void);

// Debug visualization - call between line_renderer_begin/end
struct LineRenderer;  // Forward declare
void physics_debug_draw(PhysicsWorld* pw, struct LineRenderer* lr);

#endif // ODE_PHYSICS_H
