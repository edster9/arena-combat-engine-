/*
 * JSON Configuration Loader
 * Hot-reloadable vehicle and scene configuration
 */

#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <stdbool.h>
#include "../math/vec3.h"
#include "../physics/jolt_physics.h"

#define MAX_SCENE_VEHICLES 8
#define MAX_SCENE_OBSTACLES 32
#define MAX_NAME_LENGTH 64
#define MAX_WHEELS 16          // Max wheels per vehicle (supports trucks, etc.)
#define MAX_AXLES 8            // Max axles per vehicle
#define MAX_WHEELS_PER_AXLE 4  // Max wheels on one axle (dual rear wheels)
#define MAX_VEHICLE_SCRIPTS 4  // Max scripts per vehicle
#define MAX_SCRIPT_OPTIONS 8   // Max config options per script
#define MAX_SCRIPT_PATH 128    // Max path length for script file

// Vehicle class types
typedef enum {
    VEHICLE_CLASS_WHEELED = 0,  // Cars, trucks, motorcycles (tire-based)
    VEHICLE_CLASS_TRACKED,      // Tanks, bulldozers (future)
    VEHICLE_CLASS_HOVER,        // Hovercraft (future)
    VEHICLE_CLASS_AIRCRAFT,     // Helicopters, planes (future)
    VEHICLE_CLASS_BOAT,         // Boats, ships (future)
    VEHICLE_CLASS_UNKNOWN
} VehicleClass;

// Axle position (semantic - front-to-back ordering)
typedef enum {
    AXLE_POSITION_FRONT = 0,    // Frontmost axle
    AXLE_POSITION_REAR,         // Rearmost axle
    AXLE_POSITION_MIDDLE,       // Middle axle (for 3+ axle vehicles)
    AXLE_POSITION_UNKNOWN
} AxlePosition;

// Wheel side (semantic - left/right relative to vehicle forward)
typedef enum {
    WHEEL_SIDE_LEFT = 0,        // Left side (positive Z in vehicle coords)
    WHEEL_SIDE_RIGHT,           // Right side (negative Z in vehicle coords)
    WHEEL_SIDE_CENTER,          // Center wheel (motorcycles, some trucks)
    WHEEL_SIDE_UNKNOWN
} WheelSide;

// Individual wheel definition (v2 format)
typedef struct {
    char id[16];          // Wheel identifier (e.g., "FL", "RR", "RL2")
    Vec3 position;        // Position relative to chassis center
    float radius;         // Wheel radius in meters
    float width;          // Wheel width in meters
    float mass;           // Wheel mass in kg
    // Semantic info (populated during config loading)
    WheelSide side;       // Left/right/center position
    int axle_index;       // Index into axles[] array (-1 if not assigned)
} WheelDef;

// Axle definition (v2 format)
typedef struct {
    char name[MAX_NAME_LENGTH];           // Axle name/id (e.g., "front", "rear", "middle")
    char wheel_ids[MAX_WHEELS_PER_AXLE][16];  // IDs of wheels on this axle
    int wheel_count;                      // Number of wheels on this axle
    bool steering;                        // Can this axle steer?
    bool driven;                          // Is this axle powered?
    float max_steer_angle;                // Max steering angle (if steering)
    // Per-axle suspension (Jolt style: frequency/damping)
    float suspension_frequency;           // Spring frequency in Hz
    float suspension_damping;             // Damping ratio (0-1)
    float suspension_travel;
    bool has_suspension;                  // True if axle specifies its own suspension
    // Per-axle brakes
    float brake_force_multiplier;         // Brake force = mass * multiplier
    // Semantic info
    AxlePosition position;                // Front/rear/middle (for handbrake, ABS logic)
    bool has_handbrake;                   // Handbrake affects this axle (typically rear)
} AxleDef;

// Wheel defaults (v2 format) - values used when wheel doesn't specify its own
typedef struct {
    float radius;
    float width;
    float mass;
    float friction;       // Tire friction coefficient (mu)
} WheelDefaults;

// Suspension defaults (v2 format) - Jolt style
typedef struct {
    float frequency;      // Spring frequency in Hz (1.0-3.0 typical)
    float damping;        // Damping ratio (0-1), 0.5 = critical damping
    float travel;         // Suspension travel in meters
} SuspensionDef;

// Vehicle-level defaults block (contains wheel + suspension defaults)
typedef struct {
    WheelDefaults wheel;
    SuspensionDef suspension;
} VehicleDefaults;

// Legacy: Tire configuration (for backwards compatibility)
typedef struct {
    float friction;       // Friction coefficient (mu) - Jolt handles slip internally
} TireDef;

// Script configuration option (key-value pair)
typedef struct {
    char key[32];         // Option name (e.g., "abs_enabled", "pulse_rate")
    float value;          // Numeric value (booleans: 0.0 = false, 1.0 = true)
} ScriptOption;

// Script definition for vehicle
typedef struct {
    char name[MAX_NAME_LENGTH];           // Script identifier (e.g., "freestyle_assist")
    char path[MAX_SCRIPT_PATH];           // Path to Lua script file
    bool enabled;                         // Whether script is active
    ScriptOption options[MAX_SCRIPT_OPTIONS];  // Configuration options
    int option_count;
} VehicleScript;

// Vehicle configuration loaded from JSON
typedef struct {
    // Meta info
    VehicleClass vehicle_class;
    int version;                // Config format version (1 = legacy, 2 = flexible)
    char name[MAX_NAME_LENGTH];

    // Chassis (shared between v1 and v2)
    float chassis_mass;
    float chassis_length;
    float chassis_width;
    float chassis_height;

    // V2 format: Consolidated defaults (wheel + suspension)
    VehicleDefaults defaults;

    // V2 format: Individual wheel definitions
    WheelDef wheels[MAX_WHEELS];
    int wheel_count;

    // V2 format: Axle definitions
    AxleDef axles[MAX_AXLES];
    int axle_count;

    // Legacy V1 format: flat physics struct (for backwards compatibility)
    VehicleConfig physics;
    float motor_max_speed;

    // Scripts attached to this vehicle
    VehicleScript scripts[MAX_VEHICLE_SCRIPTS];
    int script_count;
} VehicleJSON;

// Vehicle spawn in scene
typedef struct {
    char type[MAX_NAME_LENGTH];  // References vehicle JSON name
    char team[MAX_NAME_LENGTH];  // "red" or "blue"
    Vec3 position;
    float rotation;
} SceneVehicle;

// Obstacle in scene
typedef struct {
    char type[MAX_NAME_LENGTH];  // "box" or "ramp"
    Vec3 position;
    Vec3 size;                   // For ramp: x=width, y=height, z=length
    Vec3 color;
    float rotation_y;            // Rotation around Y axis (radians)
} SceneObstacle;

// Contact physics parameters (simplified for Jolt)
typedef struct {
    float friction;
    float restitution;    // Bounciness (0-1)
} ContactConfig;

// World physics parameters (simplified for Jolt)
typedef struct {
    float gravity;
    ContactConfig contact;
} WorldConfig;

// Arena configuration
typedef struct {
    float size;
    float wall_height;
    float wall_thickness;
    float ground_y;
} ArenaConfig;

// Full scene configuration
typedef struct {
    char name[MAX_NAME_LENGTH];
    ArenaConfig arena;
    WorldConfig world;

    SceneVehicle vehicles[MAX_SCENE_VEHICLES];
    int vehicle_count;

    SceneObstacle obstacles[MAX_SCENE_OBSTACLES];
    int obstacle_count;
} SceneJSON;

// Physics mode types
typedef enum {
    PHYSICS_MODE_STRICT_CAR_WARS = 0,  // Enforces tabletop rules
    PHYSICS_MODE_EXTENDED              // Full configurable physics
} PhysicsModeType;

// Physics mode tire overrides
typedef struct {
    float mu;              // Standardized friction coefficient
    float reference_radius; // Reference tire radius for torque calc
    float reference_width;  // Reference tire width
} PhysicsModeTireOverrides;

// Physics mode suspension overrides
typedef struct {
    float frequency;
    float damping;
    float travel;
} PhysicsModeSuspensionOverrides;

// Physics mode overrides
typedef struct {
    PhysicsModeTireOverrides tire;
    bool override_tire;
    PhysicsModeSuspensionOverrides suspension;
    bool override_suspension;
} PhysicsModeOverrides;

// Physics mode configuration
typedef struct {
    char name[MAX_NAME_LENGTH];
    PhysicsModeType type;
    PhysicsModeOverrides overrides;
} PhysicsMode;

// Friction curve point (slip_ratio, friction_multiplier)
typedef struct {
    float slip;
    float friction;
} FrictionPoint;

#define MAX_FRICTION_POINTS 8

// Friction curve configuration (loaded from physics.json)
typedef struct {
    char name[MAX_NAME_LENGTH];
    FrictionPoint longitudinal[MAX_FRICTION_POINTS];
    int longitudinal_count;
    FrictionPoint lateral[MAX_FRICTION_POINTS];
    int lateral_count;
    float lateral_rails_multiplier;
} FrictionCurve;

// Load vehicle configuration from JSON file
// Returns true on success, false on error (uses defaults on error)
bool config_load_vehicle(const char* filepath, VehicleJSON* out);

// Load scene configuration from JSON file
// Returns true on success, false on error
bool config_load_scene(const char* filepath, SceneJSON* out);

// Convert VehicleJSON to VehicleConfig (for physics system)
VehicleConfig config_vehicle_to_physics(const VehicleJSON* json);

// Get default vehicle config (fallback if JSON fails)
VehicleJSON config_default_vehicle(void);

// Get default scene config (fallback if JSON fails)
SceneJSON config_default_scene(void);

// Load physics mode configuration from JSON file
// Returns true on success, false on error (uses strict mode as default)
bool config_load_physics_mode(const char* filepath, PhysicsMode* out);

// Get the currently active physics mode
const PhysicsMode* config_get_physics_mode(void);

// Apply physics mode overrides to a vehicle config
// Call this after loading vehicle config but before passing to physics
void config_apply_physics_mode(VehicleJSON* vehicle, const PhysicsMode* mode);

// Check if we're in strict tabletop mode
bool config_is_strict_mode(void);

// Load friction curve configuration from physics.json
// Returns true on success, false on error (uses generous curve as default)
bool config_load_friction_curve(const char* filepath, FrictionCurve* out);

// Get the currently active friction curve
const FrictionCurve* config_get_friction_curve(void);

#endif // CONFIG_LOADER_H
