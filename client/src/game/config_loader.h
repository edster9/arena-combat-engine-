/*
 * JSON Configuration Loader
 * Hot-reloadable vehicle and scene configuration
 */

#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <stdbool.h>
#include "../math/vec3.h"
#include "../physics/ode_physics.h"

#define MAX_SCENE_VEHICLES 8
#define MAX_SCENE_OBSTACLES 32
#define MAX_NAME_LENGTH 64
#define MAX_WHEELS 16          // Max wheels per vehicle (supports trucks, etc.)
#define MAX_AXLES 8            // Max axles per vehicle
#define MAX_WHEELS_PER_AXLE 4  // Max wheels on one axle (dual rear wheels)

// Vehicle class types
typedef enum {
    VEHICLE_CLASS_WHEELED = 0,  // Cars, trucks, motorcycles (tire-based)
    VEHICLE_CLASS_TRACKED,      // Tanks, bulldozers (future)
    VEHICLE_CLASS_HOVER,        // Hovercraft (future)
    VEHICLE_CLASS_AIRCRAFT,     // Helicopters, planes (future)
    VEHICLE_CLASS_BOAT,         // Boats, ships (future)
    VEHICLE_CLASS_UNKNOWN
} VehicleClass;

// Drivetrain types
typedef enum {
    DRIVETRAIN_RWD = 0,   // Rear wheel drive
    DRIVETRAIN_FWD,       // Front wheel drive
    DRIVETRAIN_AWD,       // All wheel drive
    DRIVETRAIN_4WD        // Four wheel drive (with transfer case)
} DrivetrainType;

// Individual wheel definition (v2 format)
typedef struct {
    char id[16];          // Wheel identifier (e.g., "FL", "RR", "RL2")
    Vec3 position;        // Position relative to chassis center
    float radius;         // Wheel radius in meters
    float width;          // Wheel width in meters
    float mass;           // Wheel mass in kg
} WheelDef;

// Axle definition (v2 format)
typedef struct {
    char name[MAX_NAME_LENGTH];           // Axle name/id (e.g., "front", "rear", "middle")
    char wheel_ids[MAX_WHEELS_PER_AXLE][16];  // IDs of wheels on this axle
    int wheel_count;                      // Number of wheels on this axle
    bool steering;                        // Can this axle steer?
    bool driven;                          // Is this axle powered?
    float max_steer_angle;                // Max steering angle (if steering)
    // Per-axle suspension (optional - falls back to vehicle default)
    float suspension_erp;
    float suspension_cfm;
    float suspension_travel;
    bool has_suspension;                  // True if axle specifies its own suspension
    // Per-axle brakes
    float brake_force_multiplier;         // Brake force = mass * multiplier
} AxleDef;

// Wheel defaults (v2 format) - values used when wheel doesn't specify its own
typedef struct {
    float radius;
    float width;
    float mass;
    float friction;       // Tire friction coefficient (mu)
    float slip;           // Tire slip allowed
} WheelDefaults;

// Suspension defaults (v2 format)
typedef struct {
    float erp;            // Error reduction parameter
    float cfm;            // Constraint force mixing
    float travel;         // Suspension travel in meters
} SuspensionDef;

// Vehicle-level defaults block (contains wheel + suspension defaults)
typedef struct {
    WheelDefaults wheel;
    SuspensionDef suspension;
} VehicleDefaults;

// Drivetrain configuration (v2 format)
typedef struct {
    DrivetrainType type;
    float motor_force;    // Max motor force in Newtons
    float brake_force;    // Max brake force in Newtons
    float max_speed;      // Max speed in units/sec
} DrivetrainDef;

// Legacy: Tire configuration (for backwards compatibility)
typedef struct {
    float friction;       // Friction coefficient (mu)
    float slip;           // Slip allowed
} TireDef;

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

    // V2 format: Drivetrain
    DrivetrainDef drivetrain;

    // Legacy V1 format: flat physics struct (for backwards compatibility)
    VehicleConfig physics;
    float motor_max_speed;
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
    char type[MAX_NAME_LENGTH];  // "box" for now
    Vec3 position;
    Vec3 size;
    Vec3 color;
} SceneObstacle;

// Contact physics parameters
typedef struct {
    float friction;
    float slip;
    float soft_erp;
    float soft_cfm;
} ContactConfig;

// World physics parameters
typedef struct {
    float gravity;
    float erp;
    float cfm;
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

#endif // CONFIG_LOADER_H
