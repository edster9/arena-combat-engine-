/*
 * Equipment Loader
 * Loads tabletop equipment data and provides lookup functions
 */

#ifndef EQUIPMENT_LOADER_H
#define EQUIPMENT_LOADER_H

#include <stdbool.h>

#define MAX_EQUIPMENT_ID 64
#define MAX_EQUIPMENT_NAME 64
#define MAX_EQUIPMENT_ITEMS 64
#define MAX_WHEEL_MOUNTS 6   // Support up to 6-wheel vehicles

// Physics conversion constants
#define LBS_TO_KG 0.453592f
#define MPH_TO_MS 0.44704f

// Wheel mount position (from chassis)
typedef struct {
    char id[16];        // e.g., "FL", "FR", "RL", "RR"
    float position[3];  // [x, y, z] in chassis space
} WheelMount;

// Chassis equipment (body type)
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    int cost;
    int weight_lbs;
    int max_load_lbs;
    int spaces;
    // Physics dimensions (meters) - now derived from OBJ bounds
    float length_default;
    float length_min;
    float length_max;
    float width_default;
    float width_min;
    float width_max;
    float height_default;
    float height_min;
    float height_max;
    // Center of mass offset [x, y, z] from chassis center
    float center_of_mass[3];
    // Handling class modifier (e.g., subcompact +1, van -1)
    int base_hc_modifier;
    // Model path (relative to assets)
    char model[128];
    // Texture path (relative to assets) - optional, can be overridden by vehicle config
    char texture[128];
    // Wheel mount positions
    WheelMount wheel_mounts[MAX_WHEEL_MOUNTS];
    int wheel_mount_count;
} ChassisEquipment;

// Power plant equipment
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    char type[32];  // "electric", "gas", "nuclear"
    int cost;
    int weight_lbs;
    int spaces;
    int power_units;
    float weight_kg;        // Converted from lbs
    // Real engine specs (car shop values)
    float horsepower;          // Peak HP (display value)
    float torque_nm;           // Peak torque in Nm
    float peak_power_rpm;      // RPM at peak HP
    float peak_torque_rpm;     // RPM at peak torque (0 = instant for electric)
    float redline_rpm;         // Maximum safe RPM
    float idle_rpm;            // Idle RPM
} PowerPlantEquipment;

// Gearbox equipment
#define MAX_GEARS 8
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    int cost;
    int weight_lbs;
    // Physics values
    float gear_ratios[MAX_GEARS];
    int gear_count;
    float reverse_ratios[MAX_GEARS];
    int reverse_count;
    float differential_ratio;
    float shift_up_rpm;
    float shift_down_rpm;
    float clutch_strength;    // Clutch engagement strength (Jolt default ~10)
    float switch_time;        // Time to complete gear change in seconds
    float switch_latency;     // Delay before shift initiates in seconds
} GearboxEquipment;

// Tire base type
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    int cost;
    int weight_lbs;
    int dp;
    // Physics values
    float mu;       // Friction coefficient (Jolt handles slip internally)
} TireEquipment;

// Tire modifier
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    float cost_modifier;
    float weight_modifier;
    int hc_bonus;
    // Physics modifiers
    float mu_bonus;         // Added to base mu
} TireModifier;

// Suspension type
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    float cost_modifier;
    int hc_car;
    int hc_van;
    // Physics values (Jolt style)
    float frequency; // Spring frequency in Hz
    float damping;   // Damping ratio (0-1)
    float travel;    // Suspension travel in meters
} SuspensionEquipment;

// Brake type
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    int cost;
    int weight_lbs;
    // Physics values
    float brake_force_multiplier;  // Multiplied by mass for actual force
} BrakeEquipment;

// Equipment database (singleton-ish, loaded once)
typedef struct {
    ChassisEquipment chassis[MAX_EQUIPMENT_ITEMS];
    int chassis_count;

    PowerPlantEquipment power_plants[MAX_EQUIPMENT_ITEMS];
    int power_plant_count;

    GearboxEquipment gearboxes[MAX_EQUIPMENT_ITEMS];
    int gearbox_count;

    TireEquipment tires[MAX_EQUIPMENT_ITEMS];
    int tire_count;

    TireModifier tire_modifiers[MAX_EQUIPMENT_ITEMS];
    int tire_modifier_count;

    SuspensionEquipment suspensions[MAX_EQUIPMENT_ITEMS];
    int suspension_count;

    BrakeEquipment brakes[MAX_EQUIPMENT_ITEMS];
    int brake_count;

    bool loaded;
} EquipmentDB;

// Global equipment database
extern EquipmentDB g_equipment;

// Load all equipment files from data directory
// base_path should be path to assets/data/equipment/
bool equipment_load_all(const char* base_path);

// Lookup functions - return NULL if not found
const ChassisEquipment* equipment_find_chassis(const char* id);
const PowerPlantEquipment* equipment_find_power_plant(const char* id);
const GearboxEquipment* equipment_find_gearbox(const char* id);
const TireEquipment* equipment_find_tire(const char* id);
const TireModifier* equipment_find_tire_modifier(const char* id);
const SuspensionEquipment* equipment_find_suspension(const char* id);
const BrakeEquipment* equipment_find_brake(const char* id);

// Calculate combined tire physics after applying modifiers
typedef struct {
    float mu;       // Combined friction coefficient
} TirePhysics;

TirePhysics equipment_calc_tire_physics(const char* tire_id, const char** modifier_ids, int modifier_count);

#endif // EQUIPMENT_LOADER_H
