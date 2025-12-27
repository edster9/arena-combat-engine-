/*
 * Equipment Loader
 * Loads Car Wars equipment data and provides lookup functions
 */

#ifndef EQUIPMENT_LOADER_H
#define EQUIPMENT_LOADER_H

#include <stdbool.h>

#define MAX_EQUIPMENT_ID 64
#define MAX_EQUIPMENT_NAME 64
#define MAX_EQUIPMENT_ITEMS 64

// Physics conversion constants
#define LBS_TO_KG 0.453592f
#define MPH_TO_MS 0.44704f
#define POWER_FACTOR_TO_FORCE 2.5f

// Chassis equipment (body type)
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    int cost;
    int weight_lbs;
    int max_load_lbs;
    int spaces;
    // Physics dimensions (meters)
    float length_default;
    float length_min;
    float length_max;
    float width_default;
    float width_min;
    float width_max;
    float height_default;
    float height_min;
    float height_max;
} ChassisEquipment;

// Power plant equipment
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    char type[32];  // "electric", "gas", "nuclear"
    int cost;
    int weight_lbs;
    int spaces;
    int power_factors;
    int power_units;
    // Calculated physics values
    float motor_force;      // Newtons (power_factors * 2.5)
    float weight_kg;        // Converted from lbs
} PowerPlantEquipment;

// Tire base type
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    int cost;
    int weight_lbs;
    int dp;
    // Physics values
    float mu;       // Friction coefficient
    float slip1;    // Lateral slip
    float slip2;    // Longitudinal slip
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
    float slip1_modifier;   // Multiplied with base slip1
    float slip2_modifier;   // Multiplied with base slip2
} TireModifier;

// Suspension type
typedef struct {
    char id[MAX_EQUIPMENT_ID];
    char name[MAX_EQUIPMENT_NAME];
    float cost_modifier;
    int hc_car;
    int hc_van;
    // Physics values
    float erp;      // Error reduction parameter
    float cfm;      // Constraint force mixing
    float travel;   // Suspension travel in meters
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
const TireEquipment* equipment_find_tire(const char* id);
const TireModifier* equipment_find_tire_modifier(const char* id);
const SuspensionEquipment* equipment_find_suspension(const char* id);
const BrakeEquipment* equipment_find_brake(const char* id);

// Calculate combined tire physics after applying modifiers
typedef struct {
    float mu;
    float slip1;
    float slip2;
} TirePhysics;

TirePhysics equipment_calc_tire_physics(const char* tire_id, const char** modifier_ids, int modifier_count);

// Calculate top speed from power plant and total weight
// type: "electric" or "gas"
float equipment_calc_top_speed_mph(const char* type, int power_factors, int total_weight_lbs);
float equipment_calc_top_speed_ms(const char* type, int power_factors, int total_weight_lbs);

// Calculate acceleration class from power factor to weight ratio
int equipment_calc_accel_class(int power_factors, int total_weight_lbs);

#endif // EQUIPMENT_LOADER_H
