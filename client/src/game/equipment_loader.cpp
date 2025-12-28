/*
 * Equipment Loader Implementation
 * Loads Car Wars equipment JSON files and provides lookup
 */

#include "equipment_loader.h"
#include "../vendor/cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Global equipment database
EquipmentDB g_equipment = {0};

// Read file helper
static char* read_file(const char* filepath) {
    FILE* f = fopen(filepath, "rb");
    if (!f) {
        fprintf(stderr, "Equipment: Failed to open: %s\n", filepath);
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* buffer = (char*)malloc(size + 1);
    if (!buffer) { fclose(f); return NULL; }
    fread(buffer, 1, size, f);
    buffer[size] = '\0';
    fclose(f);
    return buffer;
}

// JSON helpers
static float json_get_float(cJSON* obj, const char* key, float def) {
    cJSON* item = cJSON_GetObjectItem(obj, key);
    return (item && cJSON_IsNumber(item)) ? (float)item->valuedouble : def;
}

static int json_get_int(cJSON* obj, const char* key, int def) {
    cJSON* item = cJSON_GetObjectItem(obj, key);
    return (item && cJSON_IsNumber(item)) ? item->valueint : def;
}

static void json_get_string(cJSON* obj, const char* key, char* out, int maxlen, const char* def) {
    cJSON* item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsString(item)) {
        strncpy(out, item->valuestring, maxlen - 1);
        out[maxlen - 1] = '\0';
    } else {
        strncpy(out, def, maxlen - 1);
        out[maxlen - 1] = '\0';
    }
}

// Load chassis.json
static bool load_chassis(const char* filepath) {
    char* json_str = read_file(filepath);
    if (!json_str) return false;

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) { fprintf(stderr, "Equipment: JSON parse error in chassis.json\n"); return false; }

    // Parse cars section
    cJSON* cars = cJSON_GetObjectItem(root, "cars");
    if (cars) {
        cJSON* item;
        cJSON_ArrayForEach(item, cars) {
            if (g_equipment.chassis_count >= MAX_EQUIPMENT_ITEMS) break;
            ChassisEquipment* c = &g_equipment.chassis[g_equipment.chassis_count];

            json_get_string(item, "id", c->id, MAX_EQUIPMENT_ID, item->string);
            json_get_string(item, "name", c->name, MAX_EQUIPMENT_NAME, "Unknown");
            c->cost = json_get_int(item, "cost", 0);
            c->weight_lbs = json_get_int(item, "weight", 0);
            c->max_load_lbs = json_get_int(item, "max_load", 0);
            c->spaces = json_get_int(item, "spaces", 0);

            // Physics dimensions
            cJSON* physics = cJSON_GetObjectItem(item, "physics");
            if (physics) {
                cJSON* length = cJSON_GetObjectItem(physics, "length");
                cJSON* width = cJSON_GetObjectItem(physics, "width");
                cJSON* height = cJSON_GetObjectItem(physics, "height");

                if (length) {
                    c->length_default = json_get_float(length, "default", 4.0f);
                    c->length_min = json_get_float(length, "min", 3.5f);
                    c->length_max = json_get_float(length, "max", 5.0f);
                }
                if (width) {
                    c->width_default = json_get_float(width, "default", 1.8f);
                    c->width_min = json_get_float(width, "min", 1.5f);
                    c->width_max = json_get_float(width, "max", 2.2f);
                }
                if (height) {
                    c->height_default = json_get_float(height, "default", 1.4f);
                    c->height_min = json_get_float(height, "min", 1.2f);
                    c->height_max = json_get_float(height, "max", 1.8f);
                }

                // Center of mass offset [x, y, z]
                cJSON* com = cJSON_GetObjectItem(physics, "center_of_mass");
                if (com && cJSON_IsArray(com) && cJSON_GetArraySize(com) >= 3) {
                    c->center_of_mass[0] = (float)cJSON_GetArrayItem(com, 0)->valuedouble;
                    c->center_of_mass[1] = (float)cJSON_GetArrayItem(com, 1)->valuedouble;
                    c->center_of_mass[2] = (float)cJSON_GetArrayItem(com, 2)->valuedouble;
                } else {
                    // Default: centered, halfway down
                    c->center_of_mass[0] = 0.0f;
                    c->center_of_mass[1] = -0.5f;
                    c->center_of_mass[2] = 0.0f;
                }
            }
            g_equipment.chassis_count++;
        }
    }

    cJSON_Delete(root);
    printf("Equipment: Loaded %d chassis types\n", g_equipment.chassis_count);
    return true;
}

// Load power_plants.json
static bool load_power_plants(const char* filepath) {
    char* json_str = read_file(filepath);
    if (!json_str) return false;

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) { fprintf(stderr, "Equipment: JSON parse error in power_plants.json\n"); return false; }

    // Helper to parse power plant section
    #define PARSE_POWER_PLANTS(section_name) \
        cJSON* section_name##_section = cJSON_GetObjectItem(root, #section_name); \
        if (section_name##_section) { \
            cJSON* item; \
            cJSON_ArrayForEach(item, section_name##_section) { \
                if (g_equipment.power_plant_count >= MAX_EQUIPMENT_ITEMS) break; \
                PowerPlantEquipment* p = &g_equipment.power_plants[g_equipment.power_plant_count]; \
                json_get_string(item, "id", p->id, MAX_EQUIPMENT_ID, item->string); \
                json_get_string(item, "name", p->name, MAX_EQUIPMENT_NAME, "Unknown"); \
                json_get_string(item, "type", p->type, 32, "electric"); \
                p->cost = json_get_int(item, "cost", 0); \
                p->weight_lbs = json_get_int(item, "weight", 0); \
                p->spaces = json_get_int(item, "spaces", 0); \
                p->power_factors = json_get_int(item, "power_factors", 0); \
                p->power_units = json_get_int(item, "power_units", 0); \
                p->motor_force = json_get_float(item, "motor_force", 0); \
                if (p->motor_force <= 0) { \
                    p->motor_force = p->power_factors * POWER_FACTOR_TO_FORCE; \
                } \
                p->weight_kg = p->weight_lbs * LBS_TO_KG; \
                g_equipment.power_plant_count++; \
            } \
        }

    PARSE_POWER_PLANTS(electric)
    PARSE_POWER_PLANTS(cycle_electric)
    PARSE_POWER_PLANTS(gas_engines)

    #undef PARSE_POWER_PLANTS

    cJSON_Delete(root);
    printf("Equipment: Loaded %d power plants\n", g_equipment.power_plant_count);
    return true;
}

// Load tires.json
static bool load_tires(const char* filepath) {
    char* json_str = read_file(filepath);
    if (!json_str) return false;

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) { fprintf(stderr, "Equipment: JSON parse error in tires.json\n"); return false; }

    // Base types
    cJSON* base_types = cJSON_GetObjectItem(root, "base_types");
    if (base_types) {
        cJSON* item;
        cJSON_ArrayForEach(item, base_types) {
            if (g_equipment.tire_count >= MAX_EQUIPMENT_ITEMS) break;
            TireEquipment* t = &g_equipment.tires[g_equipment.tire_count];

            json_get_string(item, "id", t->id, MAX_EQUIPMENT_ID, item->string);
            json_get_string(item, "name", t->name, MAX_EQUIPMENT_NAME, "Unknown");
            t->cost = json_get_int(item, "cost", 50);
            t->weight_lbs = json_get_int(item, "weight", 30);
            t->dp = json_get_int(item, "dp", 4);

            cJSON* physics = cJSON_GetObjectItem(item, "physics");
            if (physics) {
                t->mu = json_get_float(physics, "mu", 1.5f);
            } else {
                t->mu = 1.5f;
            }
            g_equipment.tire_count++;
        }
    }

    // Modifiers
    cJSON* modifiers = cJSON_GetObjectItem(root, "modifiers");
    if (modifiers) {
        cJSON* item;
        cJSON_ArrayForEach(item, modifiers) {
            if (g_equipment.tire_modifier_count >= MAX_EQUIPMENT_ITEMS) break;
            TireModifier* m = &g_equipment.tire_modifiers[g_equipment.tire_modifier_count];

            json_get_string(item, "id", m->id, MAX_EQUIPMENT_ID, item->string);
            json_get_string(item, "name", m->name, MAX_EQUIPMENT_NAME, "Unknown");
            m->cost_modifier = json_get_float(item, "cost_modifier", 0.0f);
            m->weight_modifier = json_get_float(item, "weight_modifier", 0.0f);
            m->hc_bonus = json_get_int(item, "hc_bonus", 0);

            cJSON* physics_mod = cJSON_GetObjectItem(item, "physics_modifier");
            if (physics_mod) {
                m->mu_bonus = json_get_float(physics_mod, "mu_bonus", 0.0f);
            } else {
                m->mu_bonus = 0.0f;
            }
            g_equipment.tire_modifier_count++;
        }
    }

    cJSON_Delete(root);
    printf("Equipment: Loaded %d tire types, %d modifiers\n",
           g_equipment.tire_count, g_equipment.tire_modifier_count);
    return true;
}

// Load suspension.json
static bool load_suspension(const char* filepath) {
    char* json_str = read_file(filepath);
    if (!json_str) return false;

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) { fprintf(stderr, "Equipment: JSON parse error in suspension.json\n"); return false; }

    cJSON* types = cJSON_GetObjectItem(root, "types");
    if (types) {
        cJSON* item;
        cJSON_ArrayForEach(item, types) {
            if (g_equipment.suspension_count >= MAX_EQUIPMENT_ITEMS) break;
            SuspensionEquipment* s = &g_equipment.suspensions[g_equipment.suspension_count];

            json_get_string(item, "id", s->id, MAX_EQUIPMENT_ID, item->string);
            json_get_string(item, "name", s->name, MAX_EQUIPMENT_NAME, "Unknown");
            s->cost_modifier = json_get_float(item, "cost_modifier", 0.0f);
            s->hc_car = json_get_int(item, "hc_car", 1);
            s->hc_van = json_get_int(item, "hc_van", 0);

            cJSON* physics = cJSON_GetObjectItem(item, "physics");
            if (physics) {
                s->frequency = json_get_float(physics, "frequency", 1.5f);
                s->damping = json_get_float(physics, "damping", 0.5f);
                s->travel = json_get_float(physics, "travel", 0.12f);
            } else {
                s->frequency = 1.5f;
                s->damping = 0.5f;
                s->travel = 0.12f;
            }
            g_equipment.suspension_count++;
        }
    }

    cJSON_Delete(root);
    printf("Equipment: Loaded %d suspension types\n", g_equipment.suspension_count);
    return true;
}

// Load brakes.json
static bool load_brakes(const char* filepath) {
    char* json_str = read_file(filepath);
    if (!json_str) return false;

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) { fprintf(stderr, "Equipment: JSON parse error in brakes.json\n"); return false; }

    cJSON* types = cJSON_GetObjectItem(root, "types");
    if (types) {
        cJSON* item;
        cJSON_ArrayForEach(item, types) {
            if (g_equipment.brake_count >= MAX_EQUIPMENT_ITEMS) break;
            BrakeEquipment* b = &g_equipment.brakes[g_equipment.brake_count];

            json_get_string(item, "id", b->id, MAX_EQUIPMENT_ID, item->string);
            json_get_string(item, "name", b->name, MAX_EQUIPMENT_NAME, "Unknown");
            b->cost = json_get_int(item, "cost", 0);
            b->weight_lbs = json_get_int(item, "weight", 0);

            cJSON* physics = cJSON_GetObjectItem(item, "physics");
            if (physics) {
                b->brake_force_multiplier = json_get_float(physics, "brake_force_multiplier", 8.0f);
            } else {
                b->brake_force_multiplier = 8.0f;
            }
            g_equipment.brake_count++;
        }
    }

    cJSON_Delete(root);
    printf("Equipment: Loaded %d brake types\n", g_equipment.brake_count);
    return true;
}

// Load gearboxes.json
static bool load_gearboxes(const char* filepath) {
    char* json_str = read_file(filepath);
    if (!json_str) return false;

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) { fprintf(stderr, "Equipment: JSON parse error in gearboxes.json\n"); return false; }

    cJSON* types = cJSON_GetObjectItem(root, "types");
    if (types) {
        cJSON* item;
        cJSON_ArrayForEach(item, types) {
            if (g_equipment.gearbox_count >= MAX_EQUIPMENT_ITEMS) break;
            GearboxEquipment* g = &g_equipment.gearboxes[g_equipment.gearbox_count];

            json_get_string(item, "id", g->id, MAX_EQUIPMENT_ID, item->string);
            json_get_string(item, "name", g->name, MAX_EQUIPMENT_NAME, "Unknown");
            g->cost = json_get_int(item, "cost", 0);
            g->weight_lbs = json_get_int(item, "weight_lbs", 0);

            cJSON* physics = cJSON_GetObjectItem(item, "physics");
            if (physics) {
                // Parse gear ratios array
                cJSON* gear_ratios = cJSON_GetObjectItem(physics, "gear_ratios");
                g->gear_count = 0;
                if (gear_ratios && cJSON_IsArray(gear_ratios)) {
                    cJSON* ratio;
                    cJSON_ArrayForEach(ratio, gear_ratios) {
                        if (g->gear_count < MAX_GEARS && cJSON_IsNumber(ratio)) {
                            g->gear_ratios[g->gear_count++] = (float)ratio->valuedouble;
                        }
                    }
                }

                // Parse reverse ratios array
                cJSON* reverse_ratios = cJSON_GetObjectItem(physics, "reverse_ratios");
                g->reverse_count = 0;
                if (reverse_ratios && cJSON_IsArray(reverse_ratios)) {
                    cJSON* ratio;
                    cJSON_ArrayForEach(ratio, reverse_ratios) {
                        if (g->reverse_count < MAX_GEARS && cJSON_IsNumber(ratio)) {
                            g->reverse_ratios[g->reverse_count++] = (float)ratio->valuedouble;
                        }
                    }
                }

                g->differential_ratio = json_get_float(physics, "differential_ratio", 1.29f);
                g->shift_up_rpm = json_get_float(physics, "shift_up_rpm", 4500.0f);
                g->shift_down_rpm = json_get_float(physics, "shift_down_rpm", 2000.0f);
            } else {
                // Defaults
                g->gear_ratios[0] = 1.5f; g->gear_ratios[1] = 1.2f;
                g->gear_ratios[2] = 1.0f; g->gear_ratios[3] = 0.85f;
                g->gear_ratios[4] = 0.7f;
                g->gear_count = 5;
                g->reverse_ratios[0] = -1.5f;
                g->reverse_count = 1;
                g->differential_ratio = 1.29f;
                g->shift_up_rpm = 4500.0f;
                g->shift_down_rpm = 2000.0f;
            }
            g_equipment.gearbox_count++;
        }
    }

    cJSON_Delete(root);
    printf("Equipment: Loaded %d gearbox types\n", g_equipment.gearbox_count);
    return true;
}

// Load all equipment (can be called multiple times for reload)
bool equipment_load_all(const char* base_path) {
    // Reset and reload
    memset(&g_equipment, 0, sizeof(g_equipment));

    char path[512];
    bool success = true;

    snprintf(path, sizeof(path), "%s/chassis.json", base_path);
    if (!load_chassis(path)) success = false;

    snprintf(path, sizeof(path), "%s/power_plants.json", base_path);
    if (!load_power_plants(path)) success = false;

    snprintf(path, sizeof(path), "%s/tires.json", base_path);
    if (!load_tires(path)) success = false;

    snprintf(path, sizeof(path), "%s/suspension.json", base_path);
    if (!load_suspension(path)) success = false;

    snprintf(path, sizeof(path), "%s/brakes.json", base_path);
    if (!load_brakes(path)) success = false;

    snprintf(path, sizeof(path), "%s/gearboxes.json", base_path);
    if (!load_gearboxes(path)) success = false;

    g_equipment.loaded = success;
    return success;
}

// Lookup functions
const ChassisEquipment* equipment_find_chassis(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.chassis_count; i++) {
        if (strcmp(g_equipment.chassis[i].id, id) == 0) {
            return &g_equipment.chassis[i];
        }
    }
    return NULL;
}

const PowerPlantEquipment* equipment_find_power_plant(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.power_plant_count; i++) {
        if (strcmp(g_equipment.power_plants[i].id, id) == 0) {
            return &g_equipment.power_plants[i];
        }
    }
    return NULL;
}

const TireEquipment* equipment_find_tire(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.tire_count; i++) {
        if (strcmp(g_equipment.tires[i].id, id) == 0) {
            return &g_equipment.tires[i];
        }
    }
    return NULL;
}

const TireModifier* equipment_find_tire_modifier(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.tire_modifier_count; i++) {
        if (strcmp(g_equipment.tire_modifiers[i].id, id) == 0) {
            return &g_equipment.tire_modifiers[i];
        }
    }
    return NULL;
}

const SuspensionEquipment* equipment_find_suspension(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.suspension_count; i++) {
        if (strcmp(g_equipment.suspensions[i].id, id) == 0) {
            return &g_equipment.suspensions[i];
        }
    }
    return NULL;
}

const BrakeEquipment* equipment_find_brake(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.brake_count; i++) {
        if (strcmp(g_equipment.brakes[i].id, id) == 0) {
            return &g_equipment.brakes[i];
        }
    }
    return NULL;
}

const GearboxEquipment* equipment_find_gearbox(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.gearbox_count; i++) {
        if (strcmp(g_equipment.gearboxes[i].id, id) == 0) {
            return &g_equipment.gearboxes[i];
        }
    }
    return NULL;
}

// Calculate combined tire physics
TirePhysics equipment_calc_tire_physics(const char* tire_id, const char** modifier_ids, int modifier_count) {
    TirePhysics result = { 1.5f };  // default mu

    const TireEquipment* base = equipment_find_tire(tire_id);
    if (base) {
        result.mu = base->mu;
    }

    // Apply modifiers
    for (int i = 0; i < modifier_count; i++) {
        const TireModifier* mod = equipment_find_tire_modifier(modifier_ids[i]);
        if (mod) {
            result.mu += mod->mu_bonus;
        }
    }

    return result;
}

// Calculate top speed
float equipment_calc_top_speed_mph(const char* type, int power_factors, int total_weight_lbs) {
    if (total_weight_lbs <= 0 || power_factors <= 0) return 0.0f;

    float pf = (float)power_factors;
    float w = (float)total_weight_lbs;

    // Car Wars formulas
    if (strcmp(type, "electric") == 0 || strcmp(type, "nuclear") == 0) {
        return 360.0f * pf / (pf + w);
    } else {  // gas
        return 240.0f * pf / (pf + w);
    }
}

float equipment_calc_top_speed_ms(const char* type, int power_factors, int total_weight_lbs) {
    return equipment_calc_top_speed_mph(type, power_factors, total_weight_lbs) * MPH_TO_MS;
}

// Calculate acceleration class
int equipment_calc_accel_class(int power_factors, int total_weight_lbs) {
    if (total_weight_lbs <= 0 || power_factors <= 0) return 0;

    float ratio = (float)power_factors / (float)total_weight_lbs;

    // Car Wars acceleration tiers
    if (ratio >= 3.0f) return 25;       // 3x weight (nuclear only)
    if (ratio >= 2.0f) return 20;       // 2x weight (nuclear only)
    if (ratio >= 1.0f) return 15;       // >= weight
    if (ratio >= 0.5f) return 10;       // >= 1/2 weight
    if (ratio >= 0.333f) return 5;      // >= 1/3 weight
    return 0;                            // underpowered
}
