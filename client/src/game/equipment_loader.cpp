/*
 * Equipment Loader Implementation
 * Loads tabletop equipment JSON files and provides lookup
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
            c->base_hc_modifier = json_get_int(item, "base_hc_modifier", 0);

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

            // Model path (relative to assets)
            json_get_string(item, "model", c->model, 128, "");

            // Texture path (relative to assets) - optional default texture
            json_get_string(item, "texture", c->texture, 128, "");

            // Wheel mount positions
            c->wheel_mount_count = 0;
            cJSON* mounts = cJSON_GetObjectItem(item, "wheel_mounts");
            if (mounts && cJSON_IsArray(mounts)) {
                cJSON* mount;
                cJSON_ArrayForEach(mount, mounts) {
                    if (c->wheel_mount_count >= MAX_WHEEL_MOUNTS) break;
                    WheelMount* wm = &c->wheel_mounts[c->wheel_mount_count];

                    json_get_string(mount, "id", wm->id, 16, "W");

                    cJSON* pos = cJSON_GetObjectItem(mount, "position");
                    if (pos && cJSON_IsArray(pos) && cJSON_GetArraySize(pos) >= 3) {
                        wm->position[0] = (float)cJSON_GetArrayItem(pos, 0)->valuedouble;
                        wm->position[1] = (float)cJSON_GetArrayItem(pos, 1)->valuedouble;
                        wm->position[2] = (float)cJSON_GetArrayItem(pos, 2)->valuedouble;
                    }
                    c->wheel_mount_count++;
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
                p->power_units = json_get_int(item, "power_units", 0); \
                p->weight_kg = p->weight_lbs * LBS_TO_KG; \
                /* Real engine specs */ \
                p->horsepower = json_get_float(item, "horsepower", 100.0f); \
                p->torque_nm = json_get_float(item, "torque_nm", 150.0f); \
                p->peak_power_rpm = json_get_float(item, "peak_power_rpm", 6000.0f); \
                p->peak_torque_rpm = json_get_float(item, "peak_torque_rpm", 4000.0f); \
                p->redline_rpm = json_get_float(item, "redline_rpm", 7000.0f); \
                p->idle_rpm = 1000.0f; /* Default idle */ \
                /* Electric motors have instant torque (peak_torque_rpm = 0) */ \
                if (strcmp(p->type, "electric") == 0 || strcmp(p->type, "nuclear") == 0) { \
                    p->peak_torque_rpm = 0.0f; \
                    p->idle_rpm = 0.0f; \
                } \
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

            // Physics values
            cJSON* physics = cJSON_GetObjectItem(item, "physics");
            if (physics) {
                // Gear ratios
                cJSON* gear_ratios = cJSON_GetObjectItem(physics, "gear_ratios");
                if (gear_ratios && cJSON_IsArray(gear_ratios)) {
                    g->gear_count = 0;
                    cJSON* ratio;
                    cJSON_ArrayForEach(ratio, gear_ratios) {
                        if (g->gear_count >= MAX_GEARS) break;
                        g->gear_ratios[g->gear_count++] = (float)ratio->valuedouble;
                    }
                }

                // Reverse gear ratios
                cJSON* reverse_ratios = cJSON_GetObjectItem(physics, "reverse_ratios");
                if (reverse_ratios && cJSON_IsArray(reverse_ratios)) {
                    g->reverse_count = 0;
                    cJSON* ratio;
                    cJSON_ArrayForEach(ratio, reverse_ratios) {
                        if (g->reverse_count >= MAX_GEARS) break;
                        g->reverse_ratios[g->reverse_count++] = (float)ratio->valuedouble;
                    }
                }

                g->differential_ratio = json_get_float(physics, "differential_ratio", 1.0f);
                g->shift_up_rpm = json_get_float(physics, "shift_up_rpm", 4500.0f);
                g->shift_down_rpm = json_get_float(physics, "shift_down_rpm", 2000.0f);
                g->clutch_strength = json_get_float(physics, "clutch_strength", 10.0f);
                g->switch_time = json_get_float(physics, "switch_time", 0.5f);
                g->switch_latency = json_get_float(physics, "switch_latency", 0.5f);
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

    snprintf(path, sizeof(path), "%s/gearboxes.json", base_path);
    if (!load_gearboxes(path)) success = false;

    snprintf(path, sizeof(path), "%s/tires.json", base_path);
    if (!load_tires(path)) success = false;

    snprintf(path, sizeof(path), "%s/suspension.json", base_path);
    if (!load_suspension(path)) success = false;

    snprintf(path, sizeof(path), "%s/brakes.json", base_path);
    if (!load_brakes(path)) success = false;

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

const GearboxEquipment* equipment_find_gearbox(const char* id) {
    if (!id) return NULL;
    for (int i = 0; i < g_equipment.gearbox_count; i++) {
        if (strcmp(g_equipment.gearboxes[i].id, id) == 0) {
            return &g_equipment.gearboxes[i];
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

