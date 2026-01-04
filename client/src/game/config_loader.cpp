 /*
 * JSON Configuration Loader Implementation
 */

#include "config_loader.h"
#include "equipment_loader.h"
#include "handling.h"
#include "../vendor/cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#define getcwd _getcwd
#else
#include <unistd.h>
#endif

// Read entire file into memory
static char* read_file(const char* filepath) {
    FILE* f = fopen(filepath, "rb");
    if (!f) {
        // Print current working directory for debugging
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            fprintf(stderr, "CWD: %s\n", cwd);
        }
        fprintf(stderr, "Failed to open: %s\n", filepath);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* buffer = (char*)malloc(size + 1);
    if (!buffer) {
        fclose(f);
        return NULL;
    }

    fread(buffer, 1, size, f);
    buffer[size] = '\0';
    fclose(f);
    return buffer;
}

// Helper: get float from JSON object
static float json_get_float(cJSON* obj, const char* key, float def) {
    cJSON* item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsNumber(item)) {
        return (float)item->valuedouble;
    }
    return def;
}

// Helper: get string from JSON object
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

// Helper: get Vec3 from JSON array [x, y, z]
static Vec3 json_get_vec3(cJSON* obj, const char* key, Vec3 def) {
    cJSON* arr = cJSON_GetObjectItem(obj, key);
    if (arr && cJSON_IsArray(arr) && cJSON_GetArraySize(arr) >= 3) {
        return vec3(
            (float)cJSON_GetArrayItem(arr, 0)->valuedouble,
            (float)cJSON_GetArrayItem(arr, 1)->valuedouble,
            (float)cJSON_GetArrayItem(arr, 2)->valuedouble
        );
    }
    return def;
}

// Helper: parse vehicle class from string
static VehicleClass parse_vehicle_class(const char* str) {
    if (!str) return VEHICLE_CLASS_WHEELED;
    if (strcmp(str, "wheeled") == 0) return VEHICLE_CLASS_WHEELED;
    if (strcmp(str, "tracked") == 0) return VEHICLE_CLASS_TRACKED;
    if (strcmp(str, "hover") == 0) return VEHICLE_CLASS_HOVER;
    if (strcmp(str, "aircraft") == 0) return VEHICLE_CLASS_AIRCRAFT;
    if (strcmp(str, "boat") == 0) return VEHICLE_CLASS_BOAT;
    return VEHICLE_CLASS_UNKNOWN;
}

// Helper: map wheel ID to index (FL=0, FR=1, RL=2, RR=3)
// Returns -1 if not a standard 4-wheel ID
static int wheel_id_to_index(const char* id) {
    if (!id) return -1;
    if (strcmp(id, "FL") == 0) return 0;  // Front Left
    if (strcmp(id, "FR") == 0) return 1;  // Front Right
    if (strcmp(id, "RL") == 0) return 2;  // Rear Left
    if (strcmp(id, "RR") == 0) return 3;  // Rear Right
    return -1;
}

// Helper: get bool from JSON
static bool json_get_bool(cJSON* obj, const char* key, bool def) {
    cJSON* item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsBool(item)) {
        return cJSON_IsTrue(item);
    }
    return def;
}

// Helper: parse Vec3 from array directly
static Vec3 json_array_to_vec3(cJSON* arr, Vec3 def) {
    if (arr && cJSON_IsArray(arr) && cJSON_GetArraySize(arr) >= 3) {
        return vec3(
            (float)cJSON_GetArrayItem(arr, 0)->valuedouble,
            (float)cJSON_GetArrayItem(arr, 1)->valuedouble,
            (float)cJSON_GetArrayItem(arr, 2)->valuedouble
        );
    }
    return def;
}

VehicleJSON config_default_vehicle(void) {
    // Return zeroed struct - caller must validate all required fields are set
    VehicleJSON v = {};
    memset(&v, 0, sizeof(v));
    v.vehicle_class = VEHICLE_CLASS_UNKNOWN;  // Marker for uninitialized
    return v;
}

// Validate that vehicle config has all required fields for physics
static bool validate_vehicle_config(const VehicleJSON* v, const char* filepath) {
    bool valid = true;

    // Check chassis mass (required)
    if (v->chassis_mass <= 0.0f) {
        fprintf(stderr, "ERROR [%s]: Missing or invalid chassis mass\n", filepath);
        valid = false;
    }
    // Note: chassis dimensions can be 0 if derived from OBJ bounds at runtime

    // Check wheels
    if (v->wheel_count < 4) {
        fprintf(stderr, "ERROR [%s]: Need at least 4 wheels, found %d\n", filepath, v->wheel_count);
        valid = false;
    }

    // Check that standard wheel IDs are present (FL, FR, RL, RR)
    bool has_fl = false, has_fr = false, has_rl = false, has_rr = false;
    for (int i = 0; i < v->wheel_count; i++) {
        if (strcmp(v->wheels[i].id, "FL") == 0) has_fl = true;
        if (strcmp(v->wheels[i].id, "FR") == 0) has_fr = true;
        if (strcmp(v->wheels[i].id, "RL") == 0) has_rl = true;
        if (strcmp(v->wheels[i].id, "RR") == 0) has_rr = true;

        // Check each wheel has valid dimensions
        if (v->wheels[i].radius <= 0.0f) {
            fprintf(stderr, "ERROR [%s]: Wheel %s has invalid radius %.2f\n",
                    filepath, v->wheels[i].id, v->wheels[i].radius);
            valid = false;
        }
        if (v->wheels[i].mass <= 0.0f) {
            fprintf(stderr, "ERROR [%s]: Wheel %s has invalid mass %.2f\n",
                    filepath, v->wheels[i].id, v->wheels[i].mass);
            valid = false;
        }
    }
    if (!has_fl || !has_fr || !has_rl || !has_rr) {
        fprintf(stderr, "ERROR [%s]: Missing standard wheel IDs (FL=%d FR=%d RL=%d RR=%d)\n",
                filepath, has_fl, has_fr, has_rl, has_rr);
        valid = false;
    }

    // Check axles
    if (v->axle_count < 1) {
        fprintf(stderr, "ERROR [%s]: Need at least 1 axle, found %d\n", filepath, v->axle_count);
        valid = false;
    }

    // Check tire friction
    if (v->defaults.wheel.friction <= 0.0f) {
        fprintf(stderr, "ERROR [%s]: Missing or invalid tire friction (mu=%.2f)\n",
                filepath, v->defaults.wheel.friction);
        valid = false;
    }

    // Check suspension - either from defaults or from all axles
    bool has_suspension = false;
    if (v->defaults.suspension.frequency > 0.0f) {
        has_suspension = true;
    } else {
        // Check if all axles have their own suspension
        has_suspension = true;
        for (int i = 0; i < v->axle_count; i++) {
            if (!v->axles[i].has_suspension) {
                has_suspension = false;
                break;
            }
        }
    }
    if (!has_suspension) {
        fprintf(stderr, "ERROR [%s]: Missing suspension - need defaults or per-axle suspension\n", filepath);
        fprintf(stderr, "       defaults.suspension.frequency=%.2f damping=%.2f\n",
                v->defaults.suspension.frequency, v->defaults.suspension.damping);
        for (int i = 0; i < v->axle_count; i++) {
            fprintf(stderr, "       axle[%d] '%s': has_suspension=%d\n",
                    i, v->axles[i].name, v->axles[i].has_suspension);
        }
        valid = false;
    }

    // Check matchbox physics values
    if (v->physics.accel_force <= 0.0f) {
        fprintf(stderr, "ERROR [%s]: Missing or invalid acceleration force (%.0fN)\n",
                filepath, v->physics.accel_force);
        valid = false;
    }

    return valid;
}

// Parsed tire config (from vehicle JSON tires array)
typedef struct {
    char id[32];
    char type[64];
    char modifiers[4][64];
    int modifier_count;
    float radius;
    float width;
    // Which wheel IDs this tire applies to (new format)
    char wheel_ids[MAX_WHEEL_MOUNTS][16];
    int wheel_count;
} ParsedTireConfig;

static ParsedTireConfig s_parsed_tires[16];
static int s_parsed_tire_count = 0;

// Parse tires array
static void parse_tires_array(cJSON* tires_arr) {
    s_parsed_tire_count = 0;
    cJSON* t;
    cJSON_ArrayForEach(t, tires_arr) {
        if (s_parsed_tire_count >= 16) break;
        ParsedTireConfig* tire = &s_parsed_tires[s_parsed_tire_count];

        json_get_string(t, "id", tire->id, 32, "");
        json_get_string(t, "type", tire->type, 64, "tire_standard");

        // Parse modifiers array
        tire->modifier_count = 0;
        cJSON* mods = cJSON_GetObjectItem(t, "modifiers");
        if (mods && cJSON_IsArray(mods)) {
            cJSON* m;
            cJSON_ArrayForEach(m, mods) {
                if (tire->modifier_count >= 4) break;
                if (cJSON_IsString(m)) {
                    strncpy(tire->modifiers[tire->modifier_count], m->valuestring, 63);
                    tire->modifiers[tire->modifier_count][63] = '\0';
                    tire->modifier_count++;
                }
            }
        }

        // Parse size
        cJSON* size = cJSON_GetObjectItem(t, "size");
        if (size) {
            tire->radius = json_get_float(size, "radius", 0.35f);
            tire->width = json_get_float(size, "width", 0.2f);
        } else {
            tire->radius = 0.35f;
            tire->width = 0.2f;
        }

        // Parse wheels array (which wheel positions this tire applies to)
        tire->wheel_count = 0;
        cJSON* wheels_arr = cJSON_GetObjectItem(t, "wheels");
        if (wheels_arr && cJSON_IsArray(wheels_arr)) {
            cJSON* wid;
            cJSON_ArrayForEach(wid, wheels_arr) {
                if (tire->wheel_count >= MAX_WHEEL_MOUNTS) break;
                if (cJSON_IsString(wid)) {
                    strncpy(tire->wheel_ids[tire->wheel_count], wid->valuestring, 15);
                    tire->wheel_ids[tire->wheel_count][15] = '\0';
                    tire->wheel_count++;
                }
            }
        }

        s_parsed_tire_count++;
    }
}

// Find parsed tire by tire ID (e.g., "front_tires")
static ParsedTireConfig* find_parsed_tire(const char* id) {
    for (int i = 0; i < s_parsed_tire_count; i++) {
        if (strcmp(s_parsed_tires[i].id, id) == 0) {
            return &s_parsed_tires[i];
        }
    }
    return NULL;
}

// Find parsed tire that applies to a given wheel ID (e.g., "FL")
static ParsedTireConfig* find_tire_for_wheel(const char* wheel_id) {
    for (int i = 0; i < s_parsed_tire_count; i++) {
        for (int w = 0; w < s_parsed_tires[i].wheel_count; w++) {
            if (strcmp(s_parsed_tires[i].wheel_ids[w], wheel_id) == 0) {
                return &s_parsed_tires[i];
            }
        }
    }
    return NULL;
}

// Parse wheels array (uses defaults.wheel for values not specified)
// Parse wheel side from string
static WheelSide parse_wheel_side(const char* str) {
    if (!str) return WHEEL_SIDE_UNKNOWN;
    if (strcmp(str, "left") == 0 || strcmp(str, "L") == 0) return WHEEL_SIDE_LEFT;
    if (strcmp(str, "right") == 0 || strcmp(str, "R") == 0) return WHEEL_SIDE_RIGHT;
    if (strcmp(str, "center") == 0 || strcmp(str, "C") == 0) return WHEEL_SIDE_CENTER;
    return WHEEL_SIDE_UNKNOWN;
}

// Infer wheel side from Z position (positive Z = left, negative Z = right)
static WheelSide infer_wheel_side(float z_pos) {
    const float threshold = 0.1f;  // 10cm tolerance for center wheels
    if (z_pos > threshold) return WHEEL_SIDE_LEFT;
    if (z_pos < -threshold) return WHEEL_SIDE_RIGHT;
    return WHEEL_SIDE_CENTER;
}

static void parse_wheels(cJSON* wheels_arr, VehicleJSON* out) {
    out->wheel_count = 0;
    cJSON* w;
    cJSON_ArrayForEach(w, wheels_arr) {
        if (out->wheel_count >= MAX_WHEELS) break;

        WheelDef* wheel = &out->wheels[out->wheel_count];
        json_get_string(w, "id", wheel->id, 16, "W");
        wheel->position = json_array_to_vec3(cJSON_GetObjectItem(w, "position"), vec3(0, 0, 0));

        // Initialize semantic fields
        wheel->axle_index = -1;  // Will be populated after axle parsing

        // Parse or infer wheel side
        cJSON* side_item = cJSON_GetObjectItem(w, "side");
        if (side_item && cJSON_IsString(side_item)) {
            wheel->side = parse_wheel_side(side_item->valuestring);
        } else {
            // Infer from Z position (JSON coords: positive Z = left)
            wheel->side = infer_wheel_side(wheel->position.z);
        }

        // Check for tire reference
        cJSON* tire_ref = cJSON_GetObjectItem(w, "tire");
        if (tire_ref && cJSON_IsString(tire_ref)) {
            ParsedTireConfig* tire = find_parsed_tire(tire_ref->valuestring);
            if (tire) {
                wheel->radius = tire->radius;
                wheel->width = tire->width;
            } else {
                wheel->radius = out->defaults.wheel.radius;
                wheel->width = out->defaults.wheel.width;
            }
        } else {
            // Use defaults or direct values
            wheel->radius = json_get_float(w, "radius", out->defaults.wheel.radius);
            wheel->width = json_get_float(w, "width", out->defaults.wheel.width);
        }
        // Get mass from JSON, defaults, or estimate from dimensions
        float default_mass = out->defaults.wheel.mass;
        if (default_mass <= 0.0f && wheel->radius > 0.0f && wheel->width > 0.0f) {
            default_mass = 15.0f * (wheel->radius / 0.35f) * (wheel->width / 0.2f);
        }
        wheel->mass = json_get_float(w, "mass", default_mass);
        out->wheel_count++;
    }
}

// Parse axle position from string
static AxlePosition parse_axle_position(const char* str) {
    if (!str) return AXLE_POSITION_UNKNOWN;
    if (strcmp(str, "front") == 0) return AXLE_POSITION_FRONT;
    if (strcmp(str, "rear") == 0) return AXLE_POSITION_REAR;
    if (strcmp(str, "middle") == 0) return AXLE_POSITION_MIDDLE;
    return AXLE_POSITION_UNKNOWN;
}

// Infer axle position from name (if not explicitly specified)
static AxlePosition infer_axle_position(const char* name, int axle_index, int total_axles) {
    // Try to infer from common naming conventions
    if (name) {
        if (strstr(name, "front") != NULL || strcmp(name, "f") == 0) return AXLE_POSITION_FRONT;
        if (strstr(name, "rear") != NULL || strcmp(name, "r") == 0) return AXLE_POSITION_REAR;
        if (strstr(name, "mid") != NULL) return AXLE_POSITION_MIDDLE;
    }
    // Fallback: first axle = front, last = rear, others = middle
    if (axle_index == 0) return AXLE_POSITION_FRONT;
    if (axle_index == total_axles - 1) return AXLE_POSITION_REAR;
    return AXLE_POSITION_MIDDLE;
}

// Parse axles array (with per-axle suspension/brakes support)
static void parse_axles(cJSON* axles_arr, VehicleJSON* out, int* hc_suspension_out) {
    out->axle_count = 0;
    int best_suspension_hc = 0;  // Track best suspension HC found

    // First pass: count axles
    int total_axles = 0;
    cJSON* a_count;
    cJSON_ArrayForEach(a_count, axles_arr) { total_axles++; }

    cJSON* a;
    cJSON_ArrayForEach(a, axles_arr) {
        if (out->axle_count >= MAX_AXLES) break;

        AxleDef* axle = &out->axles[out->axle_count];
        // Try "id" first, then "name" for backwards compatibility
        cJSON* id_item = cJSON_GetObjectItem(a, "id");
        if (id_item && cJSON_IsString(id_item)) {
            strncpy(axle->name, id_item->valuestring, MAX_NAME_LENGTH - 1);
            axle->name[MAX_NAME_LENGTH - 1] = '\0';
        } else {
            json_get_string(a, "name", axle->name, MAX_NAME_LENGTH, "axle");
        }
        axle->steering = json_get_bool(a, "steering", false);
        axle->driven = json_get_bool(a, "driven", false);
        axle->max_steer_angle = json_get_float(a, "max_steer_angle", 0.6f);

        // Parse semantic fields: position and handbrake
        cJSON* pos_item = cJSON_GetObjectItem(a, "position");
        if (pos_item && cJSON_IsString(pos_item)) {
            axle->position = parse_axle_position(pos_item->valuestring);
        } else {
            axle->position = infer_axle_position(axle->name, out->axle_count, total_axles);
        }

        // Handbrake: explicit or default to rear axle
        cJSON* handbrake_item = cJSON_GetObjectItem(a, "handbrake");
        if (handbrake_item && cJSON_IsBool(handbrake_item)) {
            axle->has_handbrake = cJSON_IsTrue(handbrake_item);
        } else {
            // Default: handbrake on rear axle
            axle->has_handbrake = (axle->position == AXLE_POSITION_REAR);
        }

        // Parse per-axle suspension (can be string ID or object)
        cJSON* axle_susp = cJSON_GetObjectItem(a, "suspension");
        if (axle_susp && cJSON_IsString(axle_susp)) {
            // New format: suspension is an equipment ID string
            const SuspensionEquipment* susp = equipment_find_suspension(axle_susp->valuestring);
            if (susp) {
                axle->has_suspension = true;
                axle->suspension_frequency = susp->frequency;
                axle->suspension_damping = susp->damping;
                axle->suspension_travel = susp->travel;
                // Track best suspension HC (use hc_car for now, could check vehicle type later)
                if (susp->hc_car > best_suspension_hc) {
                    best_suspension_hc = susp->hc_car;
                }
            } else {
                fprintf(stderr, "Warning: Suspension '%s' not found, using defaults\n", axle_susp->valuestring);
                axle->has_suspension = false;
            }
        } else if (axle_susp && cJSON_IsObject(axle_susp)) {
            // Legacy format: suspension is an object with values
            axle->has_suspension = true;
            axle->suspension_frequency = json_get_float(axle_susp, "frequency", out->defaults.suspension.frequency);
            axle->suspension_damping = json_get_float(axle_susp, "damping", out->defaults.suspension.damping);
            axle->suspension_travel = json_get_float(axle_susp, "travel", out->defaults.suspension.travel);
        } else {
            axle->has_suspension = false;
        }

        // Parse per-axle brakes (string ID)
        cJSON* axle_brakes = cJSON_GetObjectItem(a, "brakes");
        if (axle_brakes && cJSON_IsString(axle_brakes)) {
            const BrakeEquipment* brk = equipment_find_brake(axle_brakes->valuestring);
            if (brk) {
                axle->brake_force_multiplier = brk->brake_force_multiplier;
            } else {
                fprintf(stderr, "Warning: Brakes '%s' not found, using default\n", axle_brakes->valuestring);
                axle->brake_force_multiplier = 8.0f;
            }
        } else {
            axle->brake_force_multiplier = 8.0f;  // Default standard brakes
        }

        // Parse wheel IDs array
        axle->wheel_count = 0;
        cJSON* wheel_ids = cJSON_GetObjectItem(a, "wheels");
        if (wheel_ids && cJSON_IsArray(wheel_ids)) {
            cJSON* wid;
            cJSON_ArrayForEach(wid, wheel_ids) {
                if (axle->wheel_count >= MAX_WHEELS_PER_AXLE) break;
                if (cJSON_IsString(wid)) {
                    strncpy(axle->wheel_ids[axle->wheel_count], wid->valuestring, 15);
                    axle->wheel_ids[axle->wheel_count][15] = '\0';
                    axle->wheel_count++;
                }
            }
        }
        out->axle_count++;
    }

    // Store the best suspension HC found
    if (hc_suspension_out) {
        *hc_suspension_out = best_suspension_hc;
    }
}

bool config_load_vehicle(const char* filepath, VehicleJSON* out) {
    *out = config_default_vehicle();

    // Track HC components for final calculation
    int hc_chassis = 0;      // From chassis type (e.g., subcompact +1, van -1)
    int hc_suspension = 0;   // From suspension type (e.g., improved = 2)
    int hc_tires = 0;        // From tire modifiers (e.g., radials +1)

    char* json_str = read_file(filepath);
    if (!json_str) {
        fprintf(stderr, "ERROR: Vehicle config required but not found: %s\n", filepath);
        fprintf(stderr, "       Game will use defaults but physics may be wrong!\n");
        return false;
    }

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);

    if (!root) {
        fprintf(stderr, "JSON parse error in %s\n", filepath);
        return false;
    }

    // Meta info
    char class_str[MAX_NAME_LENGTH];
    json_get_string(root, "class", class_str, MAX_NAME_LENGTH, "wheeled");
    out->vehicle_class = parse_vehicle_class(class_str);

    cJSON* version_item = cJSON_GetObjectItem(root, "version");
    if (version_item && cJSON_IsNumber(version_item)) {
        out->version = version_item->valueint;
    }

    json_get_string(root, "name", out->name, MAX_NAME_LENGTH, "default");

    // Texture - can be overridden at vehicle level
    json_get_string(root, "texture", out->texture, 128, "");

    // Chassis - can be string ID or object with body reference
    const ChassisEquipment* chassis_equip = NULL;
    cJSON* chassis = cJSON_GetObjectItem(root, "chassis");
    if (chassis) {
        const char* chassis_id = NULL;

        if (cJSON_IsString(chassis)) {
            // New format: direct string reference e.g., "chassis": "compact"
            chassis_id = chassis->valuestring;
        } else if (cJSON_IsObject(chassis)) {
            // Legacy format: object with body field e.g., "chassis": { "body": "compact" }
            cJSON* body_ref = cJSON_GetObjectItem(chassis, "body");
            if (body_ref && cJSON_IsString(body_ref)) {
                chassis_id = body_ref->valuestring;
            }
        }

        if (chassis_id) {
            chassis_equip = equipment_find_chassis(chassis_id);
            if (chassis_equip) {
                // Start with defaults from chassis equipment
                out->chassis_length = chassis_equip->length_default;
                out->chassis_width = chassis_equip->width_default;
                out->chassis_height = chassis_equip->height_default;
                // Calculate mass from weight (tabletop lbs -> kg)
                out->chassis_mass = chassis_equip->weight_lbs * LBS_TO_KG;
                // Copy center of mass from chassis equipment
                out->physics.center_of_mass = vec3(
                    chassis_equip->center_of_mass[0],
                    chassis_equip->center_of_mass[1],
                    chassis_equip->center_of_mass[2]
                );

                // Store chassis HC modifier
                hc_chassis = chassis_equip->base_hc_modifier;

                printf("  Chassis: %s (%.1fm x %.1fm x %.1fm, %.0fkg, CoM=[%.2f,%.2f,%.2f], HC%+d)\n",
                       chassis_equip->name, out->chassis_length, out->chassis_width,
                       out->chassis_height, out->chassis_mass,
                       out->physics.center_of_mass.x, out->physics.center_of_mass.y,
                       out->physics.center_of_mass.z, hc_chassis);
            } else {
                fprintf(stderr, "Warning: Chassis '%s' not found, using defaults\n", chassis_id);
            }
        } else if (cJSON_IsObject(chassis)) {
            // Legacy format: direct dimension values in object
            out->chassis_mass = json_get_float(chassis, "mass", out->chassis_mass);
            out->chassis_length = json_get_float(chassis, "length", out->chassis_length);
            out->chassis_width = json_get_float(chassis, "width", out->chassis_width);
            out->chassis_height = json_get_float(chassis, "height", out->chassis_height);
        }
        // Update legacy physics struct
        out->physics.chassis_mass = out->chassis_mass;
        out->physics.chassis_length = out->chassis_length;
        out->physics.chassis_width = out->chassis_width;
        out->physics.chassis_height = out->chassis_height;
    }

    // Apply texture fallback: if vehicle doesn't specify texture, use chassis default
    if (out->texture[0] == '\0' && chassis_equip && chassis_equip->texture[0] != '\0') {
        strncpy(out->texture, chassis_equip->texture, 127);
        out->texture[127] = '\0';
    }
    if (out->texture[0] != '\0') {
        printf("  Texture: %s\n", out->texture);
    }

    // Parse defaults block
    cJSON* defaults = cJSON_GetObjectItem(root, "defaults");
    if (defaults && cJSON_IsObject(defaults)) {
        // Parse defaults.wheel
        cJSON* def_wheel = cJSON_GetObjectItem(defaults, "wheel");
        if (def_wheel && cJSON_IsObject(def_wheel)) {
            out->defaults.wheel.radius = json_get_float(def_wheel, "radius", out->defaults.wheel.radius);
            out->defaults.wheel.width = json_get_float(def_wheel, "width", out->defaults.wheel.width);
            out->defaults.wheel.mass = json_get_float(def_wheel, "mass", out->defaults.wheel.mass);
            out->defaults.wheel.friction = json_get_float(def_wheel, "friction", out->defaults.wheel.friction);
        }
        // Parse defaults.suspension (Jolt style: frequency/damping)
        cJSON* def_susp = cJSON_GetObjectItem(defaults, "suspension");
        if (def_susp && cJSON_IsObject(def_susp)) {
            out->defaults.suspension.frequency = json_get_float(def_susp, "frequency", out->defaults.suspension.frequency);
            out->defaults.suspension.damping = json_get_float(def_susp, "damping", out->defaults.suspension.damping);
            out->defaults.suspension.travel = json_get_float(def_susp, "travel", out->defaults.suspension.travel);
        }
    }

    // Parse tires array first (needed for wheel parsing)
    cJSON* tires = cJSON_GetObjectItem(root, "tires");
    if (tires && cJSON_IsArray(tires)) {
        parse_tires_array(tires);
    }

    // Build wheels: new format uses chassis wheel_mounts + tires, legacy uses wheels array
    cJSON* wheels = cJSON_GetObjectItem(root, "wheels");
    if (wheels && cJSON_IsArray(wheels)) {
        // Legacy format: parse wheels array directly
        parse_wheels(wheels, out);
    } else if (chassis_equip && chassis_equip->wheel_mount_count > 0) {
        // New format: build wheels from chassis wheel_mounts + tire info
        out->wheel_count = 0;
        for (int i = 0; i < chassis_equip->wheel_mount_count && i < MAX_WHEELS; i++) {
            const WheelMount* mount = &chassis_equip->wheel_mounts[i];
            WheelDef* wheel = &out->wheels[out->wheel_count];

            // Copy ID and position from chassis wheel mount
            strncpy(wheel->id, mount->id, 15);
            wheel->id[15] = '\0';
            wheel->position = vec3(mount->position[0], mount->position[1], mount->position[2]);

            // Initialize semantic fields
            wheel->axle_index = -1;

            // Infer wheel side from Z position (positive Z = left)
            wheel->side = infer_wheel_side(wheel->position.z);

            // Get tire info for this wheel
            ParsedTireConfig* tire = find_tire_for_wheel(mount->id);
            if (tire) {
                wheel->radius = tire->radius;
                wheel->width = tire->width;
            } else {
                // Fallback to defaults
                wheel->radius = out->defaults.wheel.radius > 0 ? out->defaults.wheel.radius : 0.35f;
                wheel->width = out->defaults.wheel.width > 0 ? out->defaults.wheel.width : 0.2f;
            }

            // Calculate mass from dimensions or use default
            float default_mass = out->defaults.wheel.mass;
            if (default_mass <= 0.0f && wheel->radius > 0.0f && wheel->width > 0.0f) {
                default_mass = 15.0f * (wheel->radius / 0.35f) * (wheel->width / 0.2f);
            }
            wheel->mass = default_mass > 0.0f ? default_mass : 15.0f;

            out->wheel_count++;
        }
    }

    // Debug: print wheel positions
    printf("  Wheels: %d parsed\n", out->wheel_count);
    for (int i = 0; i < out->wheel_count; i++) {
        printf("    %s: (%.2f, %.2f, %.2f) r=%.2f m=%.1fkg\n",
               out->wheels[i].id,
               out->wheels[i].position.x,
               out->wheels[i].position.y,
               out->wheels[i].position.z,
               out->wheels[i].radius,
               out->wheels[i].mass);
    }

    // Parse axles array
    cJSON* axles = cJSON_GetObjectItem(root, "axles");
    if (axles && cJSON_IsArray(axles)) {
        parse_axles(axles, out, &hc_suspension);
    }

    // Link wheels to their axles (set axle_index for each wheel)
    for (int a = 0; a < out->axle_count; a++) {
        for (int aw = 0; aw < out->axles[a].wheel_count; aw++) {
            const char* wheel_id = out->axles[a].wheel_ids[aw];
            // Find this wheel in the wheels array
            for (int w = 0; w < out->wheel_count; w++) {
                if (strcmp(out->wheels[w].id, wheel_id) == 0) {
                    out->wheels[w].axle_index = a;
                    break;
                }
            }
        }
    }

    // Calculate tire friction from first tire's equipment
    if (s_parsed_tire_count > 0) {
        ParsedTireConfig* first_tire = &s_parsed_tires[0];
        const char* mod_ids[4];
        for (int i = 0; i < first_tire->modifier_count; i++) {
            mod_ids[i] = first_tire->modifiers[i];
        }
        TirePhysics tp = equipment_calc_tire_physics(first_tire->type, mod_ids, first_tire->modifier_count);
        out->defaults.wheel.friction = tp.mu;

        // Add tire weight to total mass
        const TireEquipment* tire_equip = equipment_find_tire(first_tire->type);
        if (tire_equip) {
            float tire_weight_kg = tire_equip->weight_lbs * LBS_TO_KG * out->wheel_count;
            out->chassis_mass += tire_weight_kg;
            out->physics.chassis_mass = out->chassis_mass;
            printf("  Tires: %s x%d (mu=%.2f, %.0fkg total)\n",
                   first_tire->type, out->wheel_count, tp.mu, tire_weight_kg);
        } else {
            printf("  Tires: %s (mu=%.2f)\n", first_tire->type, tp.mu);
        }
    }

    // ========== MATCHBOX CAR PHYSICS - Power Plant ==========
    // Read power_plant from drivetrain section (preferred) or top level (legacy)
    cJSON* drivetrain_section = cJSON_GetObjectItem(root, "drivetrain");
    cJSON* pp_ref = NULL;
    if (drivetrain_section) {
        pp_ref = cJSON_GetObjectItem(drivetrain_section, "power_plant");
    }
    if (!pp_ref) {
        // Fallback to top-level for backwards compatibility
        pp_ref = cJSON_GetObjectItem(root, "power_plant");
    }
    if (pp_ref && cJSON_IsString(pp_ref)) {
        const PowerPlantEquipment* pp = equipment_find_power_plant(pp_ref->valuestring);
        if (pp) {
            // Add power plant weight to chassis mass
            out->chassis_mass += pp->weight_kg;
            out->physics.chassis_mass = out->chassis_mass;

            strncpy(out->physics.vehicle_name, out->name, 63);
            out->physics.vehicle_name[63] = '\0';

            // Store engine specs for Jolt drivetrain
            out->physics.engine_max_torque = pp->torque_nm;
            out->physics.engine_idle_rpm = pp->idle_rpm;
            out->physics.engine_max_rpm = pp->redline_rpm;

            // Estimate acceleration from torque (rough estimate using avg gear ratio)
            // Real acceleration depends on current gear - Jolt handles this
            float avg_gear_ratio = 1.5f;  // Approximate mid-range gear
            float diff_ratio = 3.42f;     // Will be overridden by gearbox
            float wheel_radius = 0.32f;   // Typical
            float wheel_force = (pp->torque_nm * avg_gear_ratio * diff_ratio) / wheel_radius;
            out->physics.accel_force = wheel_force;

            float accel_ms2 = wheel_force / out->chassis_mass;
            out->physics.target_accel_ms2 = accel_ms2;
            out->physics.target_0_60_seconds = 26.82f / accel_ms2;
            out->physics.top_speed_ms = 0.0f;  // Drivetrain simulation handles this

            float weight_lbs = out->chassis_mass / LBS_TO_KG;
            printf("  Power Plant: %s (%.0f HP, %.0f Nm @ %.0f RPM)\n",
                   pp->name, pp->horsepower, pp->torque_nm, pp->peak_torque_rpm);
            printf("  Total Weight: %.0f kg (%.0f lbs), HP/ton: %.0f\n",
                   out->chassis_mass, weight_lbs, pp->horsepower / (out->chassis_mass / 1000.0f));
            printf("  Redline: %.0f RPM, Idle: %.0f RPM\n", pp->redline_rpm, pp->idle_rpm);
        } else {
            fprintf(stderr, "Warning: Power plant '%s' not found\n", pp_ref->valuestring);
        }
    } else {
        // No power plant - set defaults for testing
        out->physics.target_0_60_seconds = 6.0f;
        out->physics.target_accel_ms2 = 4.47f;
        out->physics.accel_force = out->physics.chassis_mass * out->physics.target_accel_ms2;
        out->physics.top_speed_ms = 40.0f;  // ~90 mph default
        printf("  No power_plant defined - using defaults\n");
    }

    // Calculate brake force from axle brake multipliers
    float total_brake_mult = 0.0f;
    for (int i = 0; i < out->axle_count; i++) {
        total_brake_mult += out->axles[i].brake_force_multiplier;
    }
    if (total_brake_mult > 0) {
        // Brake force = mass * deceleration (assume ~10 m/s² for hard braking)
        out->physics.brake_force = out->physics.chassis_mass * 10.0f * (total_brake_mult / out->axle_count);
    } else {
        out->physics.brake_force = out->physics.chassis_mass * 8.0f;  // Default braking
    }
    printf("  Brake Force: %.0fN\n", out->physics.brake_force);

    // Populate physics struct for Jolt
    out->physics.use_per_wheel_config = true;

    // Use first wheel's dimensions as legacy values
    if (out->wheel_count > 0) {
        out->physics.wheel_radius = out->wheels[0].radius;
        out->physics.wheel_width = out->wheels[0].width;
        out->physics.wheel_mass = out->wheels[0].mass;
    }
    out->physics.suspension_frequency = out->defaults.suspension.frequency;
    out->physics.suspension_damping = out->defaults.suspension.damping;
    out->physics.suspension_travel = out->defaults.suspension.travel;
    out->physics.tire_friction = out->defaults.wheel.friction;

    // Initialize per-wheel arrays with defaults
    for (int i = 0; i < 4; i++) {
        out->physics.wheel_positions[i] = vec3(0, 0, 0);
        out->physics.wheel_radii[i] = out->defaults.wheel.radius;
        out->physics.wheel_widths[i] = out->defaults.wheel.width;
        out->physics.wheel_masses[i] = out->defaults.wheel.mass;
        out->physics.wheel_steering[i] = false;
        out->physics.wheel_driven[i] = (i >= 2);  // RWD by default (rear wheels)
        out->physics.wheel_steer_angles[i] = 0.6f;
        out->physics.wheel_suspension_frequency[i] = out->defaults.suspension.frequency;
        out->physics.wheel_suspension_damping[i] = out->defaults.suspension.damping;
        out->physics.wheel_suspension_travel[i] = out->defaults.suspension.travel;
    }

    // ========== ENGINE & TRANSMISSION DEFAULTS ==========
    // Real drivetrain mode: engine torque through gearbox
    out->physics.use_linear_accel = false;

    // Engine defaults - ONLY if not already set by power plant
    if (out->physics.engine_max_torque <= 0) {
        out->physics.engine_max_torque = 300.0f;    // Nm (typical sports car)
    }
    if (out->physics.engine_max_rpm <= 0) {
        out->physics.engine_max_rpm = 6000.0f;       // RPM
    }
    if (out->physics.engine_idle_rpm <= 0) {
        out->physics.engine_idle_rpm = 1000.0f;      // RPM
    }

    // Transmission defaults (standard 5-speed from gearboxes.json)
    out->physics.use_config_transmission = true;
    out->physics.gear_count = 5;
    out->physics.gear_ratios[0] = 1.5f;
    out->physics.gear_ratios[1] = 1.2f;
    out->physics.gear_ratios[2] = 1.0f;
    out->physics.gear_ratios[3] = 0.85f;
    out->physics.gear_ratios[4] = 0.7f;
    out->physics.reverse_count = 1;
    out->physics.reverse_ratios[0] = 1.5f;  // POSITIVE - Jolt handles direction via input sign
    out->physics.differential_ratio = 1.29f;
    out->physics.clutch_strength = 10.0f;   // Clutch engagement strength
    out->physics.switch_time = 0.2f;        // Time to complete gear change
    out->physics.switch_latency = 0.1f;     // Delay before shift initiates

    // ========== DRIVETRAIN SECTION ==========
    // Parse drivetrain if present - overrides defaults (use drivetrain_section from above)
    if (drivetrain_section) {
        // Engine physics already set above from power plant lookup

        // Get gearbox from drivetrain
        cJSON* gearbox_ref = cJSON_GetObjectItem(drivetrain_section, "gearbox");
        if (gearbox_ref && cJSON_IsString(gearbox_ref)) {
            const GearboxEquipment* gb = equipment_find_gearbox(gearbox_ref->valuestring);
            if (gb) {
                out->physics.gear_count = gb->gear_count;
                for (int g = 0; g < gb->gear_count && g < MAX_CONFIG_GEARS; g++) {
                    out->physics.gear_ratios[g] = gb->gear_ratios[g];
                }
                out->physics.reverse_count = gb->reverse_count;
                for (int g = 0; g < gb->reverse_count && g < MAX_CONFIG_GEARS; g++) {
                    out->physics.reverse_ratios[g] = gb->reverse_ratios[g];
                }
                out->physics.differential_ratio = gb->differential_ratio;
                out->physics.clutch_strength = gb->clutch_strength;
                out->physics.switch_time = gb->switch_time;
                out->physics.switch_latency = gb->switch_latency;
                out->physics.shift_up_rpm = gb->shift_up_rpm;
                out->physics.shift_down_rpm = gb->shift_down_rpm;
                printf("  Drivetrain: %s (%.0f Nm, %d gears, diff=%.2f)\n",
                       gb->name, out->physics.engine_max_torque, gb->gear_count, gb->differential_ratio);
            } else {
                fprintf(stderr, "Warning: Gearbox '%s' not found\n", gearbox_ref->valuestring);
            }
        }

        // Set driven wheels from driven_axles
        cJSON* driven_axles = cJSON_GetObjectItem(drivetrain_section, "driven_axles");
        if (driven_axles && cJSON_IsArray(driven_axles)) {
            // First clear all driven flags
            for (int i = 0; i < 4; i++) {
                out->physics.wheel_driven[i] = false;
            }
            // Then set driven for each specified axle
            cJSON* axle_id;
            cJSON_ArrayForEach(axle_id, driven_axles) {
                if (!cJSON_IsString(axle_id)) continue;
                // Find the axle and mark its wheels as driven
                for (int a = 0; a < out->axle_count; a++) {
                    if (strcmp(out->axles[a].name, axle_id->valuestring) == 0) {
                        for (int w = 0; w < out->axles[a].wheel_count; w++) {
                            int idx = wheel_id_to_index(out->axles[a].wheel_ids[w]);
                            if (idx >= 0 && idx < 4) {
                                out->physics.wheel_driven[idx] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    // Map wheels to per-wheel arrays
    // Convert from JSON coords (X=forward, Z=left) to physics coords (X=right, Z=forward)
    for (int i = 0; i < out->wheel_count; i++) {
        int idx = wheel_id_to_index(out->wheels[i].id);
        if (idx >= 0 && idx < 4) {
            Vec3 json_pos = out->wheels[i].position;
            // Transform: physics_x = -json_z, physics_z = json_x
            out->physics.wheel_positions[idx] = vec3(-json_pos.z, json_pos.y, json_pos.x);
            out->physics.wheel_radii[idx] = out->wheels[i].radius;
            out->physics.wheel_widths[idx] = out->wheels[i].width;
            out->physics.wheel_masses[idx] = out->wheels[i].mass;
        }
    }

    // Map axle properties to per-wheel arrays
    for (int a = 0; a < out->axle_count; a++) {
        AxleDef* axle = &out->axles[a];
        for (int w = 0; w < axle->wheel_count; w++) {
            int idx = wheel_id_to_index(axle->wheel_ids[w]);
            if (idx >= 0 && idx < 4) {
                out->physics.wheel_steering[idx] = axle->steering;
                if (axle->steering) {
                    out->physics.wheel_steer_angles[idx] = axle->max_steer_angle;
                }
                if (axle->has_suspension) {
                    out->physics.wheel_suspension_frequency[idx] = axle->suspension_frequency;
                    out->physics.wheel_suspension_damping[idx] = axle->suspension_damping;
                    out->physics.wheel_suspension_travel[idx] = axle->suspension_travel;
                }
            }
        }
    }

    // Debug: print physics wheel positions
    printf("  Physics wheel positions:\n");
    for (int i = 0; i < 4; i++) {
        printf("    [%d]: (%.2f, %.2f, %.2f)\n", i,
               out->physics.wheel_positions[i].x,
               out->physics.wheel_positions[i].y,
               out->physics.wheel_positions[i].z);
    }

    // Get max steer angle from steering axle
    for (int i = 0; i < out->axle_count; i++) {
        if (out->axles[i].steering) {
            out->physics.max_steer_angle = out->axles[i].max_steer_angle;
            break;
        }
    }

    // ========== SCRIPTS SECTION ==========
    // Parse scripts array for driver assists, AI, etc.
    cJSON* scripts = cJSON_GetObjectItem(root, "scripts");
    out->script_count = 0;
    if (scripts && cJSON_IsArray(scripts)) {
        cJSON* script_item;
        cJSON_ArrayForEach(script_item, scripts) {
            if (out->script_count >= MAX_VEHICLE_SCRIPTS) break;

            VehicleScript* s = &out->scripts[out->script_count];
            json_get_string(script_item, "name", s->name, MAX_NAME_LENGTH, "unnamed");
            json_get_string(script_item, "path", s->path, MAX_SCRIPT_PATH, "");
            s->enabled = json_get_bool(script_item, "enabled", true);

            // Parse options object
            s->option_count = 0;
            cJSON* options = cJSON_GetObjectItem(script_item, "options");
            if (options && cJSON_IsObject(options)) {
                cJSON* opt;
                cJSON_ArrayForEach(opt, options) {
                    if (s->option_count >= MAX_SCRIPT_OPTIONS) break;
                    if (opt->string && (cJSON_IsNumber(opt) || cJSON_IsBool(opt))) {
                        ScriptOption* so = &s->options[s->option_count];
                        strncpy(so->key, opt->string, 31);
                        so->key[31] = '\0';
                        if (cJSON_IsBool(opt)) {
                            so->value = cJSON_IsTrue(opt) ? 1.0f : 0.0f;
                        } else {
                            so->value = (float)opt->valuedouble;
                        }
                        s->option_count++;
                    }
                }
            }

            if (s->path[0] != '\0') {
                printf("  Script: %s (%s, %s, %d options)\n",
                       s->name, s->path, s->enabled ? "enabled" : "disabled", s->option_count);
                out->script_count++;
            }
        }
    }

    cJSON_Delete(root);

    // Validate that all required fields are present
    if (!validate_vehicle_config(out, filepath)) {
        fprintf(stderr, "FAILED: Vehicle '%s' is missing required configuration\n", out->name);
        return false;
    }

    // Print total weight summary
    float total_weight_lbs = out->chassis_mass / LBS_TO_KG;
    printf("  TOTAL WEIGHT: %.0f lbs (%.0f kg) -> Physics mass: %.0f kg\n",
           total_weight_lbs, out->chassis_mass, out->physics.chassis_mass);

    // Calculate and store handling class
    out->physics.handling_class = handling_calculate_hc(hc_chassis, hc_suspension, hc_tires);
    printf("  Handling Class: HC %d (chassis %+d, suspension %+d, tires %+d)\n",
           out->physics.handling_class, hc_chassis, hc_suspension, hc_tires);

    printf("Loaded vehicle: %s (%d wheels, %d axles)\n",
           out->name, out->wheel_count, out->axle_count);
    return true;
}

VehicleConfig config_vehicle_to_physics(const VehicleJSON* json) {
    return json->physics;
}

SceneJSON config_default_scene(void) {
    SceneJSON s = {0};
    strcpy(s.name, "default");

    s.arena.size = 60.0f;
    s.arena.wall_height = 4.0f;
    s.arena.wall_thickness = 1.0f;
    s.arena.ground_y = 0.0f;

    s.world.gravity = -9.81f;
    s.world.contact.friction = 1.0f;
    s.world.contact.restitution = 0.0f;

    // Default showdown vehicles
    s.vehicle_count = 2;
    strcpy(s.vehicles[0].type, "sports_car");
    strcpy(s.vehicles[0].team, "red");
    s.vehicles[0].position = vec3(0, 0, -26);
    s.vehicles[0].rotation = 0.0f;

    strcpy(s.vehicles[1].type, "sports_car");
    strcpy(s.vehicles[1].team, "blue");
    s.vehicles[1].position = vec3(0, 0, 26);
    s.vehicles[1].rotation = 3.14159f;

    // Default obstacles
    s.obstacle_count = 5;
    strcpy(s.obstacles[0].type, "box");
    s.obstacles[0].position = vec3(0, 2.0f, 0);
    s.obstacles[0].size = vec3(5, 4, 5);
    s.obstacles[0].color = vec3(0.6f, 0.35f, 0.25f);

    for (int i = 0; i < 4; i++) {
        strcpy(s.obstacles[i+1].type, "box");
        s.obstacles[i+1].size = vec3(3, 4, 3);
        s.obstacles[i+1].color = vec3(0.4f, 0.4f, 0.45f);
    }
    s.obstacles[1].position = vec3(22, 2.0f, 22);
    s.obstacles[2].position = vec3(-22, 2.0f, 22);
    s.obstacles[3].position = vec3(22, 2.0f, -22);
    s.obstacles[4].position = vec3(-22, 2.0f, -22);

    return s;
}

bool config_load_scene(const char* filepath, SceneJSON* out) {
    *out = config_default_scene();

    char* json_str = read_file(filepath);
    if (!json_str) {
        fprintf(stderr, "ERROR: Scene config required but not found: %s\n", filepath);
        fprintf(stderr, "       Game will use defaults but layout may be wrong!\n");
        return false;
    }

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);

    if (!root) {
        fprintf(stderr, "JSON parse error in %s\n", filepath);
        return false;
    }

    json_get_string(root, "name", out->name, MAX_NAME_LENGTH, "default");

    // Arena
    cJSON* arena = cJSON_GetObjectItem(root, "arena");
    if (arena) {
        out->arena.size = json_get_float(arena, "size", out->arena.size);
        out->arena.wall_height = json_get_float(arena, "wall_height", out->arena.wall_height);
        out->arena.wall_thickness = json_get_float(arena, "wall_thickness", out->arena.wall_thickness);
        out->arena.ground_y = json_get_float(arena, "ground_y", out->arena.ground_y);
    }

    // World physics (simplified for Jolt)
    cJSON* world = cJSON_GetObjectItem(root, "world");
    if (world) {
        out->world.gravity = json_get_float(world, "gravity", out->world.gravity);

        cJSON* contact = cJSON_GetObjectItem(world, "contact");
        if (contact) {
            out->world.contact.friction = json_get_float(contact, "friction", out->world.contact.friction);
            out->world.contact.restitution = json_get_float(contact, "restitution", out->world.contact.restitution);
        }
    }

    // Vehicles
    cJSON* vehicles = cJSON_GetObjectItem(root, "vehicles");
    if (vehicles && cJSON_IsArray(vehicles)) {
        out->vehicle_count = 0;
        cJSON* v;
        cJSON_ArrayForEach(v, vehicles) {
            if (out->vehicle_count >= MAX_SCENE_VEHICLES) break;

            SceneVehicle* sv = &out->vehicles[out->vehicle_count];
            json_get_string(v, "type", sv->type, MAX_NAME_LENGTH, "sports_car");
            json_get_string(v, "team", sv->team, MAX_NAME_LENGTH, "red");
            sv->position = json_get_vec3(v, "position", vec3(0, 0, 0));
            sv->rotation = json_get_float(v, "rotation", 0.0f);
            out->vehicle_count++;
        }
    }

    // Obstacles
    cJSON* obstacles = cJSON_GetObjectItem(root, "obstacles");
    if (obstacles && cJSON_IsArray(obstacles)) {
        out->obstacle_count = 0;
        cJSON* o;
        cJSON_ArrayForEach(o, obstacles) {
            if (out->obstacle_count >= MAX_SCENE_OBSTACLES) break;

            SceneObstacle* so = &out->obstacles[out->obstacle_count];
            json_get_string(o, "type", so->type, MAX_NAME_LENGTH, "box");
            so->position = json_get_vec3(o, "position", vec3(0, 0, 0));
            so->size = json_get_vec3(o, "size", vec3(1, 1, 1));
            so->color = json_get_vec3(o, "color", vec3(0.5f, 0.5f, 0.5f));
            so->rotation_y = json_get_float(o, "rotation", 0.0f);
            out->obstacle_count++;
        }
    }

    cJSON_Delete(root);
    printf("Loaded scene config: %s (%d vehicles, %d obstacles)\n",
           out->name, out->vehicle_count, out->obstacle_count);
    return true;
}

// ============================================================================
// Physics Mode Loading
// ============================================================================

// Global active physics mode
static PhysicsMode s_active_physics_mode = {
    .name = "Strict tabletop",
    .type = PHYSICS_MODE_STRICT_CAR_WARS,
    .overrides = {
        .tire = { .mu = 2.0f, .reference_radius = 0.35f, .reference_width = 0.2f },
        .override_tire = true,
        .suspension = { .frequency = 3.0f, .damping = 0.8f, .travel = 0.15f },
        .override_suspension = false  // Use suspension.json values (hot-reload with R)
    }
};

bool config_load_physics_mode(const char* filepath, PhysicsMode* out) {
    // Set defaults (strict mode)
    memset(out, 0, sizeof(*out));
    strcpy(out->name, "Strict tabletop");
    out->type = PHYSICS_MODE_STRICT_CAR_WARS;
    out->overrides.tire.mu = 2.0f;
    out->overrides.tire.reference_radius = 0.35f;
    out->overrides.tire.reference_width = 0.2f;
    out->overrides.override_tire = true;
    out->overrides.suspension.frequency = 3.0f;
    out->overrides.suspension.damping = 0.8f;
    out->overrides.suspension.travel = 0.15f;
    out->overrides.override_suspension = false;  // Use suspension.json values (hot-reload with R)

    char* json_str = read_file(filepath);
    if (!json_str) {
        fprintf(stderr, "Warning: Physics mode config not found: %s (using strict defaults)\n", filepath);
        s_active_physics_mode = *out;
        return false;
    }

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);

    if (!root) {
        fprintf(stderr, "JSON parse error in %s\n", filepath);
        s_active_physics_mode = *out;
        return false;
    }

    // Get active mode name
    char active_mode[64] = "strict_car_wars";
    json_get_string(root, "active_mode", active_mode, 64, "strict_car_wars");

    // Parse modes
    cJSON* modes = cJSON_GetObjectItem(root, "modes");
    if (!modes) {
        cJSON_Delete(root);
        s_active_physics_mode = *out;
        return false;
    }

    // Get the active mode config
    cJSON* mode_config = cJSON_GetObjectItem(modes, active_mode);
    if (!mode_config) {
        fprintf(stderr, "Warning: Physics mode '%s' not found, using strict defaults\n", active_mode);
        cJSON_Delete(root);
        s_active_physics_mode = *out;
        return false;
    }

    // Parse mode name and type
    json_get_string(mode_config, "name", out->name, MAX_NAME_LENGTH, "Unknown");
    if (strcmp(active_mode, "extended") == 0) {
        out->type = PHYSICS_MODE_EXTENDED;
        // Extended mode has no overrides by default
        out->overrides.override_tire = false;
        out->overrides.override_suspension = false;
    } else {
        out->type = PHYSICS_MODE_STRICT_CAR_WARS;
    }

    // Parse overrides
    cJSON* overrides = cJSON_GetObjectItem(mode_config, "overrides");
    if (overrides) {
        // Tire overrides
        cJSON* tire = cJSON_GetObjectItem(overrides, "tire");
        if (tire && cJSON_IsObject(tire)) {
            out->overrides.override_tire = true;
            out->overrides.tire.mu = json_get_float(tire, "mu", 2.0f);
            out->overrides.tire.reference_radius = json_get_float(tire, "reference_radius", 0.35f);
            out->overrides.tire.reference_width = json_get_float(tire, "reference_width", 0.2f);
        }

        // Suspension overrides
        cJSON* susp = cJSON_GetObjectItem(overrides, "suspension");
        if (susp && cJSON_IsObject(susp)) {
            out->overrides.override_suspension = true;
            out->overrides.suspension.frequency = json_get_float(susp, "frequency", 3.0f);
            out->overrides.suspension.damping = json_get_float(susp, "damping", 0.8f);
            out->overrides.suspension.travel = json_get_float(susp, "travel", 0.15f);
        }

    }

    cJSON_Delete(root);

    // Store as active mode
    s_active_physics_mode = *out;

    printf("Loaded physics mode: %s (%s)\n", out->name,
           out->type == PHYSICS_MODE_STRICT_CAR_WARS ? "strict" : "extended");
    if (out->overrides.override_tire) {
        printf("  Tire override: mu=%.2f, ref_radius=%.2f\n",
               out->overrides.tire.mu, out->overrides.tire.reference_radius);
    }
    if (out->overrides.override_suspension) {
        printf("  Suspension override: freq=%.1f, damp=%.2f, travel=%.2f\n",
               out->overrides.suspension.frequency,
               out->overrides.suspension.damping,
               out->overrides.suspension.travel);
    }

    return true;
}

const PhysicsMode* config_get_physics_mode(void) {
    return &s_active_physics_mode;
}

bool config_is_strict_mode(void) {
    return s_active_physics_mode.type == PHYSICS_MODE_STRICT_CAR_WARS;
}

void config_apply_physics_mode(VehicleJSON* vehicle, const PhysicsMode* mode) {
    if (!mode) return;

    printf("  Applying physics mode: %s (matchbox car physics - wheels unpowered)\n", mode->name);

    // Apply tire override
    if (mode->overrides.override_tire) {
        vehicle->defaults.wheel.friction = mode->overrides.tire.mu;
        vehicle->physics.tire_friction = mode->overrides.tire.mu;
        printf("    -> Tire friction: %.2f\n", mode->overrides.tire.mu);
    }

    // Apply suspension override
    if (mode->overrides.override_suspension) {
        vehicle->defaults.suspension.frequency = mode->overrides.suspension.frequency;
        vehicle->defaults.suspension.damping = mode->overrides.suspension.damping;
        vehicle->defaults.suspension.travel = mode->overrides.suspension.travel;

        vehicle->physics.suspension_frequency = mode->overrides.suspension.frequency;
        vehicle->physics.suspension_damping = mode->overrides.suspension.damping;
        vehicle->physics.suspension_travel = mode->overrides.suspension.travel;

        // Update per-wheel suspension
        for (int i = 0; i < 4; i++) {
            vehicle->physics.wheel_suspension_frequency[i] = mode->overrides.suspension.frequency;
            vehicle->physics.wheel_suspension_damping[i] = mode->overrides.suspension.damping;
            vehicle->physics.wheel_suspension_travel[i] = mode->overrides.suspension.travel;
        }
        printf("    -> Suspension: freq=%.1f, damp=%.2f\n",
               mode->overrides.suspension.frequency,
               mode->overrides.suspension.damping);
    }

    // Matchbox car physics is always on - accel_force is set during vehicle loading
    printf("    -> Accel force: %.0fN (%.1fs 0-60)\n",
           vehicle->physics.accel_force, vehicle->physics.target_0_60_seconds);
}

// ============================================================================
// Friction Curve Loading
// ============================================================================

// Global active friction curve (generous by default)
static FrictionCurve s_active_friction_curve = {
    .name = "Generous (Arcade)",
    .longitudinal = {
        {0.00f, 0.0f},
        {0.06f, 1.2f},
        {0.20f, 1.0f},
        {1.00f, 0.9f},
        {10.0f, 0.85f}
    },
    .longitudinal_count = 5,
    .lateral = {
        {0.0f, 0.0f},
        {3.0f, 1.0f},
        {20.0f, 1.0f},
        {90.0f, 0.9f}
    },
    .lateral_count = 4,
    .lateral_rails_multiplier = 3.0f
};

bool config_load_friction_curve(const char* filepath, FrictionCurve* out) {
    // Set generous defaults
    memset(out, 0, sizeof(*out));
    strcpy(out->name, "Generous (Arcade)");
    out->longitudinal[0] = (FrictionPoint){0.00f, 0.0f};
    out->longitudinal[1] = (FrictionPoint){0.06f, 1.2f};
    out->longitudinal[2] = (FrictionPoint){0.20f, 1.0f};
    out->longitudinal[3] = (FrictionPoint){1.00f, 0.9f};
    out->longitudinal[4] = (FrictionPoint){10.0f, 0.85f};
    out->longitudinal_count = 5;
    out->lateral[0] = (FrictionPoint){0.0f, 0.0f};
    out->lateral[1] = (FrictionPoint){3.0f, 1.0f};
    out->lateral[2] = (FrictionPoint){20.0f, 1.0f};
    out->lateral[3] = (FrictionPoint){90.0f, 0.9f};
    out->lateral_count = 4;
    out->lateral_rails_multiplier = 3.0f;

    char* json_str = read_file(filepath);
    if (!json_str) {
        printf("[Physics] Friction curve config not found, using generous defaults\n");
        s_active_friction_curve = *out;
        return false;
    }

    cJSON* root = cJSON_Parse(json_str);
    free(json_str);

    if (!root) {
        fprintf(stderr, "JSON parse error in %s\n", filepath);
        s_active_friction_curve = *out;
        return false;
    }

    // Get active curve name
    char active_curve[64] = "generous";
    json_get_string(root, "active_curve", active_curve, 64, "generous");

    // Parse friction_curves
    cJSON* curves = cJSON_GetObjectItem(root, "friction_curves");
    if (!curves) {
        cJSON_Delete(root);
        s_active_friction_curve = *out;
        return false;
    }

    // Get the active curve config
    cJSON* curve_config = cJSON_GetObjectItem(curves, active_curve);
    if (!curve_config) {
        fprintf(stderr, "Warning: Friction curve '%s' not found, using generous\n", active_curve);
        cJSON_Delete(root);
        s_active_friction_curve = *out;
        return false;
    }

    // Parse curve name
    json_get_string(curve_config, "name", out->name, MAX_NAME_LENGTH, "Unknown");

    // Parse longitudinal friction points
    cJSON* longitudinal = cJSON_GetObjectItem(curve_config, "longitudinal");
    if (longitudinal && cJSON_IsArray(longitudinal)) {
        out->longitudinal_count = 0;
        cJSON* point;
        cJSON_ArrayForEach(point, longitudinal) {
            if (out->longitudinal_count >= MAX_FRICTION_POINTS) break;
            if (cJSON_IsArray(point) && cJSON_GetArraySize(point) >= 2) {
                out->longitudinal[out->longitudinal_count].slip =
                    (float)cJSON_GetArrayItem(point, 0)->valuedouble;
                out->longitudinal[out->longitudinal_count].friction =
                    (float)cJSON_GetArrayItem(point, 1)->valuedouble;
                out->longitudinal_count++;
            }
        }
    }

    // Parse lateral friction
    cJSON* lateral_config = cJSON_GetObjectItem(root, "lateral_friction");
    if (lateral_config) {
        out->lateral_rails_multiplier = json_get_float(lateral_config, "rails_multiplier", 3.0f);

        cJSON* lateral_points = cJSON_GetObjectItem(lateral_config, "points");
        if (lateral_points && cJSON_IsArray(lateral_points)) {
            out->lateral_count = 0;
            cJSON* point;
            cJSON_ArrayForEach(point, lateral_points) {
                if (out->lateral_count >= MAX_FRICTION_POINTS) break;
                if (cJSON_IsArray(point) && cJSON_GetArraySize(point) >= 2) {
                    out->lateral[out->lateral_count].slip =
                        (float)cJSON_GetArrayItem(point, 0)->valuedouble;
                    out->lateral[out->lateral_count].friction =
                        (float)cJSON_GetArrayItem(point, 1)->valuedouble;
                    out->lateral_count++;
                }
            }
        }
    }

    cJSON_Delete(root);

    // Store as active curve
    s_active_friction_curve = *out;

    printf("[Physics] Loaded friction curve: %s (%d longitudinal points)\n",
           out->name, out->longitudinal_count);
    for (int i = 0; i < out->longitudinal_count; i++) {
        printf("  [%.0f%% slip -> %.0f%% grip]\n",
               out->longitudinal[i].slip * 100.0f,
               out->longitudinal[i].friction * 100.0f);
    }

    return true;
}

const FrictionCurve* config_get_friction_curve(void) {
    return &s_active_friction_curve;
}
