 /*
 * JSON Configuration Loader Implementation
 */

#include "config_loader.h"
#include "../vendor/cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Read entire file into memory
static char* read_file(const char* filepath) {
    FILE* f = fopen(filepath, "rb");
    if (!f) {
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

// Helper: parse drivetrain type from string
static DrivetrainType parse_drivetrain_type(const char* str) {
    if (!str) return DRIVETRAIN_RWD;
    if (strcmp(str, "RWD") == 0) return DRIVETRAIN_RWD;
    if (strcmp(str, "FWD") == 0) return DRIVETRAIN_FWD;
    if (strcmp(str, "AWD") == 0) return DRIVETRAIN_AWD;
    if (strcmp(str, "4WD") == 0) return DRIVETRAIN_4WD;
    return DRIVETRAIN_RWD;
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
    VehicleJSON v = {0};
    v.vehicle_class = VEHICLE_CLASS_WHEELED;
    v.version = 1;
    strcpy(v.name, "default");

    // Chassis defaults
    v.chassis_mass = 600.0f;
    v.chassis_length = 3.5f;
    v.chassis_width = 1.8f;
    v.chassis_height = 0.8f;

    // V2 consolidated defaults
    v.defaults.wheel.radius = 0.35f;
    v.defaults.wheel.width = 0.2f;
    v.defaults.wheel.mass = 15.0f;
    v.defaults.wheel.friction = 3.0f;
    v.defaults.wheel.slip = 0.0001f;

    v.defaults.suspension.erp = 0.7f;
    v.defaults.suspension.cfm = 0.005f;
    v.defaults.suspension.travel = 0.2f;

    v.wheel_count = 0;
    v.axle_count = 0;

    v.drivetrain.type = DRIVETRAIN_RWD;
    v.drivetrain.motor_force = 5000.0f;
    v.drivetrain.brake_force = 15000.0f;
    v.drivetrain.max_speed = 80.0f;

    // Legacy V1 physics struct
    v.physics.chassis_mass = 600.0f;
    v.physics.chassis_length = 3.5f;
    v.physics.chassis_width = 1.8f;
    v.physics.chassis_height = 0.8f;

    v.physics.wheel_mass = 15.0f;
    v.physics.wheel_radius = 0.35f;
    v.physics.wheel_width = 0.2f;

    v.physics.suspension_erp = 0.7f;
    v.physics.suspension_cfm = 0.005f;
    v.physics.suspension_travel = 0.2f;

    v.physics.max_steer_angle = 0.7f;
    v.physics.max_motor_force = 5000.0f;
    v.physics.max_brake_force = 15000.0f;

    v.physics.tire_friction = 3.0f;
    v.physics.tire_slip = 0.0001f;

    v.physics.wheelbase = 0.0f;
    v.physics.track_width = 0.0f;
    v.physics.wheel_mount_height = 0.0f;

    v.motor_max_speed = 80.0f;

    return v;
}

// Parse wheels array (uses defaults.wheel for values not specified)
static void parse_wheels(cJSON* wheels_arr, VehicleJSON* out) {
    out->wheel_count = 0;
    cJSON* w;
    cJSON_ArrayForEach(w, wheels_arr) {
        if (out->wheel_count >= MAX_WHEELS) break;

        WheelDef* wheel = &out->wheels[out->wheel_count];
        json_get_string(w, "id", wheel->id, 16, "W");
        wheel->position = json_array_to_vec3(cJSON_GetObjectItem(w, "position"), vec3(0, 0, 0));
        // Use defaults.wheel as fallback for unspecified values
        wheel->radius = json_get_float(w, "radius", out->defaults.wheel.radius);
        wheel->width = json_get_float(w, "width", out->defaults.wheel.width);
        wheel->mass = json_get_float(w, "mass", out->defaults.wheel.mass);
        out->wheel_count++;
    }
}

// Parse axles array (with per-axle suspension support)
static void parse_axles(cJSON* axles_arr, VehicleJSON* out) {
    out->axle_count = 0;
    cJSON* a;
    cJSON_ArrayForEach(a, axles_arr) {
        if (out->axle_count >= MAX_AXLES) break;

        AxleDef* axle = &out->axles[out->axle_count];
        json_get_string(a, "name", axle->name, MAX_NAME_LENGTH, "axle");
        axle->steering = json_get_bool(a, "steering", false);
        axle->driven = json_get_bool(a, "driven", false);
        axle->max_steer_angle = json_get_float(a, "max_steer_angle", 0.6f);

        // Parse per-axle suspension (optional - falls back to defaults.suspension)
        cJSON* axle_susp = cJSON_GetObjectItem(a, "suspension");
        if (axle_susp && cJSON_IsObject(axle_susp)) {
            axle->has_suspension = true;
            // Use defaults.suspension as fallback, override with axle-specific values
            axle->suspension_erp = json_get_float(axle_susp, "erp", out->defaults.suspension.erp);
            axle->suspension_cfm = json_get_float(axle_susp, "cfm", out->defaults.suspension.cfm);
            axle->suspension_travel = json_get_float(axle_susp, "travel", out->defaults.suspension.travel);
        } else {
            axle->has_suspension = false;
            // Will use defaults.suspension
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
}

bool config_load_vehicle(const char* filepath, VehicleJSON* out) {
    *out = config_default_vehicle();

    char* json_str = read_file(filepath);
    if (!json_str) {
        printf("Using default vehicle config\n");
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

    // Chassis (shared between v1 and v2)
    cJSON* chassis = cJSON_GetObjectItem(root, "chassis");
    if (chassis) {
        out->chassis_mass = json_get_float(chassis, "mass", out->chassis_mass);
        out->chassis_length = json_get_float(chassis, "length", out->chassis_length);
        out->chassis_width = json_get_float(chassis, "width", out->chassis_width);
        out->chassis_height = json_get_float(chassis, "height", out->chassis_height);
        // Also update legacy physics struct
        out->physics.chassis_mass = out->chassis_mass;
        out->physics.chassis_length = out->chassis_length;
        out->physics.chassis_width = out->chassis_width;
        out->physics.chassis_height = out->chassis_height;
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
            out->defaults.wheel.slip = json_get_float(def_wheel, "slip", out->defaults.wheel.slip);
        }
        // Parse defaults.suspension
        cJSON* def_susp = cJSON_GetObjectItem(defaults, "suspension");
        if (def_susp && cJSON_IsObject(def_susp)) {
            out->defaults.suspension.erp = json_get_float(def_susp, "erp", out->defaults.suspension.erp);
            out->defaults.suspension.cfm = json_get_float(def_susp, "cfm", out->defaults.suspension.cfm);
            out->defaults.suspension.travel = json_get_float(def_susp, "travel", out->defaults.suspension.travel);
        }
    }

    // Parse wheels array
    cJSON* wheels = cJSON_GetObjectItem(root, "wheels");
    if (wheels && cJSON_IsArray(wheels)) {
        parse_wheels(wheels, out);
    }

    // Parse axles array
    cJSON* axles = cJSON_GetObjectItem(root, "axles");
    if (axles && cJSON_IsArray(axles)) {
        parse_axles(axles, out);
    }

    // Parse drivetrain
    cJSON* drivetrain = cJSON_GetObjectItem(root, "drivetrain");
    if (drivetrain) {
        char dt_type[16];
        json_get_string(drivetrain, "type", dt_type, 16, "RWD");
        out->drivetrain.type = parse_drivetrain_type(dt_type);
        out->drivetrain.motor_force = json_get_float(drivetrain, "motor_force", out->drivetrain.motor_force);
        out->drivetrain.brake_force = json_get_float(drivetrain, "brake_force", out->drivetrain.brake_force);
        out->drivetrain.max_speed = json_get_float(drivetrain, "max_speed", out->drivetrain.max_speed);
    }

    // Populate physics struct for ODE
    out->physics.use_per_wheel_config = true;

    // Use first wheel's dimensions as legacy values
    if (out->wheel_count > 0) {
        out->physics.wheel_radius = out->wheels[0].radius;
        out->physics.wheel_width = out->wheels[0].width;
        out->physics.wheel_mass = out->wheels[0].mass;
    }
    out->physics.suspension_erp = out->defaults.suspension.erp;
    out->physics.suspension_cfm = out->defaults.suspension.cfm;
    out->physics.suspension_travel = out->defaults.suspension.travel;
    out->physics.max_motor_force = out->drivetrain.motor_force;
    out->physics.max_brake_force = out->drivetrain.brake_force;
    out->motor_max_speed = out->drivetrain.max_speed;
    out->physics.tire_friction = out->defaults.wheel.friction;
    out->physics.tire_slip = out->defaults.wheel.slip;

    // Initialize per-wheel arrays with defaults
    for (int i = 0; i < 4; i++) {
        out->physics.wheel_positions[i] = vec3(0, 0, 0);
        out->physics.wheel_radii[i] = out->defaults.wheel.radius;
        out->physics.wheel_widths[i] = out->defaults.wheel.width;
        out->physics.wheel_masses[i] = out->defaults.wheel.mass;
        out->physics.wheel_steering[i] = false;
        out->physics.wheel_driven[i] = false;
        out->physics.wheel_steer_angles[i] = 0.6f;
        out->physics.wheel_suspension_erp[i] = out->defaults.suspension.erp;
        out->physics.wheel_suspension_cfm[i] = out->defaults.suspension.cfm;
        out->physics.wheel_suspension_travel[i] = out->defaults.suspension.travel;
    }

    // Map wheels to per-wheel arrays
    for (int i = 0; i < out->wheel_count; i++) {
        int idx = wheel_id_to_index(out->wheels[i].id);
        if (idx >= 0 && idx < 4) {
            out->physics.wheel_positions[idx] = out->wheels[i].position;
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
                out->physics.wheel_driven[idx] = axle->driven;
                if (axle->steering) {
                    out->physics.wheel_steer_angles[idx] = axle->max_steer_angle;
                }
                if (axle->has_suspension) {
                    out->physics.wheel_suspension_erp[idx] = axle->suspension_erp;
                    out->physics.wheel_suspension_cfm[idx] = axle->suspension_cfm;
                    out->physics.wheel_suspension_travel[idx] = axle->suspension_travel;
                }
            }
        }
    }

    // Get max steer angle from steering axle
    for (int i = 0; i < out->axle_count; i++) {
        if (out->axles[i].steering) {
            out->physics.max_steer_angle = out->axles[i].max_steer_angle;
            break;
        }
    }

    printf("Loaded vehicle: %s (%d wheels, %d axles)\n",
           out->name, out->wheel_count, out->axle_count);

    cJSON_Delete(root);
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
    s.world.erp = 0.8f;
    s.world.cfm = 0.00001f;
    s.world.contact.friction = 3.0f;
    s.world.contact.slip = 0.0001f;
    s.world.contact.soft_erp = 0.8f;
    s.world.contact.soft_cfm = 0.0001f;

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
        printf("Using default scene config\n");
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

    // World physics
    cJSON* world = cJSON_GetObjectItem(root, "world");
    if (world) {
        out->world.gravity = json_get_float(world, "gravity", out->world.gravity);
        out->world.erp = json_get_float(world, "erp", out->world.erp);
        out->world.cfm = json_get_float(world, "cfm", out->world.cfm);

        cJSON* contact = cJSON_GetObjectItem(world, "contact");
        if (contact) {
            out->world.contact.friction = json_get_float(contact, "friction", out->world.contact.friction);
            out->world.contact.slip = json_get_float(contact, "slip", out->world.contact.slip);
            out->world.contact.soft_erp = json_get_float(contact, "soft_erp", out->world.contact.soft_erp);
            out->world.contact.soft_cfm = json_get_float(contact, "soft_cfm", out->world.contact.soft_cfm);
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
            out->obstacle_count++;
        }
    }

    cJSON_Delete(root);
    printf("Loaded scene config: %s (%d vehicles, %d obstacles)\n",
           out->name, out->vehicle_count, out->obstacle_count);
    return true;
}
