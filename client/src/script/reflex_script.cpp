/*
 * Reflex Script Engine Implementation
 * Uses Sol3 with LuaJIT backend
 *
 * Architecture:
 * - C++ loads master.lua once (the orchestrator)
 * - Master script manages per-vehicle script instances via loadfile()
 * - Each vehicle gets an isolated script environment
 * - Modules (abs, tcs) are singletons loaded via require()
 */

#include "reflex_script.h"
#include "../physics/jolt_physics.h"

#include <sol/sol.hpp>
#include <string>
#include <iostream>
#include <cmath>

// Script engine - manages Lua state and master script
struct ReflexScriptEngine {
    sol::state lua;
    sol::table master;  // The master script module
    bool valid;
};

// Helper: convert radians to degrees
static float rad_to_deg(float rad) {
    return rad * 180.0f / 3.14159265f;
}

// Helper: convert m/s to mph
static float ms_to_mph(float ms) {
    return ms * 2.23694f;
}

extern "C" {

ReflexScriptEngine* reflex_create(void) {
    auto* engine = new ReflexScriptEngine();
    engine->valid = false;

    // Open libraries
    engine->lua.open_libraries(
        sol::lib::base,
        sol::lib::math,
        sol::lib::string,
        sol::lib::table,
        sol::lib::package,
        sol::lib::io  // For loadfile
    );

    // Set up package.path for module loading
    // Game runs from client/build/, scripts are at ../../assets/scripts/
    engine->lua["package"]["path"] =
        "../../assets/scripts/?.lua;"
        "../../assets/scripts/?/init.lua;"
        "../../assets/scripts/modules/?.lua";

    // Load the master script
    const char* master_path = "../../assets/scripts/master.lua";
    sol::protected_function_result result = engine->lua.safe_script_file(
        master_path,
        sol::script_pass_on_error
    );

    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] Failed to load master.lua: " << err.what() << std::endl;
        delete engine;
        return nullptr;
    }

    // Get the master module table
    engine->master = result;
    if (!engine->master.valid() || engine->master.get_type() != sol::type::table) {
        std::cerr << "[Reflex] master.lua did not return a table" << std::endl;
        delete engine;
        return nullptr;
    }

    // Verify required functions exist
    sol::object attach_fn = engine->master["attach_script"];
    sol::object update_fn = engine->master["update"];
    if (!attach_fn.is<sol::function>() || !update_fn.is<sol::function>()) {
        std::cerr << "[Reflex] master.lua missing required functions" << std::endl;
        delete engine;
        return nullptr;
    }

    engine->valid = true;
    std::cout << "[Reflex] Script engine initialized with master.lua" << std::endl;
    return engine;
}

void reflex_destroy(ReflexScriptEngine* engine) {
    if (!engine) return;
    std::cout << "[Reflex] Script engine destroyed" << std::endl;
    delete engine;
}

bool reflex_attach_script(ReflexScriptEngine* engine,
                          int vehicle_id,
                          const char* script_name,
                          const char* script_path,
                          const char* config_keys[],
                          float config_values[],
                          int config_count) {
    if (!engine || !engine->valid || !script_name || !script_path) return false;

    // Build config table
    sol::table config = engine->lua.create_table();
    for (int i = 0; i < config_count; i++) {
        if (config_keys[i]) {
            config[config_keys[i]] = config_values[i];
        }
    }

    // Build full path (scripts are relative to assets)
    char full_path[512];
    snprintf(full_path, sizeof(full_path), "../../assets/%s", script_path);

    // Call master.attach_script(vehicle_id, script_name, script_path, config)
    sol::protected_function attach_fn = engine->master["attach_script"];
    sol::protected_function_result result = attach_fn(vehicle_id, script_name, full_path, config);

    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] attach_script error: " << err.what() << std::endl;
        return false;
    }

    bool success = result.get<bool>();
    return success;
}

void reflex_detach_script(ReflexScriptEngine* engine, int vehicle_id, const char* script_name) {
    if (!engine || !engine->valid || !script_name) return;

    sol::protected_function detach_fn = engine->master["detach_script"];
    sol::protected_function_result result = detach_fn(vehicle_id, script_name);

    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] detach_script error: " << err.what() << std::endl;
    }
}

void reflex_detach_all(ReflexScriptEngine* engine, int vehicle_id) {
    if (!engine || !engine->valid) return;

    sol::protected_function detach_fn = engine->master["detach_all"];
    sol::protected_function_result result = detach_fn(vehicle_id);

    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] detach_all error: " << err.what() << std::endl;
    }
}

void reflex_apply_controls(PhysicsWorld* pw, int vehicle_id,
                           const ScriptControls* controls) {
    if (!pw || !controls) return;
    if (!controls->controls_modified) return;

    physics_vehicle_set_steering(pw, vehicle_id, controls->steering);
    physics_vehicle_set_throttle(pw, vehicle_id, controls->throttle);
    physics_vehicle_set_handbrake(pw, vehicle_id, controls->handbrake);

    if (controls->use_per_wheel_brake) {
        for (int w = 0; w < MAX_SCRIPT_WHEELS; w++) {
            physics_vehicle_set_wheel_brake(pw, vehicle_id, w, controls->wheel_brake[w]);
        }
    } else {
        physics_vehicle_clear_per_wheel_brake(pw, vehicle_id);
        physics_vehicle_set_brake(pw, vehicle_id, controls->brake);
    }
}

void reflex_update_vehicle(ReflexScriptEngine* engine,
                           PhysicsWorld* pw,
                           int vehicle_id,
                           float dt) {
    if (!engine || !engine->valid || !pw) return;

    // Get physics data
    Vec3 pos;
    float heading, speed_ms;
    physics_vehicle_get_position(pw, vehicle_id, &pos);
    physics_vehicle_get_rotation(pw, vehicle_id, &heading);
    physics_vehicle_get_velocity(pw, vehicle_id, &speed_ms);

    PhysicsVehicle* vehicle = &pw->vehicles[vehicle_id];
    WheelState wheel_states[4];
    physics_vehicle_get_wheel_states(pw, vehicle_id, wheel_states);

    // ========================================
    // BUILD CONTEXT (ctx) TABLE
    // ========================================
    sol::table ctx = engine->lua.create_table();

    // ctx.id - Entity identifier
    ctx["id"] = vehicle_id;

    // ctx.dt - Delta time
    ctx["dt"] = dt;

    // ctx.telemetry - Read-only vehicle state
    sol::table telemetry = engine->lua.create_table();
    telemetry["position"] = engine->lua.create_table_with(
        "x", pos.x, "y", pos.y, "z", pos.z
    );
    telemetry["heading"] = rad_to_deg(heading);
    telemetry["speed"] = ms_to_mph(speed_ms);
    telemetry["speed_ms"] = speed_ms;

    // Accumulated time for rate-limiting in scripts
    // Only increment once per frame (on vehicle 0), not per vehicle
    static float accumulated_time = 0.0f;
    static int debug_counter = 0;
    if (vehicle_id == 0) {
        accumulated_time += dt;
        // Debug: print time every ~2 seconds to verify sync with physics [T=X.Xs] log
        debug_counter++;
        if (debug_counter % 120 == 0) {  // Roughly every 2 seconds at 60fps
            std::cout << "[Script] accumulated_time=" << accumulated_time
                      << "s (dt=" << dt << ")" << std::endl;
        }
    }
    telemetry["time"] = accumulated_time;

    // Wheel data for slip calculations
    sol::table wheels = engine->lua.create_table();
    for (int w = 0; w < 4; w++) {
        sol::table wheel = engine->lua.create_table();
        wheel["angular_velocity"] = wheel_states[w].angular_velocity;
        wheel["radius"] = vehicle->config.wheel_radii[w];
        wheel["slip"] = wheel_states[w].longitudinal_slip;
        wheel["has_contact"] = wheel_states[w].has_contact;
        wheels[w + 1] = wheel;  // Lua arrays are 1-indexed
    }
    telemetry["wheels"] = wheels;
    ctx["telemetry"] = telemetry;

    // ctx.controls - Player inputs (scripts can modify these)
    sol::table controls = engine->lua.create_table_with(
        "steering", vehicle->steering,
        "throttle", vehicle->throttle,
        "brake", vehicle->brake,
        "handbrake", vehicle->handbrake
    );
    // Per-wheel brake for TCS/ABS (Lua 1-indexed: 1=FL, 2=FR, 3=RL, 4=RR)
    sol::table wheel_brake = engine->lua.create_table();
    for (int w = 0; w < 4; w++) {
        wheel_brake[w + 1] = 0.0f;  // Initialize to 0
    }
    controls["wheel_brake"] = wheel_brake;
    ctx["controls"] = controls;

    // ========================================
    // CALL MASTER.UPDATE
    // ========================================
    sol::protected_function update_fn = engine->master["update"];
    sol::protected_function_result result = update_fn(vehicle_id, ctx);

    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] update error for vehicle " << vehicle_id
                  << ": " << err.what() << std::endl;
        return;
    }

    // ========================================
    // APPLY CONTROLS FROM SCRIPT
    // ========================================
    ScriptControls out_controls;
    memset(&out_controls, 0, sizeof(out_controls));

    out_controls.steering = controls.get_or("steering", 0.0f);
    out_controls.throttle = controls.get_or("throttle", 0.0f);
    out_controls.brake = controls.get_or("brake", 0.0f);
    out_controls.handbrake = controls.get_or("handbrake", 0.0f);
    out_controls.controls_modified = true;

    // Read back per-wheel brake values (for TCS/ABS)
    sol::optional<sol::table> wb_opt = controls["wheel_brake"];
    if (wb_opt) {
        sol::table wb = *wb_opt;
        bool any_wheel_brake = false;
        for (int w = 0; w < 4; w++) {
            float brake_val = wb.get_or(w + 1, 0.0f);  // Lua 1-indexed
            out_controls.wheel_brake[w] = brake_val;
            if (brake_val > 0.01f) any_wheel_brake = true;
        }
        out_controls.use_per_wheel_brake = any_wheel_brake;
    }

    reflex_apply_controls(pw, vehicle_id, &out_controls);
}

int reflex_reload_all_scripts(ReflexScriptEngine* engine) {
    if (!engine || !engine->valid) return -1;

    sol::protected_function reload_fn = engine->master["reload_all"];
    if (!reload_fn.valid()) {
        std::cerr << "[Reflex] master.reload_all not found" << std::endl;
        return -1;
    }

    sol::protected_function_result result = reload_fn();
    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] reload_all error: " << err.what() << std::endl;
        return -1;
    }

    int count = result.get<int>();
    return count;
}

// ============================================================================
// Turn-Based Maneuver Control
// ============================================================================

// Static storage for result strings (to return const char* safely)
static char s_result_level[32] = "pending";

bool reflex_start_turn(ReflexScriptEngine* engine,
                       int vehicle_id,
                       const char* maneuver_type,
                       const char* direction,
                       float duration) {
    if (!engine || !engine->valid) return false;

    // Call the vehicle's start_turn function via master
    // First, we need to get the vehicle's script environment
    // The turn_executor script exposes start_turn() as a global function

    // Build options table
    sol::table options = engine->lua.create_table();
    options["duration"] = duration;

    // Call: start_turn(ctx, maneuver_type, direction, options)
    // We need to build a minimal ctx with telemetry
    // For now, call a master function that dispatches to the right vehicle

    // Add a start_turn function to master that we can call
    sol::protected_function start_fn = engine->master["start_turn"];
    if (!start_fn.valid()) {
        // Master doesn't have start_turn yet - we need to add it
        // For now, try calling the vehicle's script directly
        std::cerr << "[Reflex] master.start_turn not implemented yet" << std::endl;
        return false;
    }

    sol::protected_function_result result = start_fn(vehicle_id, maneuver_type, direction, options);
    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] start_turn error: " << err.what() << std::endl;
        return false;
    }

    return result.get<bool>();
}

ManeuverResult reflex_end_turn(ReflexScriptEngine* engine, int vehicle_id) {
    ManeuverResult result = {};
    result.success = false;
    result.level = "failed";
    result.heading_error = 999.0f;

    if (!engine || !engine->valid) return result;

    sol::protected_function end_fn = engine->master["end_turn"];
    if (!end_fn.valid()) {
        std::cerr << "[Reflex] master.end_turn not implemented yet" << std::endl;
        return result;
    }

    sol::protected_function_result lua_result = end_fn(vehicle_id);
    if (!lua_result.valid()) {
        sol::error err = lua_result;
        std::cerr << "[Reflex] end_turn error: " << err.what() << std::endl;
        return result;
    }

    // Parse the result table from Lua
    sol::table res_table = lua_result;
    if (res_table.valid()) {
        result.success = res_table.get_or("success", false);

        std::string level = res_table.get_or<std::string>("level", "failed");
        strncpy(s_result_level, level.c_str(), sizeof(s_result_level) - 1);
        result.level = s_result_level;

        result.heading_error = res_table.get_or("heading_error", 999.0f);
        result.position_error = res_table.get_or("position_error", 999.0f);
        result.heading_achieved = res_table.get_or("heading_delta_achieved", 0.0f);
        result.heading_target = res_table.get_or("target_heading_delta", 0.0f);
    }

    return result;
}

bool reflex_is_turn_active(ReflexScriptEngine* engine, int vehicle_id) {
    if (!engine || !engine->valid) return false;

    sol::protected_function active_fn = engine->master["is_turn_active"];
    if (!active_fn.valid()) {
        return false;
    }

    sol::protected_function_result result = active_fn(vehicle_id);
    if (!result.valid()) {
        return false;
    }

    return result.get<bool>();
}

// ============================================================================
// Test Sequence Control
// ============================================================================

bool reflex_start_test_sequence(ReflexScriptEngine* engine,
                                 int vehicle_id,
                                 const char* sequence_name) {
    if (!engine || !engine->valid) return false;

    sol::protected_function start_fn = engine->master["start_test_sequence"];
    if (!start_fn.valid()) {
        std::cerr << "[Reflex] master.start_test_sequence not found" << std::endl;
        return false;
    }

    sol::protected_function_result result = start_fn(vehicle_id, sequence_name);
    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] start_test_sequence error: " << err.what() << std::endl;
        return false;
    }

    return result.get<bool>();
}

void reflex_stop_test_sequence(ReflexScriptEngine* engine) {
    if (!engine || !engine->valid) return;

    sol::protected_function stop_fn = engine->master["stop_test_sequence"];
    if (!stop_fn.valid()) return;

    stop_fn();
}

bool reflex_is_test_running(ReflexScriptEngine* engine) {
    if (!engine || !engine->valid) return false;

    sol::protected_function is_running_fn = engine->master["is_test_running"];
    if (!is_running_fn.valid()) return false;

    sol::protected_function_result result = is_running_fn();
    if (!result.valid()) return false;

    return result.get<bool>();
}

// ============================================================================
// Input Handling
// ============================================================================

void reflex_on_input(ReflexScriptEngine* engine,
                     const bool* keys_pressed,
                     int key_count,
                     int selected_vehicle_id) {
    if (!engine || !engine->valid || !keys_pressed) return;

    // Build table of pressed keys (only include keys that are actually pressed)
    sol::table pressed = engine->lua.create_table();
    int count = 0;
    for (int k = 0; k < key_count; k++) {
        if (keys_pressed[k]) {
            pressed[k] = true;
            count++;
        }
    }

    // Early exit if no keys pressed
    if (count == 0) return;

    // Call master.on_input(keys_pressed, selected_vehicle_id)
    sol::protected_function on_input_fn = engine->master["on_input"];
    if (!on_input_fn.valid()) {
        // master.on_input not implemented yet - silent fail
        return;
    }

    sol::protected_function_result result = on_input_fn(pressed, selected_vehicle_id);
    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] on_input error: " << err.what() << std::endl;
    }
}

// ============================================================================
// Event System
// ============================================================================

ScriptEventData reflex_event_data_create(void) {
    ScriptEventData data;
    data.count = 0;
    for (int i = 0; i < 16; i++) {
        data.keys[i] = nullptr;
        data.values[i] = 0.0f;
        data.strings[i] = nullptr;
    }
    return data;
}

void reflex_event_data_add_float(ScriptEventData* data, const char* key, float value) {
    if (!data || data->count >= 16) return;
    int i = data->count++;
    data->keys[i] = key;
    data->values[i] = value;
    data->strings[i] = nullptr;
}

void reflex_event_data_add_string(ScriptEventData* data, const char* key, const char* value) {
    if (!data || data->count >= 16) return;
    int i = data->count++;
    data->keys[i] = key;
    data->values[i] = 0.0f;
    data->strings[i] = value;
}

void reflex_send_event(ReflexScriptEngine* engine,
                       int vehicle_id,
                       const char* event_name,
                       const ScriptEventData* data) {
    if (!engine || !engine->valid || !event_name) return;

    // Build data table from ScriptEventData
    sol::table event_data = engine->lua.create_table();
    if (data) {
        for (int i = 0; i < data->count; i++) {
            if (data->keys[i]) {
                if (data->strings[i]) {
                    event_data[data->keys[i]] = data->strings[i];
                } else {
                    event_data[data->keys[i]] = data->values[i];
                }
            }
        }
    }

    // Call master.on_event(vehicle_id, event_name, data)
    sol::protected_function on_event_fn = engine->master["on_event"];
    if (!on_event_fn.valid()) {
        // master.on_event not implemented yet - silent fail
        return;
    }

    sol::protected_function_result result = on_event_fn(vehicle_id, event_name, event_data);
    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] on_event error: " << err.what() << std::endl;
    }
}

} // extern "C"