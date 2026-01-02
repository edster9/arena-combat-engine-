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
                          const char* script_path,
                          const char* config_keys[],
                          float config_values[],
                          int config_count) {
    if (!engine || !engine->valid || !script_path) return false;

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

    // Call master.attach_script(vehicle_id, script_path, config)
    sol::protected_function attach_fn = engine->master["attach_script"];
    sol::protected_function_result result = attach_fn(vehicle_id, full_path, config);

    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] attach_script error: " << err.what() << std::endl;
        return false;
    }

    bool success = result.get<bool>();
    return success;
}

void reflex_detach_script(ReflexScriptEngine* engine, int vehicle_id) {
    if (!engine || !engine->valid) return;

    sol::protected_function detach_fn = engine->master["detach_script"];
    sol::protected_function_result result = detach_fn(vehicle_id);

    if (!result.valid()) {
        sol::error err = result;
        std::cerr << "[Reflex] detach_script error: " << err.what() << std::endl;
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

} // extern "C"