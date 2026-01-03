--[[
    Master Script - Orchestrates all vehicle scripts

    This script is loaded ONCE by C++. It manages per-vehicle script instances,
    ensuring each vehicle has its own isolated script environment.

    Architecture:
    - C++ loads master.lua once
    - C++ calls master functions: attach_script(), update(), detach_script()
    - Master uses loadfile() to create isolated script instances per vehicle
    - Modules (abs, tcs) are singletons via require(), stateless
    - Vehicle scripts have their own local state, no collision between vehicles

    C++ API:
    - master.attach_script(vehicle_id, script_name, script_path, config) -> bool
    - master.update(vehicle_id, ctx)
    - master.detach_script(vehicle_id, script_name)
    - master.reload_all()

    Turn/Test API (delegates to turn_executor script if attached):
    - master.start_turn(vehicle_id, maneuver_type, direction, options)
    - master.end_turn(vehicle_id)
    - master.is_turn_active(vehicle_id)
    - master.start_test_sequence(vehicle_id, sequence_name, options)
    - master.stop_test_sequence()
    - master.is_test_running()
]]

local master = {}

-- Vehicle data: vehicle_id -> { scripts = { name -> script_data }, state = {} }
-- Each script_data = { env, update_fn, init_fn, destroy_fn, script_path, config, name }
local vehicles = {}

-- Create isolated environment for a script instance
local function create_script_env()
    local env = {}
    setmetatable(env, { __index = _G })
    return env
end

-- Get or create vehicle entry
local function get_vehicle(vehicle_id)
    if not vehicles[vehicle_id] then
        vehicles[vehicle_id] = {
            scripts = {},
            state = {},
        }
    end
    return vehicles[vehicle_id]
end

-- ============================================================================
-- SCRIPT ATTACHMENT
-- ============================================================================

-- Attach a script to a vehicle (creates new isolated instance)
-- script_name: identifier like "freestyle_assist" or "turn_executor"
function master.attach_script(vehicle_id, script_name, script_path, config)
    local vehicle = get_vehicle(vehicle_id)

    -- Check if this script name is already attached
    if vehicle.scripts[script_name] then
        print(string.format("[Master] Vehicle %d already has '%s', replacing", vehicle_id, script_name))
        master.detach_script(vehicle_id, script_name)
    end

    -- Load the script file
    local chunk, err = loadfile(script_path)
    if not chunk then
        print(string.format("[Master] Failed to load '%s': %s", script_path, tostring(err)))
        return false
    end

    -- Create isolated environment
    local env = create_script_env()
    env.config = config or {}

    -- Set environment and execute
    setfenv(chunk, env)
    local ok, err = pcall(chunk)
    if not ok then
        print(string.format("[Master] Error executing '%s': %s", script_path, tostring(err)))
        return false
    end

    -- Verify update function exists
    if type(env.update) ~= "function" then
        print(string.format("[Master] Script '%s' missing update() function", script_path))
        return false
    end

    -- Store script data
    vehicle.scripts[script_name] = {
        name = script_name,
        env = env,
        update_fn = env.update,
        init_fn = env.init,
        destroy_fn = env.destroy,
        script_path = script_path,
        config = config or {},
    }

    print(string.format("[Master] Attached '%s' to vehicle %d", script_name, vehicle_id))

    -- Call init if defined
    if env.init then
        local init_ctx = {
            id = vehicle_id,
            config = config or {},
            state = vehicle.state,
        }
        local ok, err = pcall(env.init, init_ctx)
        if not ok then
            print(string.format("[Master] init() error for %s on vehicle %d: %s", script_name, vehicle_id, tostring(err)))
        end
    end

    return true
end

-- Detach a specific script from a vehicle
function master.detach_script(vehicle_id, script_name)
    local vehicle = vehicles[vehicle_id]
    if not vehicle or not vehicle.scripts[script_name] then
        return
    end

    local script = vehicle.scripts[script_name]

    -- Call destroy if defined
    if script.destroy_fn then
        local ok, err = pcall(script.destroy_fn, { id = vehicle_id })
        if not ok then
            print(string.format("[Master] destroy() error for %s: %s", script_name, tostring(err)))
        end
    end

    vehicle.scripts[script_name] = nil
    print(string.format("[Master] Detached '%s' from vehicle %d", script_name, vehicle_id))
end

-- Detach all scripts from a vehicle
function master.detach_all(vehicle_id)
    local vehicle = vehicles[vehicle_id]
    if not vehicle then return end

    for script_name, _ in pairs(vehicle.scripts) do
        master.detach_script(vehicle_id, script_name)
    end
    vehicles[vehicle_id] = nil
end

-- ============================================================================
-- UPDATE
-- ============================================================================

function master.update(vehicle_id, ctx)
    local vehicle = vehicles[vehicle_id]
    if not vehicle then return end

    -- Store telemetry for other functions
    vehicle.state.last_telemetry = ctx.telemetry

    -- Inject shared state into context
    ctx.state = vehicle.state

    -- Update all attached scripts
    for script_name, script in pairs(vehicle.scripts) do
        ctx.config = script.config

        local ok, err = pcall(script.update_fn, ctx)
        if not ok then
            print(string.format("[Master] update() error for %s on vehicle %d: %s",
                script_name, vehicle_id, tostring(err)))
        end
    end
end

-- ============================================================================
-- TURN-BASED MANEUVER CONTROL (delegates to turn_executor script)
-- ============================================================================

-- Helper to get turn_executor script for a vehicle
local function get_turn_executor(vehicle_id)
    local vehicle = vehicles[vehicle_id]
    if not vehicle then return nil end
    return vehicle.scripts["turn_executor"]
end

-- Start a turn with a declared maneuver
function master.start_turn(vehicle_id, maneuver_type, direction, options)
    local executor = get_turn_executor(vehicle_id)
    if not executor then
        print(string.format("[Master] start_turn: no turn_executor for vehicle %d", vehicle_id))
        return false
    end

    local start_turn_fn = executor.env.start_turn
    if not start_turn_fn then
        print("[Master] turn_executor missing start_turn function")
        return false
    end

    local vehicle = vehicles[vehicle_id]
    local ctx = {
        id = vehicle_id,
        state = vehicle.state,
        telemetry = vehicle.state.last_telemetry or {},
        controls = { steering = 0, throttle = 0, brake = 0, handbrake = 0 },
        config = executor.config,
    }

    local ok, result = pcall(start_turn_fn, ctx, maneuver_type, direction, options)
    if not ok then
        print(string.format("[Master] start_turn error: %s", tostring(result)))
        return false
    end
    return result
end

-- End the current turn and get result
function master.end_turn(vehicle_id)
    local executor = get_turn_executor(vehicle_id)
    if not executor then
        return { success = false, reason = "no_turn_executor" }
    end

    local end_turn_fn = executor.env.end_turn
    if not end_turn_fn then
        return { success = false, reason = "missing_end_turn_function" }
    end

    local vehicle = vehicles[vehicle_id]
    local ctx = {
        id = vehicle_id,
        state = vehicle.state,
        telemetry = vehicle.state.last_telemetry or {},
        controls = {},
        config = executor.config,
    }

    local ok, result = pcall(end_turn_fn, ctx)
    if not ok then
        print(string.format("[Master] end_turn error: %s", tostring(result)))
        return { success = false, reason = "error" }
    end
    return result
end

-- Check if a turn is active
function master.is_turn_active(vehicle_id)
    local executor = get_turn_executor(vehicle_id)
    if not executor then return false end

    local is_active_fn = executor.env.is_turn_active
    if not is_active_fn then return false end

    local vehicle = vehicles[vehicle_id]
    local ctx = {
        id = vehicle_id,
        state = vehicle.state,
    }

    local ok, result = pcall(is_active_fn, ctx)
    if not ok then return false end
    return result
end

-- ============================================================================
-- TEST SEQUENCE CONTROL (delegates to turn_executor script)
-- ============================================================================

function master.start_test_sequence(vehicle_id, sequence_name, options)
    local executor = get_turn_executor(vehicle_id)
    if not executor then
        print(string.format("[Master] start_test_sequence: no turn_executor for vehicle %d", vehicle_id))
        return false
    end

    local start_fn = executor.env.start_test_sequence
    if not start_fn then
        print("[Master] turn_executor missing start_test_sequence function")
        return false
    end

    local vehicle = vehicles[vehicle_id]
    local ctx = {
        id = vehicle_id,
        state = vehicle.state,
        telemetry = vehicle.state.last_telemetry or {},
        controls = {},
        config = executor.config,
    }

    local ok, result = pcall(start_fn, ctx, sequence_name, options)
    if not ok then
        print(string.format("[Master] start_test_sequence error: %s", tostring(result)))
        return false
    end
    return result
end

function master.stop_test_sequence()
    -- Find any vehicle with a turn_executor and call stop
    for vehicle_id, vehicle in pairs(vehicles) do
        local executor = vehicle.scripts["turn_executor"]
        if executor and executor.env.stop_test_sequence then
            pcall(executor.env.stop_test_sequence)
            return
        end
    end
end

function master.is_test_running()
    -- Check any turn_executor for running test
    for vehicle_id, vehicle in pairs(vehicles) do
        local executor = vehicle.scripts["turn_executor"]
        if executor and executor.env.is_test_running then
            local ok, result = pcall(executor.env.is_test_running)
            if ok and result then return true end
        end
    end
    return false
end

-- ============================================================================
-- EVENT SYSTEM
-- ============================================================================

-- Called by C++ to send events to scripts
-- vehicle_id: target vehicle
-- event_name: string identifier like "execute_maneuver", "cancel_maneuver"
-- data: table with event-specific data
function master.on_event(vehicle_id, event_name, data)
    local vehicle = vehicles[vehicle_id]
    if not vehicle then
        print(string.format("[Master] on_event: no vehicle %d", vehicle_id))
        return
    end

    -- Build event context
    local event_ctx = {
        vehicle_id = vehicle_id,
        event = event_name,
        data = data or {},
        state = vehicle.state,
        telemetry = vehicle.state.last_telemetry or {},
    }

    -- Broadcast to all scripts attached to this vehicle
    local handled = false
    for script_name, script in pairs(vehicle.scripts) do
        if script.env.on_event then
            local ok, result = pcall(script.env.on_event, event_ctx)
            if not ok then
                print(string.format("[Master] on_event error for %s: %s", script_name, tostring(result)))
            elseif result then
                -- Script returned true, meaning it handled the event
                handled = true
            end
        end
    end

    if not handled then
        print(string.format("[Master] Event '%s' not handled by any script", event_name))
    end

    return handled
end

-- ============================================================================
-- INPUT HANDLING
-- ============================================================================

-- Called by C++ when keys are pressed
-- keys_pressed: table where keys_pressed[key_code] = true for pressed keys
-- selected_vehicle_id: currently selected vehicle (-1 if none)
function master.on_input(keys_pressed, selected_vehicle_id)
    if selected_vehicle_id < 0 then
        -- No vehicle selected, nothing to do
        return
    end

    local vehicle = vehicles[selected_vehicle_id]
    if not vehicle then return end

    -- Build input context
    local input_ctx = {
        vehicle_id = selected_vehicle_id,
        keys = keys_pressed,
        state = vehicle.state,
        telemetry = vehicle.state.last_telemetry or {},
    }

    -- Broadcast to all scripts attached to this vehicle
    for script_name, script in pairs(vehicle.scripts) do
        if script.env.on_input then
            local ok, err = pcall(script.env.on_input, input_ctx)
            if not ok then
                print(string.format("[Master] on_input error for %s: %s", script_name, tostring(err)))
            end
        end
    end
end

-- ============================================================================
-- UTILITY
-- ============================================================================

function master.reload_all()
    print("[Master] Reloading all scripts...")

    -- Collect attachments
    local attachments = {}
    for vehicle_id, vehicle in pairs(vehicles) do
        for script_name, script in pairs(vehicle.scripts) do
            table.insert(attachments, {
                vehicle_id = vehicle_id,
                script_name = script_name,
                script_path = script.script_path,
                config = script.config,
            })
        end
    end

    -- Clear module cache
    for name, _ in pairs(package.loaded) do
        if name:match("^modules/") then
            package.loaded[name] = nil
            print(string.format("[Master] Cleared module cache: %s", name))
        end
    end

    -- Detach all
    for vehicle_id, _ in pairs(vehicles) do
        master.detach_all(vehicle_id)
    end

    -- Re-attach
    local success = 0
    for _, att in ipairs(attachments) do
        if master.attach_script(att.vehicle_id, att.script_name, att.script_path, att.config) then
            success = success + 1
        end
    end

    print(string.format("[Master] Reloaded %d/%d scripts", success, #attachments))
    return success
end

function master.get_vehicle_count()
    local count = 0
    for _ in pairs(vehicles) do count = count + 1 end
    return count
end

print("[Master] Script orchestrator loaded")

return master