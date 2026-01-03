--[[
    Turn Executor Script

    Executes declared maneuvers during the turn-based execution phase.
    Each turn lasts 1 second and the vehicle attempts to complete
    the declared maneuver using physics-based control.

    Flow:
    1. User presses X to start a test maneuver
    2. update() is called each frame during the 1-second turn
    3. Maneuver auto-evaluates when duration elapses

    Input Bindings:
    - X: Start a test maneuver (bend_45 right)

    Context (ctx) structure:
    - ctx.id          : Entity ID (vehicle_id)
    - ctx.dt          : Delta time
    - ctx.state       : Per-entity persistent state
    - ctx.config      : Script configuration
    - ctx.telemetry   : Read-only vehicle state
    - ctx.controls    : Control outputs (steering, throttle, brake)
]]

-- Key codes (from platform.h)
local KEY_X = 27

-- Load the maneuver executor module
local maneuver = require("modules/maneuver")

-- Load test sequence module (for automated testing)
local test_sequence = require("modules/test_sequence")

-- Turn state (per-entity, stored in ctx.state)
local function init_turn_state(ctx)
    if not ctx.state.turn then
        ctx.state.turn = {
            active = false,
            maneuver_type = nil,
            direction = nil,
            elapsed = 0,
            duration = 1.0,
            result = nil,
            speed_change = "maintain",  -- "accelerate", "decelerate", "hard_brake"
        }
    end
end

-- ============================================================================
-- TURN CONTROL API (called by C++ or master script)
-- ============================================================================

-- Start a turn with the declared maneuver
-- Called at the beginning of the execution phase
function start_turn(ctx, maneuver_type, direction, options)
    init_turn_state(ctx)

    local state = ctx.state.turn
    state.active = true
    state.maneuver_type = maneuver_type
    state.direction = direction or "center"
    state.elapsed = 0
    state.duration = options and options.duration or 1.0
    state.speed_change = options and options.speed_change or "maintain"
    state.result = nil

    -- Start the maneuver
    maneuver.start(ctx, maneuver_type, direction, {
        duration = state.duration,
        speed_change = state.speed_change
    })

    print(string.format("[Turn] Entity %d: Starting turn - %s %s",
        ctx.id, direction, maneuver_type))

    return true
end

-- End the turn and evaluate the result
-- Called at the end of the execution phase
function end_turn(ctx)
    init_turn_state(ctx)

    local state = ctx.state.turn
    if not state.active then
        return { success = false, reason = "no_active_turn" }
    end

    -- Evaluate the maneuver
    local result = maneuver.evaluate(ctx)

    state.active = false
    state.result = result

    print(string.format("[Turn] Entity %d: Turn ended - %s (%s)",
        ctx.id, result.level or "unknown", result.success and "SUCCESS" or "FAILED"))

    return result
end

-- Check if a turn is currently active
function is_turn_active(ctx)
    init_turn_state(ctx)
    return ctx.state.turn.active
end

-- Get current turn progress (0.0 to 1.0)
function get_turn_progress(ctx)
    init_turn_state(ctx)
    local state = ctx.state.turn
    if not state.active then return 0 end
    return math.min(state.elapsed / state.duration, 1.0)
end

-- Get last turn result
function get_turn_result(ctx)
    init_turn_state(ctx)
    return ctx.state.turn.result
end

-- ============================================================================
-- TEST SEQUENCE API (for automated maneuver testing)
-- ============================================================================

function start_test_sequence(ctx, sequence_name, options)
    init_turn_state(ctx)

    if test_sequence.is_active() then
        print("[turn_executor] Test sequence already running, stopping first")
        test_sequence.stop()
    end

    options = options or {}
    options.entity = ctx.id

    return test_sequence.start(sequence_name or "smoke_test", options)
end

function stop_test_sequence()
    test_sequence.stop()
end

function is_test_running()
    return test_sequence.is_active()
end

-- ============================================================================
-- MAIN UPDATE (called each frame during turn execution)
-- ============================================================================

function update(ctx)
    init_turn_state(ctx)

    -- Run test sequence if active for this entity
    if test_sequence.is_active() and ctx.id == test_sequence.state.target_entity then
        test_sequence.update(ctx)
    end

    local state = ctx.state.turn
    if not state.active then
        -- No active turn - pass through controls unchanged
        return
    end

    -- Update timing
    state.elapsed = state.elapsed + ctx.dt

    -- Update the maneuver executor
    maneuver.update(ctx)

    -- Handle speed change
    apply_speed_change(ctx, state.speed_change)

    -- Check if turn is complete
    if state.elapsed >= state.duration then
        local result = maneuver.evaluate(ctx)
        print(string.format("[Turn] Turn complete: %s",
            result.success and "SUCCESS" or "FAILED"))
        state.time_complete = true
        state.active = false  -- Mark turn as inactive so C++ can detect completion
        state.result = result
    end
end

-- ============================================================================
-- SPEED CHANGE HANDLING
-- ============================================================================

function apply_speed_change(ctx, speed_change)
    local controls = ctx.controls
    local tel = ctx.telemetry
    local current_speed = tel.speed_ms or 0

    if speed_change == "accelerate" then
        -- Target higher throttle
        controls.throttle = math.max(controls.throttle or 0, 0.7)

    elseif speed_change == "decelerate" then
        -- Light braking
        controls.throttle = 0
        controls.brake = 0.3

    elseif speed_change == "hard_brake" then
        -- Heavy braking
        controls.throttle = 0
        controls.brake = 0.8

    -- "maintain" - let maneuver control handle it
    end
end

-- ============================================================================
-- INIT (called when script is attached to a vehicle)
-- ============================================================================

function init(ctx)
    init_turn_state(ctx)
    print(string.format("[Turn] Executor attached to vehicle %d", ctx.id))
end

-- ============================================================================
-- DESTROY (called when script is detached)
-- ============================================================================

function destroy(ctx)
    print(string.format("[Turn] Executor detached from vehicle %d", ctx.id))
end

-- ============================================================================
-- INPUT HANDLING (called when keys are pressed)
-- ============================================================================

function on_input(input_ctx)
    local keys = input_ctx.keys
    local vehicle_id = input_ctx.vehicle_id

    -- X: Start a test maneuver (bend_45 right)
    if keys[KEY_X] then
        -- Check if a turn is already active
        if input_ctx.state.turn and input_ctx.state.turn.active then
            print("[Turn] Turn already in progress, ignoring X")
            return
        end

        -- Build context for start_turn
        local ctx = {
            id = vehicle_id,
            state = input_ctx.state,
            telemetry = input_ctx.telemetry,
            controls = { steering = 0, throttle = 0, brake = 0, handbrake = 0 },
            config = config or {},
        }

        local test_maneuver = "bend_45"
        local test_direction = "right"
        local test_duration = 1.0

        print(string.format("[Turn] X pressed: Starting %s %s (%.1fs)",
            test_direction, test_maneuver, test_duration))

        start_turn(ctx, test_maneuver, test_direction, { duration = test_duration })
    end
end

-- ============================================================================
-- EVENT HANDLING (called when C++ sends events)
-- ============================================================================

-- Supported events:
--   execute_maneuver: Start a single-phase turn
--     { type="bend_45", direction="right", duration=1.0, speed_change="maintain" }
--   cancel_maneuver: Cancel active turn
function on_event(event_ctx)
    local event = event_ctx.event
    local data = event_ctx.data
    local vehicle_id = event_ctx.vehicle_id

    if event == "execute_maneuver" then
        -- Check if a turn is already active
        if event_ctx.state.turn and event_ctx.state.turn.active then
            print("[Turn] Turn already in progress, ignoring execute_maneuver")
            return false
        end

        local maneuver_type = data.type or "straight"
        local direction = data.direction or "center"
        local duration = data.duration or 1.0
        local speed_change = data.speed_change or "maintain"

        -- Build context for start_turn
        local ctx = {
            id = vehicle_id,
            state = event_ctx.state,
            telemetry = event_ctx.telemetry,
            controls = { steering = 0, throttle = 0, brake = 0, handbrake = 0 },
            config = config or {},
        }

        print(string.format("[Turn] Event: Starting %s %s (%.1fs, %s)",
            direction, maneuver_type, duration, speed_change))

        start_turn(ctx, maneuver_type, direction, {
            duration = duration,
            speed_change = speed_change
        })
        return true  -- Event handled

    elseif event == "cancel_maneuver" then
        if event_ctx.state.turn and event_ctx.state.turn.active then
            event_ctx.state.turn.active = false
            print("[Turn] Maneuver cancelled")
            return true
        end
        return false

    end

    -- Event not handled by this script
    return false
end