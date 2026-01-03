--[[
    Maneuver Executor Module

    This module executes maneuvers by applying controls to achieve goal states.
    Each maneuver is defined by goals (heading change, lateral offset, etc.)
    and the script applies steering/throttle/brake to reach those goals.

    Usage:
        local maneuver = require("modules/maneuver")

        -- Start a maneuver at turn begin
        maneuver.start(ctx, "bend_45", "left")

        -- Update each frame during turn execution
        maneuver.update(ctx)

        -- Check result at turn end
        local result = maneuver.evaluate(ctx)

    Context (ctx) structure:
        ctx.state           - Per-entity state storage
        ctx.telemetry       - Vehicle telemetry (position, heading, speed, etc.)
        ctx.controls        - Controls to modify (steering, throttle, brake)
        ctx.dt              - Delta time
        ctx.turn_time       - Time elapsed in current turn (0.0 to 1.0)
        ctx.turn_duration   - Turn duration in seconds (default 1.0)
]]

local maneuver = {}

-- Constants
local DEG_TO_RAD = math.pi / 180
local RAD_TO_DEG = 180 / math.pi
local INCH_TO_METERS = 4.572

-- Maneuver state
maneuver.active = {}  -- Per-entity maneuver state

-- ============================================================================
-- UTILITY FUNCTIONS
-- ============================================================================

-- Normalize angle to -180 to 180
local function normalize_angle(deg)
    while deg > 180 do deg = deg - 360 end
    while deg < -180 do deg = deg + 360 end
    return deg
end

-- Get signed angle difference (target - current), normalized
local function angle_diff(target, current)
    return normalize_angle(target - current)
end

-- Clamp value between min and max
local function clamp(val, min_val, max_val)
    return math.max(min_val, math.min(max_val, val))
end

-- Linear interpolation
local function lerp(a, b, t)
    return a + (b - a) * t
end

-- Apply speed controls based on speed_change mode
-- Returns throttle, brake values
local function get_speed_controls(state)
    local mode = state.speed_change or "maintain"

    if mode == "accelerate" then
        return 0.5, 0  -- Throttle to speed up
    elseif mode == "decelerate" then
        return 0, 0.3  -- Brake to slow down
    else  -- "maintain"
        return 0, 0    -- Coast - no throttle, no brake
    end
end

-- ============================================================================
-- MANEUVER STATE MANAGEMENT
-- ============================================================================

-- Initialize maneuver state for an entity
function maneuver.init_state(ctx)
    ctx.state.maneuver = {
        active = false,
        type = nil,
        direction = nil,  -- 1 for right, -1 for left

        -- Captured at start
        start_heading = 0,
        start_position = {x = 0, y = 0, z = 0},
        start_speed = 0,

        -- Goals
        target_heading_delta = 0,
        target_lateral_offset = 0,
        target_speed_delta = 0,

        -- Tolerances
        heading_tolerance = 10,
        position_tolerance = 2.0,

        -- Timing
        elapsed = 0,
        duration = 1.0,

        -- Execution phase (for multi-phase maneuvers)
        phase = "execute",
        phase_time = 0,

        -- Result
        result = "pending"
    }
end

-- Start a maneuver
function maneuver.start(ctx, maneuver_type, direction, options)
    if not ctx.state.maneuver then
        maneuver.init_state(ctx)
    end

    local state = ctx.state.maneuver
    local tel = ctx.telemetry

    -- Direction: "left" = -1, "right" = 1
    local dir_sign = 1
    if direction == "left" then dir_sign = -1 end

    -- Capture start state
    state.active = true
    state.type = maneuver_type
    state.direction = dir_sign
    state.start_heading = tel.heading or 0  -- Already in degrees from C++
    state.start_position = {
        x = tel.position and tel.position.x or 0,
        y = tel.position and tel.position.y or 0,
        z = tel.position and tel.position.z or 0
    }
    state.start_speed = tel.speed_ms or 0
    state.elapsed = 0
    state.duration = options and options.duration or 1.0
    state.phase = "execute"
    state.phase_time = 0
    state.result = "pending"

    -- Store speed change mode: "accelerate", "maintain", "decelerate"
    state.speed_change = options and options.speed_change or "maintain"

    -- Set goals based on maneuver type
    maneuver.set_goals(state, maneuver_type, dir_sign, options)

    print(string.format("[Maneuver] Entity %d: Started %s %s (heading delta: %.0f°)",
        ctx.id, direction or "center", maneuver_type, state.target_heading_delta))
end

-- Set goals based on maneuver type
function maneuver.set_goals(state, mtype, direction, options)
    -- Default goals
    state.target_heading_delta = 0
    state.target_lateral_offset = 0
    state.heading_tolerance = 10
    state.position_tolerance = 2.0

    if mtype == "straight" then
        state.target_heading_delta = 0
        state.target_lateral_offset = 0
        state.heading_tolerance = 5

    elseif mtype == "drift" then
        state.target_heading_delta = 0
        state.target_lateral_offset = 0.25 * INCH_TO_METERS * direction
        state.heading_tolerance = 5

    elseif mtype == "steep_drift" then
        state.target_heading_delta = 0
        state.target_lateral_offset = 0.5 * INCH_TO_METERS * direction
        state.heading_tolerance = 8

    elseif mtype == "bend_15" then
        state.target_heading_delta = 15 * direction
        state.heading_tolerance = 5

    elseif mtype == "bend_30" then
        state.target_heading_delta = 30 * direction
        state.heading_tolerance = 8

    elseif mtype == "bend_45" then
        state.target_heading_delta = 45 * direction
        state.heading_tolerance = 10

    elseif mtype == "bend_60" then
        state.target_heading_delta = 60 * direction
        state.heading_tolerance = 12

    elseif mtype == "bend_75" then
        state.target_heading_delta = 75 * direction
        state.heading_tolerance = 15

    elseif mtype == "bend_90" then
        state.target_heading_delta = 90 * direction
        state.heading_tolerance = 15

    elseif mtype == "swerve" then
        -- Drift then straighten
        state.target_heading_delta = 0  -- End same heading
        state.target_lateral_offset = 0.5 * INCH_TO_METERS * direction
        state.heading_tolerance = 10
        state.is_swerve = true

    elseif mtype == "controlled_skid" then
        local skid_dist = options and options.skid_distance or 1  -- 1-4 quarter inches
        state.target_heading_delta = (15 + skid_dist * 10) * direction
        state.target_lateral_offset = skid_dist * 0.25 * INCH_TO_METERS * direction
        state.heading_tolerance = 20
        state.allow_slip = true

    elseif mtype == "t_stop" then
        state.target_heading_delta = 90 * direction
        state.target_speed_delta = -999  -- Stop completely
        state.heading_tolerance = 15
        state.is_stop = true

    elseif mtype == "pivot" then
        local pivot_angle = options and options.angle or 45
        state.target_heading_delta = pivot_angle * direction
        state.heading_tolerance = 10
        state.is_pivot = true

    elseif mtype == "bootlegger" then
        state.target_heading_delta = 180 * direction
        state.heading_tolerance = 20
        state.position_tolerance = 5.0
        state.is_bootlegger = true
        state.phase = "initiate"
    end
end

-- ============================================================================
-- MANEUVER EXECUTION (called each frame)
-- ============================================================================

function maneuver.update(ctx)
    if not ctx.state.maneuver or not ctx.state.maneuver.active then
        return
    end

    local state = ctx.state.maneuver
    local tel = ctx.telemetry
    local controls = ctx.controls
    local dt = ctx.dt or 0.016

    -- Update timing
    state.elapsed = state.elapsed + dt
    state.phase_time = state.phase_time + dt
    local progress = clamp(state.elapsed / state.duration, 0, 1)

    -- Get current heading delta from start
    local current_heading = tel.heading or 0  -- Already in degrees from C++
    local heading_delta = angle_diff(current_heading, state.start_heading)

    -- Calculate remaining heading to achieve
    local remaining_heading = state.target_heading_delta - heading_delta

    -- Dispatch to specific maneuver executor
    local mtype = state.type

    if mtype == "straight" then
        maneuver.execute_straight(ctx, state, remaining_heading, progress)

    elseif mtype == "drift" or mtype == "steep_drift" then
        maneuver.execute_drift(ctx, state, remaining_heading, progress)

    elseif string.find(mtype, "bend_") then
        maneuver.execute_bend(ctx, state, remaining_heading, progress)

    elseif mtype == "swerve" then
        maneuver.execute_swerve(ctx, state, remaining_heading, progress)

    elseif mtype == "controlled_skid" then
        maneuver.execute_controlled_skid(ctx, state, remaining_heading, progress)

    elseif mtype == "t_stop" then
        maneuver.execute_t_stop(ctx, state, remaining_heading, progress)

    elseif mtype == "pivot" then
        maneuver.execute_pivot(ctx, state, remaining_heading, progress)

    elseif mtype == "bootlegger" then
        maneuver.execute_bootlegger(ctx, state, remaining_heading, progress)
    end
end

-- ============================================================================
-- INDIVIDUAL MANEUVER EXECUTORS
-- ============================================================================

-- STRAIGHT: Maintain heading, go straight
function maneuver.execute_straight(ctx, state, remaining_heading, progress)
    local controls = ctx.controls

    -- Simple heading correction
    local correction = clamp(remaining_heading * 0.05, -0.3, 0.3)
    controls.steering = -correction  -- Negative because positive steering = right

    -- Apply speed controls based on declared speed change
    controls.throttle, controls.brake = get_speed_controls(state)
end

-- DRIFT: Lateral movement without heading change
function maneuver.execute_drift(ctx, state, remaining_heading, progress)
    local controls = ctx.controls
    local tel = ctx.telemetry

    -- Calculate lateral offset achieved
    local dx = (tel.position and tel.position.x or 0) - state.start_position.x
    local dz = (tel.position and tel.position.z or 0) - state.start_position.z

    -- Project onto perpendicular of start heading
    local start_rad = state.start_heading * DEG_TO_RAD
    local perp_x = -math.sin(start_rad)
    local perp_z = math.cos(start_rad)
    local lateral_achieved = dx * perp_x + dz * perp_z

    local lateral_remaining = state.target_lateral_offset - lateral_achieved
    local lateral_error = lateral_remaining / INCH_TO_METERS  -- In inches

    -- Phase 1: Steer to initiate drift (first 40%)
    -- Phase 2: Counter-steer to straighten (last 60%)
    if progress < 0.4 then
        -- Initiate: steer toward target side
        controls.steering = clamp(state.direction * 0.4 + lateral_error * 0.5, -1, 1)
    else
        -- Correct: counter-steer and correct heading
        local heading_correction = clamp(-remaining_heading * 0.03, -0.5, 0.5)
        controls.steering = heading_correction - state.direction * 0.1
    end

    -- Apply speed controls based on declared speed change
    controls.throttle, controls.brake = get_speed_controls(state)
end

-- BEND: Turn with heading change
function maneuver.execute_bend(ctx, state, remaining_heading, progress)
    local controls = ctx.controls
    local tel = ctx.telemetry

    -- Aggressive steering: full lock until within 20° of target, then proportional
    -- This ensures we actually achieve the turn angle at speed
    local steer_amount
    if math.abs(remaining_heading) > 20 then
        -- Far from target: full steering
        steer_amount = (remaining_heading > 0) and 1.0 or -1.0
    else
        -- Close to target: proportional easing (remaining/20 gives 0-1 range)
        steer_amount = remaining_heading / 20.0
    end

    -- Clamp steering (negative because positive steering input = right turn in physics)
    controls.steering = clamp(-steer_amount, -1, 1)

    -- Apply speed controls based on declared speed change
    local base_throttle, base_brake = get_speed_controls(state)

    -- For sharp turns, we need to brake regardless of speed_change to maintain control
    local target_angle = math.abs(state.target_heading_delta)
    if target_angle > 60 then
        -- Sharp turn - always brake lightly for safety
        controls.throttle = math.min(base_throttle, 0.1)
        controls.brake = math.max(base_brake, 0.2)
    elseif target_angle > 45 then
        controls.throttle = math.min(base_throttle, 0.2)
        controls.brake = math.max(base_brake, 0.1)
    else
        -- Gentle turn - use declared speed change
        controls.throttle = base_throttle
        controls.brake = base_brake
    end
end

-- SWERVE: Drift then opposite bend
function maneuver.execute_swerve(ctx, state, remaining_heading, progress)
    local controls = ctx.controls

    if progress < 0.5 then
        -- Phase 1: Turn into the swerve
        controls.steering = state.direction * 0.6
    else
        -- Phase 2: Counter-steer to straighten
        local heading_correction = clamp(-remaining_heading * 0.04, -0.8, 0.8)
        controls.steering = heading_correction
    end

    -- Apply speed controls based on declared speed change
    controls.throttle, controls.brake = get_speed_controls(state)
end

-- CONTROLLED_SKID: Powerslide with oversteer
function maneuver.execute_controlled_skid(ctx, state, remaining_heading, progress)
    local controls = ctx.controls
    local tel = ctx.telemetry

    if progress < 0.3 then
        -- Initiate: turn hard and lift throttle
        controls.steering = state.direction * 0.9
        controls.throttle = 0.1
        controls.brake = 0.3
    elseif progress < 0.7 then
        -- Slide: maintain steering, modulate throttle for slip
        controls.steering = state.direction * 0.7
        controls.throttle = 0.5  -- Power to maintain slide
        controls.brake = 0
    else
        -- Exit: counter-steer and power out
        local heading_correction = clamp(-remaining_heading * 0.03, -1, 1)
        controls.steering = heading_correction
        controls.throttle = 0.6
        controls.brake = 0
    end
end

-- T_STOP: Emergency stop with 90° rotation
function maneuver.execute_t_stop(ctx, state, remaining_heading, progress)
    local controls = ctx.controls
    local tel = ctx.telemetry
    local speed = tel.speed_ms or 0

    -- Full lock steering
    controls.steering = state.direction * 1.0

    -- Heavy braking throughout
    controls.throttle = 0
    controls.brake = 0.9

    -- E-brake for rotation (if available)
    if controls.handbrake then
        controls.handbrake = 0.8
    end

    -- Reduce steering as we slow down
    if speed < 2 then
        controls.steering = state.direction * 0.5
    end
end

-- PIVOT: Low-speed rotation around rear corner
function maneuver.execute_pivot(ctx, state, remaining_heading, progress)
    local controls = ctx.controls

    -- Full steering lock
    controls.steering = state.direction * 1.0

    -- Creep throttle
    controls.throttle = 0.15
    controls.brake = 0

    -- Ease off as we approach target
    if math.abs(remaining_heading) < 10 then
        controls.steering = state.direction * 0.5
        controls.throttle = 0.1
    end
end

-- BOOTLEGGER: 180° J-turn (the hardest maneuver)
function maneuver.execute_bootlegger(ctx, state, remaining_heading, progress)
    local controls = ctx.controls
    local tel = ctx.telemetry

    if state.phase == "initiate" and state.phase_time < 0.3 then
        -- Phase 1: Initiate spin with e-brake and full steering
        controls.steering = state.direction * 1.0
        controls.throttle = 0
        controls.brake = 0.3
        if controls.handbrake then
            controls.handbrake = 1.0
        end

        if state.phase_time >= 0.25 then
            state.phase = "rotate"
            state.phase_time = 0
        end

    elseif state.phase == "rotate" then
        -- Phase 2: Maintain rotation, modulate e-brake
        controls.steering = state.direction * 0.8
        controls.throttle = 0
        controls.brake = 0.1
        if controls.handbrake then
            controls.handbrake = 0.5
        end

        -- Check if we've rotated enough
        if math.abs(remaining_heading) < 45 then
            state.phase = "exit"
            state.phase_time = 0
        end

    elseif state.phase == "exit" then
        -- Phase 3: Counter-steer and power out
        local heading_correction = clamp(-remaining_heading * 0.03, -1, 1)
        controls.steering = heading_correction
        controls.throttle = 0.7
        controls.brake = 0
        if controls.handbrake then
            controls.handbrake = 0
        end
    end
end

-- ============================================================================
-- EVALUATION (called at turn end)
-- ============================================================================

function maneuver.evaluate(ctx)
    if not ctx.state.maneuver or not ctx.state.maneuver.active then
        return { success = false, reason = "no_maneuver" }
    end

    local state = ctx.state.maneuver
    local tel = ctx.telemetry

    -- Calculate final heading delta
    local current_heading = tel.heading or 0  -- Already in degrees from C++
    local heading_delta = angle_diff(current_heading, state.start_heading)
    local heading_error = math.abs(state.target_heading_delta - heading_delta)

    -- Calculate position error (for maneuvers with position goals)
    local dx = (tel.position and tel.position.x or 0) - state.start_position.x
    local dz = (tel.position and tel.position.z or 0) - state.start_position.z
    local position_error = math.sqrt(dx*dx + dz*dz)

    -- Evaluate success level
    local result = {
        heading_error = heading_error,
        position_error = position_error,
        heading_delta_achieved = heading_delta,
        target_heading_delta = state.target_heading_delta
    }

    if heading_error <= state.heading_tolerance * 0.5 then
        result.success = true
        result.level = "perfect"
    elseif heading_error <= state.heading_tolerance * 0.75 then
        result.success = true
        result.level = "good"
    elseif heading_error <= state.heading_tolerance then
        result.success = true
        result.level = "partial"
    else
        result.success = false
        result.level = "failed"
        result.reason = string.format("heading_error=%.1f > tolerance=%.1f",
            heading_error, state.heading_tolerance)
    end

    print(string.format("[Maneuver] Entity %d: %s %s - %s (error: %.1f°)",
        ctx.id, state.type,
        result.success and "SUCCESS" or "FAILED",
        result.level, heading_error))

    -- Mark maneuver complete
    state.active = false
    state.result = result.level

    return result
end

-- Cancel active maneuver
function maneuver.cancel(ctx)
    if ctx.state.maneuver then
        ctx.state.maneuver.active = false
        ctx.state.maneuver.result = "cancelled"
    end
end

-- Check if maneuver is active
function maneuver.is_active(ctx)
    return ctx.state.maneuver and ctx.state.maneuver.active
end

-- Get current maneuver type
function maneuver.get_type(ctx)
    if ctx.state.maneuver and ctx.state.maneuver.active then
        return ctx.state.maneuver.type
    end
    return nil
end

return maneuver