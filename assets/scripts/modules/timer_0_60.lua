--[[
    Performance Timer Module

    Measures standard automotive performance benchmarks:
    - 0-60 mph (0-96.6 km/h) - US standard
    - 0-100 km/h (0-62.1 mph) - European standard
    - 0-100 mph - extended acceleration
    - Quarter mile (402m) time and trap speed
    - Eighth mile (201m) time and trap speed

    Automatically resets when vehicle comes to a stop.

    Usage:
        local timer = require("modules/timer_0_60")
        timer.update(ctx)
]]

local timer = {}

-- Speed benchmarks (in m/s)
local BENCHMARKS = {
    { name = "0-60 mph",   speed_ms = 26.8224 },  -- 60 mph
    { name = "0-100 km/h", speed_ms = 27.7778 },  -- 100 km/h = 62.14 mph
    { name = "0-100 mph",  speed_ms = 44.704  },  -- 100 mph
}

-- Distance benchmarks (in meters)
local DISTANCE_MARKS = {
    { name = "1/8 mile", distance_m = 201.168 },  -- 660 feet
    { name = "1/4 mile", distance_m = 402.336 },  -- 1320 feet
}

local STOP_THRESHOLD = 0.5  -- m/s - consider stopped below this

-- Per-entity state stored in ctx.state.perf_timer
-- Fields:
--   .timing        : boolean - currently timing
--   .start_time    : number - when timing started
--   .start_pos     : vec3 - starting position
--   .completed     : table - which benchmarks have been reported
--   .run_complete  : boolean - all benchmarks done

function timer.update(ctx)
    local telemetry = ctx.telemetry
    local controls = ctx.controls
    local speed = telemetry.speed_ms or 0
    local throttle = controls.throttle or 0
    local now = telemetry.time or 0

    -- Initialize per-entity state
    if not ctx.state.perf_timer then
        ctx.state.perf_timer = {
            timing = false,
            start_time = 0,
            start_pos = nil,
            completed = {},
            run_complete = false
        }
    end

    local state = ctx.state.perf_timer

    -- Reset if stopped
    if speed < STOP_THRESHOLD then
        if state.run_complete or (state.timing and now - state.start_time > 2.0) then
            -- Ready for new run after a completed run or timeout
            state.timing = false
            state.run_complete = false
            state.completed = {}
        end

        -- Start timing on throttle input from standstill
        if throttle > 0.05 and not state.timing and not state.run_complete then
            state.timing = true
            state.start_time = now
            state.start_pos = {
                x = telemetry.position and telemetry.position.x or 0,
                y = telemetry.position and telemetry.position.y or 0,
                z = telemetry.position and telemetry.position.z or 0
            }
            state.completed = {}
            print(string.format("[PERF] Entity %d: Timer started at T=%.2fs", ctx.id, now))
        end
        return
    end

    -- Only check benchmarks while timing
    if not state.timing then return end

    local elapsed = now - state.start_time

    -- Calculate distance traveled
    local distance = 0
    if state.start_pos and telemetry.position then
        local dx = telemetry.position.x - state.start_pos.x
        local dy = telemetry.position.y - state.start_pos.y
        local dz = telemetry.position.z - state.start_pos.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    end

    -- Check speed benchmarks
    for _, bench in ipairs(BENCHMARKS) do
        if not state.completed[bench.name] and speed >= bench.speed_ms then
            state.completed[bench.name] = true
            print(string.format("[PERF] Entity %d: %s in %.2f seconds",
                  ctx.id, bench.name, elapsed))
        end
    end

    -- Check distance benchmarks
    for _, mark in ipairs(DISTANCE_MARKS) do
        if not state.completed[mark.name] and distance >= mark.distance_m then
            state.completed[mark.name] = true
            local speed_mph = speed * 2.23694
            print(string.format("[PERF] Entity %d: %s in %.2f seconds @ %.1f mph",
                  ctx.id, mark.name, elapsed, speed_mph))
        end
    end

    -- Check if all benchmarks completed
    local all_done = true
    for _, bench in ipairs(BENCHMARKS) do
        if not state.completed[bench.name] then all_done = false end
    end
    for _, mark in ipairs(DISTANCE_MARKS) do
        if not state.completed[mark.name] then all_done = false end
    end

    if all_done then
        state.run_complete = true
        state.timing = false
        print(string.format("[PERF] Entity %d: Run complete!", ctx.id))
    end
end

-- Manual reset
function timer.reset(ctx)
    if ctx.state then
        ctx.state.perf_timer = nil
    end
end

return timer