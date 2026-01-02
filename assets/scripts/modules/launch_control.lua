--[[
    Launch Control Module

    Limits throttle at low speeds to prevent excessive wheelspin.
    Unlike TCS (which brakes spinning wheels), this reduces engine power
    at the source - similar to real launch control systems.

    Usage:
        local lc = require("modules/launch_control")
        lc.update(ctx)
]]

local lc = {}

-- Configuration
lc.max_speed = 15.0          -- m/s (~34 mph) - disable above this speed
lc.slip_threshold = 0.5      -- 50% slip triggers throttle cut
lc.min_throttle = 0.3        -- Minimum throttle (30%) - never cut below this
lc.enabled = false           -- Disabled for TEST C

-- Debug
lc.logged = false

function lc.update(ctx)
    if not lc.enabled then return end

    local controls = ctx.controls
    local telemetry = ctx.telemetry
    local speed = telemetry.speed_ms or 0

    -- Only active at low speeds
    if speed > lc.max_speed then
        return
    end

    -- Check if we're accelerating
    local throttle = controls.throttle or 0
    if throttle < 0.1 then
        return
    end

    -- Find max wheel slip
    local max_slip = 0
    for i = 1, 4 do
        local wheel = telemetry.wheels[i]
        if wheel and wheel.slip then
            max_slip = math.max(max_slip, wheel.slip)
        end
    end

    -- If slip is excessive, reduce throttle
    if max_slip > lc.slip_threshold then
        -- Proportional reduction: more slip = less throttle
        -- At 100% slip -> min_throttle, at threshold -> full throttle
        local slip_excess = (max_slip - lc.slip_threshold) / (1.0 - lc.slip_threshold)
        slip_excess = math.min(slip_excess, 1.0)

        local throttle_mult = 1.0 - (slip_excess * (1.0 - lc.min_throttle))
        local new_throttle = throttle * throttle_mult

        controls.throttle = math.max(new_throttle, lc.min_throttle)

        -- One-time log
        if not lc.logged then
            lc.logged = true
            print(string.format("[LC] Launch control active (slip=%.0f%%, throttle=%.0f%%)",
                  max_slip * 100, controls.throttle * 100))
        end
    end
end

return lc
