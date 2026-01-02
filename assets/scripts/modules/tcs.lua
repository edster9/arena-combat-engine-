--[[
    TCS Module (Traction Control System) - PLACEHOLDER

    This module is currently a placeholder for future experimentation.

    Testing showed that brake-based TCS provides negligible acceleration benefit
    when using a generous friction curve at the physics layer. The friction curve
    approach (configured in assets/data/physics.json) handles wheelspin naturally
    without requiring script intervention.

    The script infrastructure remains in place for future experimentation with:
    - Stability control during cornering
    - Throttle-based TCS for different feel
    - Per-surface grip adjustments
    - Driver assist features

    Usage:
        local tcs = require("modules/tcs")
        tcs.update(ctx)  -- Currently does nothing

    Context (ctx) structure:
        ctx.state       - Per-entity state (use ctx.state.tcs for TCS data)
        ctx.telemetry   - Vehicle telemetry
        ctx.controls    - Controls to modify (includes wheel_brake[1-4])
        ctx.dt          - Delta time

    Per-entity config (set by parent script in ctx.state):
        ctx.state.tcs_enabled    - Enable/disable TCS
]]

local tcs = {}

-- Placeholder update - does nothing
-- The friction curve in physics.json now handles wheelspin
function tcs.update(ctx)
    -- TCS disabled - friction curve handles grip
    -- To re-enable, restore the brake-based implementation from git history
end

-- Reset TCS state for an entity
function tcs.reset(ctx)
    if ctx.state then
        ctx.state.tcs = nil
    end
end

return tcs