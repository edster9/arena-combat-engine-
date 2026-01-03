--[[
    Freestyle Driving Assist Script

    Provides driver assists for freestyle (real-time) mode:
    - ABS: Anti-lock braking system (reduces brake on wheel lockup)
    - TCS: Traction control (reduces throttle on wheelspin)
    - ESC: Electronic stability control (counter-steer) [TODO]

    This script intercepts player controls and modifies them
    to prevent loss of control with binary keyboard input.

    Control flow:
    Player keyboard -> ctx.controls -> this script -> physics

    Context (ctx) structure:
    - ctx.id          : Entity ID (vehicle_id)
    - ctx.dt          : Delta time
    - ctx.state       : Per-entity persistent state (managed by C++)
    - ctx.config      : Script configuration from vehicle JSON
    - ctx.telemetry   : Read-only vehicle state (position, speed, wheels)
    - ctx.controls    : Player inputs (steering, throttle, brake, handbrake)
]]

-- Load reusable modules (singletons - use ctx.state for per-entity data)
local abs = require("modules/abs")
local tcs = require("modules/tcs")
local esc = require("modules/esc")

-- ============================================================================
-- CONFIGURATION
-- ============================================================================

-- Helper to read config with defaults
-- ctx.config is set by C++ from vehicle JSON options
local function get_config(ctx, key, default)
    if ctx.config and ctx.config[key] ~= nil then
        -- Booleans are stored as 0.0 or 1.0 in JSON
        if type(default) == "boolean" then
            return ctx.config[key] ~= 0
        end
        return ctx.config[key]
    end
    return default
end

-- Apply configuration on first update for this entity
local function apply_config(ctx)
    -- Use ctx.state for per-entity config tracking
    if ctx.state.config_applied then return end
    if not ctx.config then
        print("[freestyle_assist] Entity " .. ctx.id .. " - NO CONFIG, skipping")
        return
    end

    -- Store per-entity config in ctx.state
    ctx.state.abs_enabled = get_config(ctx, "abs_enabled", true)
    ctx.state.abs_min_speed = get_config(ctx, "abs_min_speed", 2.2)
    ctx.state.tcs_enabled = get_config(ctx, "tcs_enabled", false)
    ctx.state.tcs_min_speed = get_config(ctx, "tcs_min_speed", 1.0)
    ctx.state.esc_enabled = get_config(ctx, "esc_enabled", false)

    ctx.state.config_applied = true
    print("[freestyle_assist] Entity " .. ctx.id .. " config applied: ABS=" ..
          tostring(ctx.state.abs_enabled) .. ", TCS=" .. tostring(ctx.state.tcs_enabled))
end

-- ============================================================================
-- MAIN UPDATE
-- ============================================================================
function update(ctx)
    -- Apply config on first update (after C++ has set ctx.config)
    apply_config(ctx)

    -- Run all enabled assists (pass ctx - modules use ctx.state for per-entity data)
    abs.update(ctx)
    tcs.update(ctx)
    esc.update(ctx)
end