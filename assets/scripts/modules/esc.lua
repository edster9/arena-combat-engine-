--[[
    ESC - Electronic Stability Control Module

    Detects oversteer/understeer and applies corrections:
    - Counter-steer assistance
    - Individual wheel braking
    - Throttle reduction during instability

    Uses ctx.state for per-entity persistent data.
    Enable via vehicle JSON: "esc_enabled": true

    TODO: Implement stability detection and correction
]]

local esc = {}

-- Initialize ESC state for this entity
local function init_state(ctx)
    if ctx.state.esc_initialized then return end

    ctx.state.esc_initialized = true
    ctx.state.esc_active = false  -- Currently intervening?

    -- Could track:
    -- ctx.state.esc_yaw_rate_history = {}
    -- ctx.state.esc_slip_angle = 0
end

-- Main update - called each frame
function esc.update(ctx)
    -- Skip if not enabled
    if not ctx.state.esc_enabled then return end

    init_state(ctx)

    -- TODO: Detect instability
    -- 1. Compare actual yaw rate vs expected (based on steering + speed)
    -- 2. Calculate slip angle from lateral_velocity
    -- 3. If slip angle or yaw error exceeds threshold, intervene

    -- TODO: Apply corrections
    -- - Oversteer: brake outer front wheel, reduce throttle
    -- - Understeer: brake inner rear wheel, reduce throttle

    -- For now: placeholder that does nothing
    ctx.state.esc_active = false
end

return esc