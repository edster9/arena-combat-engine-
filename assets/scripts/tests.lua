--[[
    Tests Script

    Provides automated test sequence control for maneuver testing.
    Handles Y key binding to start/stop test sequences.

    Input Bindings:
    - Y: Toggle test sequence (start basic_bends or stop if running)

    Context (ctx) structure:
    - ctx.id          : Entity ID (vehicle_id)
    - ctx.dt          : Delta time
    - ctx.state       : Per-entity persistent state
    - ctx.config      : Script configuration
    - ctx.telemetry   : Read-only vehicle state
    - ctx.controls    : Control outputs (steering, throttle, brake)
]]

-- Key codes (from platform.h)
local KEY_Y = 28

-- Load the test sequence module
local test_sequence = require("modules/test_sequence")

-- Load maneuver for test_sequence to use
local maneuver = require("modules/maneuver")

-- Load performance timer (0-60, quarter mile, etc.)
local timer_0_60 = require("modules/timer_0_60")

-- ============================================================================
-- MAIN UPDATE
-- ============================================================================

function update(ctx)
    -- Run test sequence if active for this entity
    if test_sequence.is_active() and ctx.id == test_sequence.state.target_entity then
        test_sequence.update(ctx)
    end

    -- Performance timer (0-60, quarter mile, etc.) - always active
    timer_0_60.update(ctx)
end

-- ============================================================================
-- INPUT HANDLING
-- ============================================================================

function on_input(input_ctx)
    local keys = input_ctx.keys
    local vehicle_id = input_ctx.vehicle_id

    -- Y: Toggle test sequence
    if keys[KEY_Y] then
        if test_sequence.is_active() then
            -- Stop current test sequence
            print("[Tests] Y pressed: Stopping test sequence")
            test_sequence.stop()
        else
            -- Start basic_bends sequence
            local sequence_name = "basic_bends"
            print(string.format("[Tests] Y pressed: Starting test sequence '%s' on vehicle %d",
                sequence_name, vehicle_id))

            test_sequence.start(sequence_name, {
                entity = vehicle_id,
            })
        end
    end
end

-- ============================================================================
-- INIT / DESTROY
-- ============================================================================

function init(ctx)
    print(string.format("[Tests] Script attached to vehicle %d", ctx.id))
end

function destroy(ctx)
    -- Stop any running test sequence when script is detached
    if test_sequence.is_active() then
        test_sequence.stop()
    end
    print(string.format("[Tests] Script detached from vehicle %d", ctx.id))
end