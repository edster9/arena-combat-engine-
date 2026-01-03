--[[
    Test Sequence Module

    Orchestrates automated maneuver testing with configurable sequences.
    Trigger with a key binding to run through multiple maneuvers with delays.

    Usage:
        local test_seq = require("modules/test_sequence")

        -- Start a test sequence
        test_seq.start("basic_bends")

        -- Update each frame (handles timing and sequencing)
        test_seq.update(dt)
]]

local test_sequence = {}

-- Import maneuver module for triggering
local maneuver = require("modules/maneuver")

-- ============================================================================
-- TEST SEQUENCES
-- Define sequences of maneuvers to test
-- ============================================================================

test_sequence.sequences = {
    -- Basic bend tests at current speed
    basic_bends = {
        { type = "bend_45", direction = "right" },
        { type = "bend_45", direction = "left" },
        { type = "bend_30", direction = "right" },
        { type = "bend_30", direction = "left" },
        { type = "bend_15", direction = "right" },
        { type = "bend_15", direction = "left" },
    },

    -- Sharp turns
    sharp_turns = {
        { type = "bend_60", direction = "right" },
        { type = "bend_60", direction = "left" },
        { type = "bend_75", direction = "right" },
        { type = "bend_75", direction = "left" },
        { type = "bend_90", direction = "right" },
        { type = "bend_90", direction = "left" },
    },

    -- All bends in order
    all_bends = {
        { type = "bend_15", direction = "right" },
        { type = "bend_30", direction = "right" },
        { type = "bend_45", direction = "right" },
        { type = "bend_60", direction = "right" },
        { type = "bend_75", direction = "right" },
        { type = "bend_90", direction = "right" },
        { type = "bend_15", direction = "left" },
        { type = "bend_30", direction = "left" },
        { type = "bend_45", direction = "left" },
        { type = "bend_60", direction = "left" },
        { type = "bend_75", direction = "left" },
        { type = "bend_90", direction = "left" },
    },

    -- Drift and swerve tests
    lateral_moves = {
        { type = "drift", direction = "right" },
        { type = "drift", direction = "left" },
        { type = "steep_drift", direction = "right" },
        { type = "steep_drift", direction = "left" },
        { type = "swerve", direction = "right" },
        { type = "swerve", direction = "left" },
    },

    -- Advanced maneuvers
    advanced = {
        { type = "controlled_skid", direction = "right" },
        { type = "controlled_skid", direction = "left" },
        { type = "t_stop", direction = "right" },
        { type = "bootlegger", direction = "right" },
    },

    -- Quick smoke test
    smoke_test = {
        { type = "bend_45", direction = "right" },
        { type = "bend_45", direction = "left" },
        { type = "bend_90", direction = "right" },
    },
}

-- ============================================================================
-- STATE
-- ============================================================================

test_sequence.state = {
    active = false,
    sequence_name = nil,
    sequence = nil,
    current_index = 0,
    phase = "idle",  -- idle, executing, waiting
    wait_timer = 0,
    execute_timer = 0,        -- Time elapsed in current maneuver
    maneuver_duration = 1.0,  -- How long each maneuver runs
    gap_duration = 2.0,       -- Gap between maneuvers
    results = {},             -- Collected results
    target_entity = 0,        -- Which entity to test
}

-- ============================================================================
-- API
-- ============================================================================

-- Start a test sequence
function test_sequence.start(sequence_name, options)
    local seq = test_sequence.sequences[sequence_name]
    if not seq then
        print(string.format("[TestSeq] ERROR: Unknown sequence '%s'", sequence_name))
        print("[TestSeq] Available sequences:")
        for name, _ in pairs(test_sequence.sequences) do
            print("  - " .. name)
        end
        return false
    end

    local state = test_sequence.state
    state.active = true
    state.sequence_name = sequence_name
    state.sequence = seq
    state.current_index = 0
    state.phase = "waiting"  -- Start with brief wait
    state.wait_timer = 0.5   -- Short initial delay
    state.results = {}
    state.target_entity = options and options.entity or 0
    state.gap_duration = options and options.gap or 2.0
    state.maneuver_duration = options and options.duration or 1.0

    print(string.format("[TestSeq] ========================================"))
    print(string.format("[TestSeq] Starting sequence: %s (%d maneuvers)",
        sequence_name, #seq))
    print(string.format("[TestSeq] Gap: %.1fs, Duration: %.1fs",
        state.gap_duration, state.maneuver_duration))
    print(string.format("[TestSeq] ========================================"))

    return true
end

-- Stop the current sequence
function test_sequence.stop()
    local state = test_sequence.state
    if state.active then
        print(string.format("[TestSeq] Sequence stopped at maneuver %d/%d",
            state.current_index, #state.sequence))
        test_sequence.print_results()
    end
    state.active = false
    state.phase = "idle"
end

-- Update - call each frame
function test_sequence.update(ctx)
    local state = test_sequence.state
    if not state.active then return end

    local dt = ctx.dt or 0.016

    if state.phase == "waiting" then
        -- Waiting between maneuvers
        state.wait_timer = state.wait_timer - dt
        if state.wait_timer <= 0 then
            -- Start next maneuver
            state.current_index = state.current_index + 1
            if state.current_index > #state.sequence then
                -- Sequence complete
                test_sequence.complete()
            else
                test_sequence.start_maneuver(ctx)
            end
        end

    elseif state.phase == "executing" then
        -- Track elapsed time
        state.execute_timer = state.execute_timer + dt

        -- Check if maneuver duration complete
        if state.execute_timer >= state.maneuver_duration then
            -- Evaluate the maneuver result
            local result = maneuver.evaluate(ctx)

            -- Clear turn_active so controls return to player
            if ctx.state then
                ctx.state.turn_active = false
            end

            -- Collect and log result
            test_sequence.collect_result(ctx, result)

            -- Transition to waiting
            state.phase = "waiting"
            state.wait_timer = state.gap_duration
        end
    end
end

-- Start the current maneuver
function test_sequence.start_maneuver(ctx)
    local state = test_sequence.state
    local move = state.sequence[state.current_index]

    print(string.format("[TestSeq] [%d/%d] Starting: %s %s",
        state.current_index, #state.sequence,
        move.direction, move.type))

    -- Reset execute timer
    state.execute_timer = 0

    -- Start the maneuver
    maneuver.start(ctx, move.type, move.direction, {
        duration = state.maneuver_duration
    })

    -- Set turn_active on vehicle state so master.update() runs maneuver.update()
    if ctx.state then
        ctx.state.turn_active = true
    end

    state.phase = "executing"
end

-- Collect result after maneuver completes
function test_sequence.collect_result(ctx, eval_result)
    local state = test_sequence.state
    local move = state.sequence[state.current_index]

    -- Use the evaluation result directly
    local result = {
        index = state.current_index,
        type = move.type,
        direction = move.direction,
        level = eval_result and eval_result.level or "unknown",
        success = eval_result and eval_result.success or false,
        heading_error = eval_result and eval_result.heading_error or 0,
    }

    table.insert(state.results, result)

    local status = result.success and "PASS" or "FAIL"
    print(string.format("[TestSeq] [%d/%d] %s: %s %s (error: %.1f°)",
        state.current_index, #state.sequence,
        status, move.direction, move.type, result.heading_error))
end

-- Sequence completed
function test_sequence.complete()
    local state = test_sequence.state
    state.active = false
    state.phase = "idle"

    print(string.format("[TestSeq] ========================================"))
    print(string.format("[TestSeq] Sequence '%s' COMPLETE", state.sequence_name))
    test_sequence.print_results()
    print(string.format("[TestSeq] ========================================"))
end

-- Print summary of results
function test_sequence.print_results()
    local state = test_sequence.state
    local passed = 0
    local failed = 0
    local by_level = { perfect = 0, good = 0, partial = 0, failed = 0 }

    for _, r in ipairs(state.results) do
        if r.success then
            passed = passed + 1
        else
            failed = failed + 1
        end
        by_level[r.level] = (by_level[r.level] or 0) + 1
    end

    print(string.format("[TestSeq] Results: %d/%d passed (%.0f%%)",
        passed, #state.results,
        #state.results > 0 and (passed / #state.results * 100) or 0))
    print(string.format("[TestSeq]   Perfect: %d, Good: %d, Partial: %d, Failed: %d",
        by_level.perfect or 0, by_level.good or 0,
        by_level.partial or 0, by_level.failed or 0))

    -- List failures
    if failed > 0 then
        print("[TestSeq] Failed maneuvers:")
        for _, r in ipairs(state.results) do
            if not r.success then
                print(string.format("  - %s %s", r.direction, r.type))
            end
        end
    end
end

-- Check if sequence is running
function test_sequence.is_active()
    return test_sequence.state.active
end

-- Get available sequence names
function test_sequence.list_sequences()
    local names = {}
    for name, _ in pairs(test_sequence.sequences) do
        table.insert(names, name)
    end
    table.sort(names)
    return names
end

return test_sequence