# Proposal: Maneuver Scripting System ("Reflex Scripts")

**Date:** 2025-12-31
**Status:** IMPLEMENTED (January 2025)
**Builds On:** [Physics-Based Turn System](physics-based-turn-system.md)

> **Note:** This proposal has been implemented. Lua Reflex Scripts are now integrated
> via Sol3, with per-vehicle instances, hot-reload support, and event-driven
> communication. This document is preserved as a historical reference.

## Executive Summary

This proposal extends the physics-based maneuver system with a **scripting engine** that allows maneuvers to be defined as **goals with adaptive control logic** rather than canned input sequences. Scripts execute at fixed intervals during physics simulation, reading vehicle state and outputting control corrections - essentially allowing players to write their own traction control and stability systems.

**Suggested Name: "Reflex Scripts"** - mirroring how a skilled driver's reflexes make micro-corrections during complex maneuvers.

---

## Problem Statement

### The Open-Loop Problem

The previous proposal defined maneuvers as pre-recorded input sequences:

```
Frame 0:  Steering +100%, Throttle 0%
Frame 5:  E-brake ON
Frame 10: Steering +80%, Throttle 30%
...
```

**The Problem:** This is **open-loop control** - there's no feedback.

```
EXPECTED:                          ACTUAL:

Frame 0:  Car at position A        Frame 0:  Car at position A
Frame 5:  Car at position B        Frame 5:  Car at position B' (slight variance)
Frame 10: Car at position C        Frame 10: Car at position C'' (error compounds)
Frame 15: Car at position D        Frame 15: Car at position D''' (way off course)
Frame 20: Perfect 180° turn        Frame 20: Spin out (inputs no longer valid)
```

Like the telephone game - small errors accumulate. By mid-maneuver, the remaining inputs are useless because they assume a position the car never reached.

### Real-World Solution: Closed-Loop Control

Real vehicles solve this with feedback systems:
- **ABS** - Monitors wheel lock, pulses brakes
- **Traction Control** - Monitors slip, adjusts throttle
- **Stability Control** - Monitors yaw, applies selective braking
- **Human Reflexes** - Eyes see drift, hands correct steering

All of these are **closed-loop**: sense → compare to goal → adjust.

---

## Proposed Solution: Reflex Scripts

### Core Concept

Maneuvers are defined as:
1. **Goal State** - What we're trying to achieve (heading change, final speed, etc.)
2. **Reflex Script** - Code that runs periodically to make corrections

```
MANEUVER: Bootlegger Reverse
├── GOAL: Rotate 180°, final speed < 5 mph, facing opposite direction
└── SCRIPT: Runs every 66ms (15 Hz), reads state, outputs controls
```

### Execution Model

```
Physics Engine (60 Hz)
    │
    ├── Frame 0:  Physics step
    ├── Frame 1:  Physics step
    ├── Frame 2:  Physics step
    ├── Frame 3:  Physics step ← Script executes (15 Hz = every 4 frames)
    │                            ├── Read: position, velocity, heading, slip angle
    │                            ├── Compare: current vs goal
    │                            └── Output: steering, throttle, brake, e-brake
    ├── Frame 4:  Physics step (uses script output)
    ├── Frame 5:  Physics step
    ├── Frame 6:  Physics step
    ├── Frame 7:  Physics step ← Script executes again
    │             ...
```

### Script Interface

```lua
-- Reflex Script: Bootlegger Reverse
-- Runs at 15 Hz during maneuver execution

function init(goal)
    -- Called once when maneuver starts
    target_heading = normalize_angle(goal.start_heading + 180)
    phase = "initiate"
end

function update(state, dt)
    -- Called every 66ms
    -- state: current vehicle state
    -- dt: time since last update
    -- Returns: control outputs

    local heading_error = angle_diff(state.heading, target_heading)
    local speed = state.speed
    local yaw_rate = state.yaw_rate
    local slip_angle = state.slip_angle

    if phase == "initiate" then
        -- Phase 1: Initiate spin
        if math.abs(heading_error) > 150 then
            return {
                steering = 1.0,      -- Full lock
                throttle = 0.0,
                brake = 0.0,
                e_brake = true       -- Kick the rear out
            }
        else
            phase = "rotate"
        end

    elseif phase == "rotate" then
        -- Phase 2: Control rotation
        if math.abs(heading_error) > 20 then
            -- Still rotating, modulate to control speed
            local steer = clamp(heading_error * 0.05, -1, 1)
            return {
                steering = steer,
                throttle = 0.2,      -- Light throttle to maintain momentum
                brake = 0.0,
                e_brake = math.abs(yaw_rate) < 50  -- Release if spinning fast enough
            }
        else
            phase = "stabilize"
        end

    elseif phase == "stabilize" then
        -- Phase 3: Stop the rotation, straighten out
        local counter_steer = clamp(-yaw_rate * 0.02, -1, 1)
        local brake_amount = clamp(speed * 0.1, 0, 1)

        return {
            steering = counter_steer,
            throttle = 0.0,
            brake = brake_amount,
            e_brake = false
        }
    end
end

function is_complete(state)
    -- Called to check if maneuver succeeded
    local heading_error = angle_diff(state.heading, target_heading)
    return math.abs(heading_error) < 15 and state.speed < 5
end

function is_failed(state)
    -- Called to check if maneuver failed
    return state.is_rolled or state.is_stuck or state.time_elapsed > 3.0
end
```

---

## Scripting Engine: Lua via Sol3

### Why Lua

Lua is the industry standard for game scripting, used in World of Warcraft, Roblox, Garry's Mod, and countless other titles. For this use case:

- **Extremely fast** - A simple control script executes in 10-50 microseconds
- **LuaJIT option** - 2-10x faster if needed (drop-in replacement)
- **Easy to embed** - Sol3 provides clean C++ bindings
- **Battle-tested** - Decades of game industry usage
- **Learnable** - Simple syntax, huge community, lots of tutorials

### C++ Integration via Sol3

```cpp
#include <sol/sol.hpp>

class ReflexScriptEngine {
    sol::state lua;

public:
    void init() {
        lua.open_libraries(sol::lib::base, sol::lib::math, sol::lib::table);

        // Expose helper functions
        lua["clamp"] = [](float v, float lo, float hi) {
            return std::clamp(v, lo, hi);
        };
        lua["normalize_angle"] = &normalize_angle;
        lua["angle_diff"] = &angle_diff;

        // Expose PID controller
        lua.new_usertype<PIDController>("PIDController",
            sol::constructors<PIDController(float, float, float)>(),
            "update", &PIDController::update,
            "reset", &PIDController::reset
        );
    }

    ControlOutput execute(const std::string& script, const VehicleState& state) {
        lua["state"] = state;
        lua.script(script);
        return lua["update"](state, 0.066f);  // 66ms dt at 15 Hz
    }
};
```

### Multi-Vehicle Performance Analysis

**Scenario: 6 vehicles in arena, all executing Reflex Scripts**

```
Script execution rate:     15 Hz per vehicle
Total script calls:        6 × 15 = 90 calls/second
Typical script duration:   20-50 microseconds
Worst-case (complex PID):  200 microseconds

Per-second script overhead:
  Typical:    90 × 50 µs  = 4.5 ms   (0.45% of 1 second)
  Worst-case: 90 × 200 µs = 18 ms   (1.8% of 1 second)

Physics budget (60 Hz):    16.67 ms per frame
Script overhead per frame: 0.075 - 0.3 ms (negligible)
```

**Verdict: Lua handles 6 vehicles easily.** Even 20+ vehicles would be fine.

### Benchmark Reference

Lua performance for simple math operations:
```
Operation                  | Time (µs)
---------------------------|----------
Function call              | 0.1
10 float multiplications   | 0.2
Table lookup (5 keys)      | 0.3
angle_diff + clamp         | 0.5
Full bootlegger script     | 20-50
Complex PID controller     | 100-200
```

These numbers are for standard Lua. LuaJIT would be 2-5x faster.

### Separate Lua States per Vehicle

Each vehicle gets its own Lua state for isolation:

```cpp
struct VehicleScriptContext {
    sol::state lua;
    std::string current_script;
    bool initialized = false;

    // Script-local variables persist between updates
    // (phase tracking, PID integrators, etc.)
};

class ScriptManager {
    std::unordered_map<VehicleID, VehicleScriptContext> contexts;

public:
    void execute_all(float dt) {
        for (auto& [id, ctx] : contexts) {
            if (ctx.initialized) {
                auto state = get_vehicle_state(id);
                auto controls = ctx.lua["update"](state, dt);
                apply_controls(id, controls);
            }
        }
    }
};
```

Memory per Lua state: ~40 KB base + script size. For 6 vehicles: ~250 KB total.

---

## Script API

### Input: Vehicle State

```lua
state = {
    -- Position and orientation
    position = {x, y, z},        -- World position
    heading = 0.0,               -- Degrees, 0 = North
    pitch = 0.0,                 -- Degrees
    roll = 0.0,                  -- Degrees

    -- Velocities
    speed = 0.0,                 -- mph or m/s
    velocity = {x, y, z},        -- World velocity vector
    yaw_rate = 0.0,              -- Degrees/second

    -- Tire state
    slip_angle = {fl, fr, rl, rr},   -- Per-tire slip angles
    slip_ratio = {fl, fr, rl, rr},   -- Per-tire slip ratios
    tire_load = {fl, fr, rl, rr},    -- Per-tire vertical load

    -- Vehicle properties (read-only)
    mass = 0.0,
    wheelbase = 0.0,
    track_width = 0.0,

    -- Maneuver context
    time_elapsed = 0.0,          -- Seconds since maneuver start
    start_heading = 0.0,         -- Heading when maneuver began
    start_speed = 0.0,           -- Speed when maneuver began
    start_position = {x, y, z},

    -- Status flags
    is_grounded = true,
    is_rolled = false,
    is_stuck = false,
}
```

### Output: Control Commands

```lua
controls = {
    steering = 0.0,      -- -1.0 (full left) to +1.0 (full right)
    throttle = 0.0,      -- 0.0 to 1.0
    brake = 0.0,         -- 0.0 to 1.0
    e_brake = false,     -- true/false
}
```

### Helper Functions (Provided by Engine)

```lua
-- Math helpers
clamp(value, min, max)
lerp(a, b, t)
normalize_angle(degrees)      -- Wraps to -180..180
angle_diff(a, b)              -- Shortest angular distance

-- PID controller helper
pid_controller(kp, ki, kd)    -- Returns a PID object
pid:update(error, dt)         -- Returns control output

-- Logging (for debugging)
log_debug(message)
log_warning(message)
```

---

## Security and Sandboxing

### Restricted Environment

Scripts run in a sandbox with:
- **No file I/O**
- **No network access**
- **No os.execute or similar**
- **Limited memory allocation**
- **Execution time limits** (kill script if >5ms per update)

### Whitelist Approach

Only expose specific functions:
```lua
-- ALLOWED
math.sin, math.cos, math.abs, math.min, math.max, math.clamp
table.insert, table.remove
string.format

-- BLOCKED
io.*, os.*, debug.*, package.*, loadfile, dofile, require
```

### Resource Limits

```cpp
struct ScriptLimits {
    size_t max_memory = 1024 * 1024;    // 1 MB
    double max_execution_time = 0.005;   // 5ms per update
    int max_instructions = 100000;       // Instruction count limit
};
```

---

## Player Experience

### Script Editor

```
┌─────────────────────────────────────────────────────────────────┐
│  REFLEX SCRIPT EDITOR                                           │
├─────────────────────────────────────────────────────────────────┤
│  Maneuver: Bootlegger Reverse          Car: Luxury XL           │
├─────────────────────────────────────────────────────────────────┤
│  1│ -- Bootlegger Reflex Script v2                              │
│  2│ local target_heading                                        │
│  3│ local phase = "initiate"                                    │
│  4│                                                             │
│  5│ function init(goal)                                         │
│  6│     target_heading = normalize_angle(                       │
│  7│         goal.start_heading + 180)                           │
│  8│ end                                                         │
│  9│                                                             │
│ 10│ function update(state, dt)                                  │
│ 11│     local err = angle_diff(state.heading, ▊                 │
│    │ ...                                                        │
├─────────────────────────────────────────────────────────────────┤
│  [SAVE]  [TEST RUN]  [VALIDATE]  [LOAD TEMPLATE]  [SHARE]       │
├─────────────────────────────────────────────────────────────────┤
│  CONSOLE OUTPUT:                                                │
│  > Script validated: OK                                         │
│  > Test run started...                                          │
│  > [0.00s] Phase: initiate, heading_err: 180.0°                 │
│  > [0.25s] Phase: rotate, heading_err: 95.2°                    │
│  > [0.50s] Phase: rotate, heading_err: 42.1°                    │
│  > [0.75s] Phase: stabilize, heading_err: 12.4°                 │
│  > [1.02s] COMPLETE: Final heading error: 8.2°, speed: 3.1 mph  │
│  > Test run: SUCCESS                                            │
└─────────────────────────────────────────────────────────────────┘
```

### Difficulty Levels

**Beginner:** Use pre-made script templates
```
[TEMPLATES]
├── Basic Stability Control
├── Simple Bootlegger
├── Drift Controller
└── Emergency Stop
```

**Intermediate:** Modify templates, adjust PID gains
```lua
-- Just tweak these values!
local STEERING_GAIN = 0.05    -- How aggressive to steer
local COUNTER_STEER_GAIN = 0.02
local BRAKE_GAIN = 0.1
```

**Advanced:** Write custom scripts from scratch

### Community Sharing

- Share scripts via export/import
- Community repository of proven scripts
- "Star" rating system
- Scripts tagged by car type/weight class

---

## Competitive Implications

### Skill Expression Layers

1. **Vehicle Building** - Classic stat optimization
2. **Script Writing** - Control algorithm design
3. **Parameter Tuning** - Dial in gains for your car
4. **Tactical Decisions** - When to use which maneuver
5. **Execution Timing** - Choosing the right moment

### Fair Play Considerations

**Option A: Open Scripts**
- All scripts visible to opponents
- Competition is about building better algorithms
- Like open-source racing

**Option B: Secret Scripts**
- Scripts are hidden
- Adds mystery/bluffing element
- Rewards R&D time

**Option C: Classified Scripts**
- Scripts revealed after match
- Best of both worlds
- Allows learning from opponents

### Anti-Cheat

Scripts can only output valid control values (-1 to 1, 0 to 1, etc.). They cannot:
- Modify vehicle physics directly
- Access opponent state
- Influence anything outside their own vehicle
- Execute between their update windows

---

## Network Architecture

### The Turn-Based Advantage

Unlike real-time games that must constantly synchronize at 60+ Hz, turn-based games have **natural synchronization points**. Each turn boundary is a checkpoint where all clients can verify they're in agreement.

```
TURN N
├── DECLARATION PHASE (network: gather inputs)
│   ├── Each client submits: {maneuver, speed_change, combat_action}
│   ├── Server collects all declarations
│   └── Server broadcasts all declarations to all clients
│
├── EXECUTION PHASE (local: simulate physics)
│   ├── All clients run identical 1-second physics simulation
│   ├── All clients run their LOCAL scripts (no network needed)
│   └── Physics outputs control values → physics engine
│
└── SYNC CHECKPOINT (network: verify agreement)
    ├── Each client reports final vehicle states
    ├── Server validates states match (within tolerance)
    └── If drift detected: server sends authoritative correction
```

### Why Scripts Run Locally

Each player's Reflex Script runs on their own machine:

```
Player A's Machine              Player B's Machine
┌─────────────────────┐         ┌─────────────────────┐
│ Vehicle A           │         │ Vehicle A           │
│ ├── Script A (runs) │         │ ├── Script A (runs) │
│ └── Controls → Phys │         │ └── Controls → Phys │
│                     │         │                     │
│ Vehicle B           │         │ Vehicle B           │
│ ├── Script B (runs) │         │ ├── Script B (runs) │
│ └── Controls → Phys │         │ └── Controls → Phys │
└─────────────────────┘         └─────────────────────┘
        │                               │
        └──────── Should match ─────────┘
```

**Key insight:** Scripts are deterministic. Given identical vehicle state input, a script will produce identical control output. So if both machines start the turn with the same state, they'll execute identically.

### What Gets Sent Over Network

**Per Turn (tiny bandwidth):**
```
Declaration packet (per vehicle):
├── maneuver_id:    uint16   (2 bytes)
├── speed_change:   int8     (1 byte: -15 to +15)
├── combat_target:  uint16   (2 bytes)
├── weapon_mask:    uint8    (1 byte)
└── Total: ~6 bytes per vehicle

Sync packet (per vehicle, end of turn):
├── position:       vec3     (12 bytes)
├── heading:        float    (4 bytes)
├── speed:          float    (4 bytes)
├── damage_state:   uint32   (4 bytes)
└── Total: ~24 bytes per vehicle
```

**6 vehicles, 1 turn:** ~180 bytes total. Negligible.

### Handling Physics Drift

Floating-point physics isn't perfectly deterministic across CPUs. Small differences accumulate.

**Solution: Authoritative Server with Tolerance**

```cpp
struct SyncTolerance {
    float position = 0.1f;      // 10cm allowed drift
    float heading = 1.0f;       // 1 degree allowed drift
    float speed = 0.5f;         // 0.5 mph allowed drift
};

bool states_match(const VehicleState& a, const VehicleState& b) {
    return glm::distance(a.position, b.position) < tolerance.position
        && std::abs(angle_diff(a.heading, b.heading)) < tolerance.heading
        && std::abs(a.speed - b.speed) < tolerance.speed;
}
```

**If drift exceeds tolerance:**
1. Server's state is authoritative
2. Clients receive correction
3. Clients snap to corrected position (or interpolate smoothly)

In practice, small drift rarely matters in a turn-based game - vehicles move significant distances each turn, and minor position errors are invisible.

### Script Sharing for Multiplayer

Scripts must be shared so all clients can execute them:

**Option A: Pre-Match Upload**
```
Match Setup:
1. Each player uploads their Reflex Scripts to server
2. Server distributes all scripts to all clients
3. Scripts cached locally for the match
```

**Option B: Compile to Bytecode**
```
Local Machine:
1. Player writes script
2. Compile to Lua bytecode (luac)
3. Hash bytecode for integrity

Network:
4. Share bytecode + hash
5. All clients execute identical bytecode
```

Bytecode is smaller, faster to load, and harder to cheat with.

### Latency Hiding

Since turns are 1 second, latency is largely hidden:

```
Timeline (200ms network latency):

0ms:     Player A declares maneuver
200ms:   Server receives A's declaration
200ms:   Server waits for all players (or timeout)
400ms:   Server broadcasts all declarations
600ms:   All clients begin execution
600ms-1600ms: Physics runs locally (no network needed)
1600ms:  Turn ends, sync check
```

Even with 200ms latency, execution feels instant because physics runs locally. The only delay is waiting for all declarations (configurable timeout: 5-10 seconds for turn-based).

### Anti-Cheat Considerations

**Script Tampering:**
- Scripts hashed on upload
- Server verifies script hasn't changed mid-match
- Periodic random sampling of script outputs vs expected

**Physics Cheating:**
- All clients run all scripts for all vehicles
- Compare outputs - if one client disagrees, they're cheating
- Authoritative server has final say

**Impossible Inputs:**
- Scripts can only output valid ranges (-1 to 1, 0 to 1)
- Server validates control outputs before physics
- Reject invalid packets, flag suspicious patterns

---

## Implementation Phases

### Phase 1: Core Integration
- [ ] Integrate Lua (via Sol3)
- [ ] Create sandbox environment
- [ ] Implement state reading API
- [ ] Implement control output API
- [ ] Basic script loading/execution

### Phase 2: Script Lifecycle
- [ ] init/update/is_complete/is_failed callbacks
- [ ] Execution timing (15 Hz)
- [ ] Error handling and timeout
- [ ] Debug logging system

### Phase 3: Editor UI
- [ ] In-game script editor
- [ ] Syntax highlighting
- [ ] Validation feedback
- [ ] Test run mode

### Phase 4: Templates and Sharing
- [ ] Built-in template scripts
- [ ] Export/import format
- [ ] Community sharing backend (future)

### Phase 5: Advanced Features
- [ ] Visual scripting option (node-based)
- [ ] Script performance profiler
- [ ] A/B testing mode (compare two scripts)

---

## Example Scripts

### Simple Drift Stabilizer

```lua
-- Maintains a drift at target slip angle
local TARGET_SLIP = 15  -- degrees

function update(state, dt)
    local rear_slip = (state.slip_angle.rl + state.slip_angle.rr) / 2
    local slip_error = TARGET_SLIP - rear_slip

    return {
        steering = clamp(slip_error * 0.1, -1, 1),
        throttle = 0.6,
        brake = 0.0,
        e_brake = false
    }
end
```

### PID-Based Heading Controller

```lua
local heading_pid = pid_controller(0.05, 0.001, 0.02)
local target_heading

function init(goal)
    target_heading = goal.target_heading
end

function update(state, dt)
    local error = angle_diff(state.heading, target_heading)
    local steer = heading_pid:update(error, dt)

    return {
        steering = clamp(steer, -1, 1),
        throttle = 0.3,
        brake = 0.0,
        e_brake = false
    }
end
```

### Traction Control System

```lua
-- Prevents wheel spin on acceleration
local MAX_SLIP_RATIO = 0.15

function update(state, dt)
    local max_slip = math.max(
        state.slip_ratio.rl,
        state.slip_ratio.rr
    )

    local throttle = 1.0
    if max_slip > MAX_SLIP_RATIO then
        -- Reduce throttle proportionally to excess slip
        throttle = MAX_SLIP_RATIO / max_slip
    end

    return {
        steering = 0.0,
        throttle = throttle,
        brake = 0.0,
        e_brake = false
    }
end
```

---

## Open Questions

1. **Visual Scripting?** Should we add a node-based alternative for non-programmers?
2. **Script Marketplace?** Allow selling/buying scripts?
3. **AI Scripts?** Can NPCs use the same system for their driving AI?
4. **Telemetry Recording?** Record script decisions for replay analysis?
5. **Multi-Script?** Can a vehicle run multiple scripts (e.g., traction + stability)?

---

## Conclusion

This isn't crazy - it's how real vehicle dynamics control works. By exposing a scripting interface, we transform maneuvers from "canned animations that probably fail" into "adaptive control systems that players can master."

The skill ceiling becomes:
- **Low:** Use template scripts, tweak basic parameters
- **Medium:** Modify scripts, understand control theory basics
- **High:** Write custom scripts, optimize for specific scenarios

This creates emergent depth while keeping the game accessible.

---

## References

- [Physics-Based Turn System Proposal](physics-based-turn-system.md)
- Lua 5.4 Reference Manual
- Sol3 Documentation
- PID Controller Theory
- Vehicle Dynamics Control Systems (ESP, ABS, TCS)
