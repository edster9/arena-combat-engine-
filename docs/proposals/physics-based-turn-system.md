# Proposal: Physics-Based Turn System

**Date:** 2025-12-30
**Status:** IMPLEMENTED (January 2025)
**Author:** Design Session Notes

> **Note:** This proposal has been implemented. The core concepts (single-phase turns,
> physics execution, Lua scripting) are now part of the engine. This document is
> preserved as a historical reference for understanding design decisions.

## Executive Summary

This proposal outlines a potential pivot from the classic tabletop 5-phase system to a physics-based execution model. The goal is to preserve the tactical depth of turn-based vehicular combat while leveraging real physics for movement resolution and enabling emergent player skill through custom maneuver profiles.

---

## Problem Statement

### The 5-Phase Timing Problem

Classic tabletop rules specify:
- 1 turn = 1 second
- Each turn divided into 5 phases (0.2 seconds each)
- Players can execute up to 5 separate maneuvers per turn (one per phase)
- At 50+ mph, vehicles move in all 5 phases

**Physics Reality:**
- Executing 5 distinct maneuvers in 1 second at 60 mph is physically impossible
- A drift + bend + swerve + drift + bend sequence in 1 second is absurd
- Real physics simulation cannot meaningfully execute sub-200ms maneuvers

### Why 5 Phases Existed (Tabletop Context)

The phase system was designed for:
1. **Miniature interleaving** - Faster cars move in more phases, creating turn order
2. **Incremental movement** - 1" per phase on grid maps
3. **Mid-turn decisions** - React to other players' movements
4. **Combat timing** - Fire during specific phases

These are **board game conveniences**, not physics requirements.

---

## Proposed Solution: Single-Phase Physics Turns

### New Turn Structure

```
TURN (1 second real-time)
├── 1. DECLARATION PHASE (simultaneous, hidden)
│   ├── Declare speed change (+/- acceleration, or maintain)
│   ├── Select maneuver (from player's library)
│   └── Declare combat actions (target, weapon)
│
├── 2. REVEAL
│   └── All players reveal declarations simultaneously
│
├── 3. PHYSICS EXECUTION (1 second simulation)
│   ├── All vehicles execute declared maneuvers via physics
│   ├── Collisions resolved by physics engine
│   ├── Weapons fire during execution window
│   └── Maneuver success/failure determined by physics outcome
│
└── 4. END OF TURN
    ├── Assess damage
    ├── Update vehicle states
    └── Begin next turn
```

### Key Changes from Classic

| Aspect | Classic (5-Phase) | Proposed (Single-Phase) |
|--------|-------------------|-------------------------|
| Phases per turn | 5 | 1 |
| Maneuvers per turn | Up to 5 | 1 |
| Movement resolution | Kinematic (predetermined arcs) | Physics simulation |
| Maneuver outcome | Dice roll + crash table | Physics determines |
| Turn duration | 1 second (abstract) | 1 second (real physics) |
| Interleaving | Phase-based | Simultaneous execution |

---

## Handling Status Rework

### Classic System
- HS starts at Handling Class (HC)
- Each maneuver subtracts difficulty value
- HS resets to HC at end of each turn
- Negative HS triggers control roll

### Proposed: Accumulated Stress Model

HS persists across turns instead of resetting:

```
Turn 1: D2 maneuver → HS drops by 2
Turn 2: D3 maneuver → HS drops by 3 more (cumulative)
Turn 3: Straight driving → HS recovers by HC value
Turn 4: Straight driving → HS recovers (capped at starting HC)
```

**Recovery Rules:**
- Driving straight at moderate speed: Recover HC points
- Aggressive maneuvering: No recovery
- HS can never exceed starting HC

**Control Checks:**
- When HS goes negative: Roll on Control Table
- Failed roll: Physics takes over (loss of control)
- Recovery when vehicle stabilizes

### Alternative: Pure Physics (No HS)

Option to disable HS entirely and let physics be the arbiter:
- Sharp turn at high speed = physics-induced spin/roll
- No dice rolls needed
- Add surface friction variance for unpredictability

---

## Player-Tuned Maneuver Profiles

### The Core Innovation

Instead of fixed maneuver definitions, players create and tune their own input sequences for complex maneuvers. Each profile is specific to their car configuration.

### Maneuver Profile Structure

```cpp
struct ManeuverProfile {
    std::string name;              // "my_bootlegger_v3"
    std::string maneuver_type;     // "bootlegger", "t_stop", "drift", etc.

    // Input curves (60 samples = 1 second at 60fps)
    std::array<float, 60> steering;    // -1.0 to +1.0
    std::array<float, 60> throttle;    // 0.0 to 1.0
    std::array<float, 60> brake;       // 0.0 to 1.0
    std::array<bool, 60> handbrake;    // on/off

    // Valid entry conditions
    float min_entry_speed;
    float max_entry_speed;

    // Success criteria
    float target_heading_change;   // degrees
    float heading_tolerance;       // +/- degrees
    float max_final_speed;         // for stop maneuvers

    // Metadata
    std::string car_config_hash;   // Tied to specific build
    int test_attempts;
    int test_successes;
    float success_rate;
};
```

### Maneuver Categories

**Simple (System-Provided Defaults):**
- Straight - No steering input
- Drift - Slight constant steering angle
- Bend - Steering proportional to desired angle

**Complex (Player-Tunable):**
- Controlled Skid - Steering + brake + counter-steer sequence
- Bootlegger Reverse - Hard steer + e-brake + throttle timing
- T-Stop - 90° lock + full brake + hold
- Pivot - Low speed full lock + throttle modulation

### Practice Mode / Maneuver Workshop

```
┌─────────────────────────────────────────────────────┐
│  MANEUVER WORKSHOP                                  │
├─────────────────────────────────────────────────────┤
│  Editing: Bootlegger Reverse                        │
│  Car: Luxury XL (5,200 lbs, Super Plant, HD Tires)  │
│                                                     │
│  TIMELINE (1 second):                               │
│  Frame:  0----10----20----30----40----50----60     │
│                                                     │
│  Steer:  ████████░░░░░░░░████████░░░░░░░░░░░░░     │
│          [+100%--][  0%  ][ -60% ][   0%    ]      │
│                                                     │
│  Throt:  ░░░░░░░░░░░░████░░░░░░░░░░░░░░░░░░░░     │
│          [   0%     ][ 40%][      0%       ]       │
│                                                     │
│  Brake:  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░     │
│          [           0%                    ]       │
│                                                     │
│  E-Brk:  ░░░░████████████████░░░░░░░░░░░░░░░░     │
│          [OFF][    ON       ][    OFF      ]       │
│                                                     │
│  Entry Speed Range: [25] to [32] mph               │
│  Target Heading: [180]° ± [15]°                    │
│                                                     │
│  [TEST RUN]  [SAVE]  [COPY]  [RESET DEFAULT]       │
├─────────────────────────────────────────────────────┤
│  RECENT TEST RESULTS:                               │
│  28 mph: ✓ 182° (success)                          │
│  32 mph: ✗ 240° (over-rotation, spin)              │
│  25 mph: ✓ 176° (success)                          │
│  35 mph: ✗ vehicle rolled                          │
│                                                     │
│  Success Rate: 60% (6/10) in optimal range         │
└─────────────────────────────────────────────────────┘
```

### Competitive Implications

1. **Preparation matters** - Players tune maneuvers before matches
2. **Car-specific knowledge** - Profiles tied to exact car config
3. **Secret techniques** - Opponents don't see your input sequences
4. **Skill expression** - Better tuning = higher success rates
5. **Emergent meta** - Community discovers optimal techniques
6. **Trading potential** - Share/sell proven maneuver profiles

---

## Dual Execution Modes

Support both physics and cinematic execution for different play styles:

### Mode A: Full Physics

```
Maneuver declared → Physics executes input sequence → Outcome determined by simulation
```

- No Handling Status tracking
- No crash table lookups
- Pure physics outcome
- Add friction variance for unpredictability

**Pros:**
- Most realistic
- Emergent outcomes
- Player skill matters

**Cons:**
- Less predictable
- Harder to balance
- May frustrate casual players

### Mode B: Hybrid (Physics + HS)

```
Maneuver declared → Check HS → Physics executes → HS updated → Recovery check
```

- Keep HS as stress accumulator
- Physics executes but HS affects success probability
- Low HS adds physics perturbations (simulating driver fatigue)

**Pros:**
- Familiar to tabletop players
- Balances skill + chance
- Predictable risk/reward

**Cons:**
- More complex
- Two systems to tune

### Mode C: Cinematic (Current Implementation)

```
Maneuver declared → Kinematic arc calculated → Animation plays → Collision check
```

- Predetermined outcomes
- Classic tabletop feel
- Ghost path preview

**Pros:**
- Most predictable
- Easiest to balance
- True to original rules

**Cons:**
- Less exciting
- No player skill in execution
- Physics only for collisions

---

## What This Preserves from Classic

| Element | Status |
|---------|--------|
| Vehicle construction (chassis, power plant, weapons) | Preserved |
| Turn-based tactical planning | Preserved |
| Simultaneous declaration | Preserved |
| Arena combat | Preserved |
| Weapon allocation | Preserved |
| Speed matters | Enhanced (physics) |
| Consequences for reckless driving | Enhanced (physics) |
| Difficulty values (D0-D7) | Optional (for HS mode) |
| Handling Status | Reworked (accumulated) or removed |
| 5-Phase timing | Removed |
| Crash tables | Removed (physics replaces) |
| Predetermined arcs | Removed |

---

## What This Adds (New)

| Feature | Benefit |
|---------|---------|
| Real physics execution | Immersive, emergent outcomes |
| Player-tuned maneuvers | Skill expression, competitive depth |
| Practice mode | Learning curve, mastery path |
| Car-specific handling | Build matters more |
| Secret techniques | Competitive advantage |
| Dual execution modes | Accessibility options |

---

## Implementation Phases

### Phase 1: Physics Foundation
- [ ] Implement steering/throttle/brake input system
- [ ] Tune vehicle physics for realistic handling
- [ ] Add friction variance for unpredictability
- [ ] Test basic maneuvers (straight, drift, bend)

### Phase 2: Maneuver Profiles
- [ ] Define ManeuverProfile data structure
- [ ] Implement input curve playback system
- [ ] Create default profiles for simple maneuvers
- [ ] Add maneuver success detection

### Phase 3: Maneuver Workshop
- [ ] Build timeline-based editor UI
- [ ] Implement test run functionality
- [ ] Add success rate tracking
- [ ] Create save/load system for profiles

### Phase 4: Dual Mode Support
- [ ] Implement mode switching
- [ ] Port existing cinematic system as Mode C
- [ ] Tune physics mode (Mode A)
- [ ] Implement hybrid HS system (Mode B)

### Phase 5: Competitive Features
- [ ] Player profile storage
- [ ] Maneuver library management
- [ ] Pre-match preparation flow
- [ ] Post-match replay/analysis

---

## Open Questions

1. **Balance:** How to ensure physics mode is competitively fair?
2. **Accessibility:** Will casual players be intimidated by tuning?
3. **Networking:** How to sync physics across multiplayer?
4. **Cheating:** Can players exploit physics edge cases?
5. **Onboarding:** How to teach the maneuver system?

---

## Recommendation

Implement all three modes, defaulting to **Mode B (Hybrid)** for the best balance of:
- Tactical depth (HS accumulation)
- Player skill (tunable maneuvers)
- Excitement (physics outcomes)
- Accessibility (predictable risk)

Allow players to choose Mode A (pure physics) or Mode C (cinematic) based on preference.

---

## References

- [BeamNG Analysis](../research/beamng-analysis.md) - Why pure physics isn't enough
- [6th Edition Comparison](../research/carwars-6th-vs-classic.md) - What makes Car Wars "Car Wars"
- Classic tabletop vehicular combat rules - Original mechanics reference
