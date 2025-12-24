# Planning Phase UI Design

## Overview

The Planning Phase is where the player inputs their vehicle's orders for the upcoming turn. This happens **before** execution - all players commit their plans, then everything resolves simultaneously.

## Design Goals

1. **Intuitive** - A Car Wars fan should recognize the concepts
2. **Quick** - Planning shouldn't take longer than execution
3. **Visual** - Show the path preview, not just numbers
4. **Informative** - Display risk (difficulty) clearly

---

## Screen Layout

```
┌────────────────────────────────────────────────────────────────────────────┐
│ TURN 3 - PLANNING PHASE                                15 sec   [SKIP AI]  │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌─────────────────────────────────┐  ┌─────────────────────────────────┐  │
│  │                                 │  │                                 │  │
│  │                                 │  │     CONTROL PANEL               │  │
│  │                                 │  │                                 │  │
│  │                                 │  │  ┌───────────────────────────┐  │  │
│  │                                 │  │  │ SPEED                     │  │  │
│  │      3D ARENA VIEW              │  │  │ Current: 45 mph           │  │  │
│  │      (70% of screen)            │  │  │ [◄-10] [HOLD] [+10►]      │  │  │
│  │                                 │  │  │ New: 55 mph ↑             │  │  │
│  │   - Ghost path preview          │  │  └───────────────────────────┘  │  │
│  │   - Selected vehicle highlighted│  │                                 │  │
│  │   - Other vehicles shown        │  │  ┌───────────────────────────┐  │  │
│  │   - Firing arcs (optional)      │  │  │ MANEUVERS (5 phases)      │  │  │
│  │                                 │  │  │                           │  │  │
│  │                                 │  │  │  P1  P2  P3  P4  P5       │  │  │
│  │                                 │  │  │ [↑] [↑] [↗] [↑] [↑]       │  │  │
│  │                                 │  │  │  D0  D0  D2  D0  D0       │  │  │
│  │                                 │  │  │                           │  │  │
│  │                                 │  │  │ [Maneuver Buttons...]     │  │  │
│  │                                 │  │  └───────────────────────────┘  │  │
│  │                                 │  │                                 │  │
│  │                                 │  │  ┌───────────────────────────┐  │  │
│  │                                 │  │  │ WEAPONS                   │  │  │
│  │                                 │  │  │ MG Front: [---]           │  │  │
│  │                                 │  │  │ RL Front: [---]           │  │  │
│  │                                 │  │  └───────────────────────────┘  │  │
│  │                                 │  │                                 │  │
│  └─────────────────────────────────┘  │  ┌───────────────────────────┐  │  │
│                                       │  │ RISK SUMMARY              │  │  │
│  ┌─────────────────────────────────┐  │  │ Handling: 3 → 1  ✓ Safe   │  │  │
│  │ VEHICLE STATUS BAR              │  │  │ Total Diff: D2            │  │  │
│  │ HP: ████████░░ 80%  Armor: F10  │  │  └───────────────────────────┘  │  │
│  │ Speed: 45→55  HC: 3             │  │                                 │  │
│  └─────────────────────────────────┘  │  [◄ BACK]         [CONFIRM ►]   │  │
│                                       └─────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Control Panel Sections

### 1. SPEED CONTROL

```
┌─────────────────────────────────────────┐
│ SPEED                                   │
│                                         │
│ Current Speed: 45 mph                   │
│                                         │
│   ┌─────┐  ┌──────┐  ┌─────┐            │
│   │ -10 │  │ HOLD │  │ +10 │            │
│   │BRAKE│  │      │  │ GAS │            │
│   └─────┘  └──────┘  └─────┘            │
│         ▲                               │
│     selected                            │
│                                         │
│ New Speed: 55 mph  (↑ Accelerating)     │
│ Max Accel: 15 mph  Max Decel: 10 mph    │
│                                         │
│ ⚠ Braking >10 mph requires control roll │
└─────────────────────────────────────────┘
```

**Interactions:**

- Click buttons or use keyboard (A/D or Left/Right)
- Shows acceleration/deceleration limits
- Warns if hard braking selected

---

### 2. MANEUVER PLANNER

```
┌───────────────────────────────────────────────────────────┐
│ MANEUVERS                                    Total: D2    │
│                                                           │
│ At 55 mph you move in 5 phases:                           │
│                                                           │
│    Phase 1    Phase 2    Phase 3    Phase 4    Phase 5    │
│   ┌───────┐  ┌───────┐  ┌───────┐  ┌───────┐  ┌───────┐   │
│   │  1.5" │  │  1"   │  │  1"   │  │  1"   │  │  1"   │   │
│   │       │  │       │  │       │  │       │  │       │   │
│   │  [↑]  │  │  [↑]  │  │  [↗]  │  │  [↑]  │  │  [↑]  │   │
│   │ STRT  │  │ STRT  │  │ B30 R │  │ STRT  │  │ STRT  │   │
│   │  D0   │  │  D0   │  │  D2   │  │  D0   │  │  D0   │   │
│   └───────┘  └───────┘  └───────┘  └───────┘  └───────┘   │
│                            ▲                              │
│                        selected                           │
│                                                           │
│ ───────────────────────────────────────────────────────── │
│ SELECT MANEUVER FOR PHASE 3:                              │
│                                                           │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐   │
│  │STRAIGHT│ │DRIFT L │ │DRIFT R │ │BEND 15L│ │BEND 15R│   │
│  │   D0   │ │   D1   │ │   D1   │ │   D1   │ │   D1   │   │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘   │
│                                                           │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐   │
│  │BEND 30L│ │BEND 30R│ │BEND 45L│ │BEND 45R│ │BEND 60L│   │
│  │   D2   │ │   D2   │ │   D3   │ │   D3   │ │   D4   │   │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘   │
│                                                           │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐              │
│  │BEND 60R│ │BEND 90L│ │BEND 90R│ │ SWERVE │  [MORE ▼]    │
│  │   D4   │ │   D6   │ │   D6   │ │  D3+   │              │
│  └────────┘ └────────┘ └────────┘ └────────┘              │
└───────────────────────────────────────────────────────────┘
```

**Visual Symbols for Maneuvers:**

```
↑  = Straight
↗  = Bend Right (small angle)
→  = Bend Right (large angle)
↖  = Bend Left (small angle)
←  = Bend Left (large angle)
⇄  = Drift
↺  = Bootlegger
⊗  = T-Stop
```

**Phase Box States:**

- **Normal**: White background
- **Selected**: Blue highlight border
- **Half-move**: Grey background (no maneuver allowed)
- **Risky (D4+)**: Yellow/orange background
- **Dangerous (D6+)**: Red background

---

### 3. WEAPON TARGETING

```
┌─────────────────────────────────────────┐
│ WEAPONS                                 │
│                                         │
│ ┌─────────────────────────────────────┐ │
│ │ [■] Machine Gun (Front)             │ │
│ │     Target: Enemy Bruiser           │ │
│ │     Status: Will fire Phase 1,3,5   │ │
│ └─────────────────────────────────────┘ │
│                                         │
│ ┌─────────────────────────────────────┐ │
│ │ [ ] Rocket Launcher (Front)         │ │
│ │     Target: Not set                 │ │
│ │     Ammo: 8/10                      │ │
│ └─────────────────────────────────────┘ │
│                                         │
│ Click arena to set target, or:          │
│ [AUTOFIRE] [HOLD FIRE] [CLEAR]          │
└─────────────────────────────────────────┘
```

**Firing Arc Visualization:**
When weapon panel is active, show firing arc in 3D view:

```
        60° arc
         /\
        /  \
       /    \
      /      \
     /________\
        CAR
```

---

### 4. RISK SUMMARY

```
┌─────────────────────────────────────────┐
│ RISK ASSESSMENT                         │
│                                         │
│ Handling Class: 3                       │
│ Starting Status: 3                      │
│ After maneuvers: 1                      │
│                                         │
│ ┌─────────────────────────────────────┐ │
│ │ ████████████░░░░░░░░  HC: 1         │ │
│ │ SAFE - No control roll needed       │ │
│ └─────────────────────────────────────┘ │
│                                         │
│ Or with risky plan:                     │
│ ┌─────────────────────────────────────┐ │
│ │ ██████░░░░░░░░░░░░░░  HC: -2        │ │
│ │ ⚠ RISKY - Roll 3+ to keep control   │ │
│ └─────────────────────────────────────┘ │
│                                         │
│ Or with dangerous plan:                 │
│ ┌─────────────────────────────────────┐ │
│ │ ██░░░░░░░░░░░░░░░░░░  HC: -5        │ │
│ │ ⛔ DANGER - Roll 6 or crash!        │ │
│ └─────────────────────────────────────┘ │
└─────────────────────────────────────────┘
```

---

## 3D Arena View Elements

### Ghost Path Preview

```
                    Phase 5 end
                        ●
                       /
                      / Phase 4
                     ●
                    /
     Phase 3 turn→ ●----● Phase 3 end
                  /
                 / Phase 2
                ●
               /
    Phase 1   /
    ●--------●
    ▲
  START (current position)
```

- **Solid line**: Committed path
- **Dotted line**: Preview (updates as you change maneuvers)
- **Circles**: Phase endpoints
- **Colors**: Green=safe, Yellow=risky, Red=dangerous

### Vehicle Highlight

- Selected vehicle: Bright outline, name label
- Enemy vehicles: Red tint
- Friendly vehicles: Blue tint
- Ghost of final position: Semi-transparent

---

## Input Flow

### Keyboard Shortcuts

```
Tab / Shift+Tab  = Cycle through phases
1-5              = Select phase directly
A / D            = Decrease/Increase speed
← / →            = Select maneuver direction
↑ / ↓            = Select maneuver type
Space            = Confirm current selection
Enter            = Lock in plan
Escape           = Cancel / Go back
F                = Toggle firing arcs
P                = Toggle path preview
```

### Mouse

- Click phase box to select it
- Click maneuver button to set it
- Click arena to set weapon target
- Scroll wheel to adjust speed

---

## State Machine

```
                   ┌──────────────┐
                   │  START TURN  │
                   └──────┬───────┘
                          │
                          ▼
                   ┌──────────────┐
         ┌─────────│ SELECT SPEED │◄────────┐
         │         └──────┬───────┘         │
         │                │                 │
         │ [BACK]         │ [NEXT]          │ [BACK]
         │                ▼                 │
         │         ┌──────────────┐         │
         │         │PLAN MANEUVERS│─────────┤
         │         │  (per phase) │         │
         │         └──────┬───────┘         │
         │                │                 │
         │                │ [NEXT]          │
         │                ▼                 │
         │         ┌──────────────┐         │
         └────────►│  TARGET WPNS │─────────┘
                   │  (optional)  │
                   └──────┬───────┘
                          │
                          │ [CONFIRM]
                          ▼
                   ┌──────────────┐
                   │  PLAN LOCKED │
                   │ (wait for    │
                   │  other cars) │
                   └──────────────┘
```

---

## Simplified Mode (Demo)

For the initial demo, we can simplify:

```
┌────────────────────────────────────────┐
│ YOUR TURN                              │
│                                        │
│ Speed: [◄ SLOWER] [45 mph] [FASTER ►]  │
│                                        │
│ Direction:                             │
│ ┌─────┐ ┌─────┐ ┌─────┐                │
│ │ ←   │ │  ↑  │ │  →  │                │
│ │LEFT │ │STRT │ │RIGHT│                │
│ └─────┘ └─────┘ └─────┘                │
│                                        │
│ [CONFIRM MOVE]                         │
└────────────────────────────────────────┘
```

This treats the whole turn as one decision:

- Speed up / slow down / maintain
- Go straight / turn left / turn right

The system then auto-generates the 5-phase plan.

---

## Color Palette

| Element    | Color  | Hex     |
| ---------- | ------ | ------- |
| Safe       | Green  | #4CAF50 |
| Caution    | Yellow | #FFC107 |
| Danger     | Red    | #F44336 |
| Selected   | Blue   | #2196F3 |
| Disabled   | Grey   | #9E9E9E |
| Background | Dark   | #1A1A2E |
| Panel      | Darker | #16213E |
| Text       | White  | #FFFFFF |
| Accent     | Orange | #FF6B35 |

---

## Next Steps

1. [ ] Implement basic panel rendering (rectangles, text)
2. [ ] Add speed control with keyboard input
3. [ ] Add phase display (read-only first)
4. [ ] Add maneuver selection
5. [ ] Add path preview calculation
6. [ ] Add path preview rendering
7. [ ] Add weapon targeting
8. [ ] Add risk assessment display
9. [ ] Polish: animations, sounds, etc.
