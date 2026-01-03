# Configuration Guide

This guide explains all configuration values used in the game engine. Use this as a reference when tuning vehicles, physics, and scenes.

---

## Table of Contents

1. [Physics Concepts](#physics-concepts)
2. [Suspension Configuration](#suspension-configuration)
3. [Chassis Configuration](#chassis-configuration)
4. [Tire Configuration](#tire-configuration)
5. [Power Plant Configuration](#power-plant-configuration)
6. [Vehicle Classes](#vehicle-classes)
7. [Vehicle JSON Structure](#vehicle-json-structure)
8. [Scene JSON Structure](#scene-json-structure)
9. [Script Configuration](#script-configuration)

---

## Physics Concepts

### Jolt Physics Engine

This game uses **Jolt Physics**, a modern physics engine. Key concepts:

| Concept | What It Does |
|---------|--------------|
| Rigid Bodies | Solid objects that can move, rotate, and collide |
| Constraints | Rules that connect bodies (e.g., wheels to chassis) |
| Forces | Push/pull applied to bodies (throttle, gravity) |

### Units Used

| Quantity | Unit | Notes |
|----------|------|-------|
| Mass | kg | kilograms (1 lb = 0.454 kg) |
| Length/Position | meters | (1 foot = 0.305 m) |
| Speed | m/s | meters per second (1 mph = 0.447 m/s) |
| Force | Newtons | kg * m/s² |
| Angle | radians | (180° = π ≈ 3.14 rad) |
| Frequency | Hz | cycles per second |
| Time | seconds | |

---

## Suspension Configuration

Suspension connects wheels to the chassis and determines how the vehicle handles bumps and weight transfer.

### Parameters (Jolt Style)

```json
"suspension": {
    "frequency": 3.0,
    "damping": 0.8,
    "travel": 0.10
}
```

### Frequency (Hz) - Spring Stiffness

**What it is:** How fast the spring oscillates if you push the car down and release it.

**Think of it as:** Spring stiffness. Higher = stiffer springs.

| Value | Feel | Real-World Example |
|-------|------|-------------------|
| 0.5 Hz | Very soft, boat-like | Old luxury sedan |
| 1.0 Hz | Soft, comfortable | Modern sedan |
| 1.5 Hz | Medium, sporty | Sports car |
| 2.0 Hz | Firm, responsive | Track car |
| 3.0 Hz | Very stiff, minimal roll | Race car / Arena vehicle |

**Why it matters:**
- **Lower frequency** = softer ride, more body roll in turns, wheels stay on rough ground better
- **Higher frequency** = stiffer ride, less body roll, more responsive handling, but may bounce on bumps

**Technical explanation:** Frequency determines spring constant via `k = (2π * frequency)² * mass`. A 1.0 Hz spring on a 1000kg car has k ≈ 39,478 N/m.

### Damping (Ratio) - Shock Absorbers

**What it is:** How quickly oscillations die out after hitting a bump.

**Think of it as:** The shock absorber strength. Higher = faster settling.

| Value | Behavior | Analogy |
|-------|----------|---------|
| 0.0 | Undamped | Pogo stick - bounces forever |
| 0.3 | Underdamped | Comfortable - bounces a few times |
| 0.5 | Critically damped | Returns smoothly with no bounce |
| 0.7 | Slightly overdamped | Quick return, very minimal overshoot |
| 1.0 | Overdamped | Slow return, no bounce at all |
| 0.8 | **Our default** | Fast return, stable for combat |

**Why it matters:**
- **Too low** (< 0.3) = car bounces repeatedly, unstable
- **Just right** (0.5-0.8) = smooth, controlled handling
- **Too high** (> 1.0) = sluggish response, suspension feels "dead"

**Technical explanation:** Damping ratio = c / (2 * sqrt(k * m)). Value of 1.0 = critical damping (fastest return without oscillation).

### Travel (meters) - Suspension Stroke

**What it is:** How far the wheel can move up/down from its rest position.

**Think of it as:** How much the wheel can compress/extend.

| Value | Type | Use Case |
|-------|------|----------|
| 0.05m (5cm) | Very short | Lowered sports car |
| 0.10m (10cm) | Short | **Our default** - Arena car |
| 0.15m (15cm) | Normal | Street car |
| 0.25m (25cm) | Long | SUV |
| 0.40m (40cm) | Very long | Off-road truck |

**Why it matters:**
- **Too short** = wheels lose contact on bumps, car bottoms out
- **Too long** = more body roll, slower response

### Suspension Tuning Recipes

| Style | Frequency | Damping | Travel | Description |
|-------|-----------|---------|--------|-------------|
| Arena Default | 3.0 | 0.8 | 0.10 | Stiff, stable for combat |
| Comfortable | 1.0 | 0.5 | 0.15 | Soft, absorbs bumps |
| Sporty | 1.5 | 0.6 | 0.12 | Balanced handling |
| Track Race | 2.0 | 0.7 | 0.08 | Minimal roll, max response |
| Off-Road | 1.2 | 0.5 | 0.30 | Soft, lots of travel |

---

## Chassis Configuration

The chassis is the vehicle body - its size and weight.

### Parameters

```json
"chassis": {
    "mass": 590,
    "length": 4.2,
    "width": 1.7,
    "height": 1.4,
    "center_of_mass": [0, -1.1, 0]
}
```

### Mass (kg)

**What it is:** Total weight of the chassis body (without power plant, which is added separately).

**Why it matters:**
- Heavier = more momentum, harder to accelerate/brake, more stable
- Lighter = quicker acceleration, easier to flip

| Chassis Type | Weight (lbs) | Mass (kg) |
|--------------|--------------|-----------|
| Subcompact | 1000 | 454 |
| Compact | 1300 | 590 |
| Mid-sized | 1600 | 726 |
| Sedan | 1700 | 771 |
| Luxury | 1800 | 816 |
| Van | 2000 | 907 |

### Length / Width / Height (meters)

**What they are:** Physical dimensions of the chassis bounding box.

| Dimension | Axis | Description |
|-----------|------|-------------|
| Length | Z | Front-to-back (driving direction) |
| Width | X | Side-to-side (left/right) |
| Height | Y | Bottom-to-top (up/down) |

**Why they matter:**
- Affect collision box size
- Affect moment of inertia (resistance to rotation)
- Longer vehicles turn wider, taller vehicles flip easier

### Center of Mass (CoM)

**What it is:** The balance point of the vehicle, as an offset from the geometric center.

**Format:** `[x, y, z]` in meters

| Offset | Meaning |
|--------|---------|
| x = 0 | Centered left-right |
| x > 0 | Right-heavy |
| x < 0 | Left-heavy |
| y = 0 | Centered vertically |
| y < 0 | **Low CoM** (more stable) |
| y > 0 | High CoM (tips easier) |
| z = 0 | Centered front-back |
| z > 0 | Front-heavy |
| z < 0 | Rear-heavy |

**Normalized Y values:**
- `y = 0` = center of chassis
- `y = -0.9` = 90% down toward bottom (default for stability)
- `y = -1.1` = even lower (compact car default)

**Why it matters:**
- **Lower CoM (negative Y)** = harder to flip, more stable in turns
- **Forward CoM (positive Z)** = better front grip, tendency to understeer
- **Rear CoM (negative Z)** = better rear grip, tendency to oversteer

---

## Tire Configuration

### Friction Coefficient (mu / μ)

**What it is:** How much grip the tire has. Multiplier for lateral/longitudinal force.

```json
"physics": {
    "mu": 2.0
}
```

| Value | Grip Level | Use Case |
|-------|------------|----------|
| 0.5 | Very low | Ice |
| 1.0 | Low | Wet pavement |
| 1.5 | Normal | Dry road |
| 2.0 | **Standard** | Arena default |
| 3.0 | High | Racing slick |
| 5.0 | Very high | Maximum grip |

**Technical explanation:** Maximum friction force = mu × normal_force. With mu=2.0 and 250kg per wheel, max grip ≈ 4900 N per wheel.

### Friction Curves

For more realistic tire behavior, you can define friction curves:

```json
"friction": {
    "longitudinal": {
        "max_slip": 0.12,
        "slip_curve": [[0.0, 0.0], [0.06, 0.9], [0.12, 1.0], [0.20, 0.95], [1.0, 0.8]]
    },
    "lateral": {
        "max_slip_angle": 8.0,
        "slip_curve": [[0.0, 0.0], [4.0, 0.9], [8.0, 1.0], [15.0, 0.9], [30.0, 0.7]]
    }
}
```

---

## Power Plant Configuration

### Power Factors (PF)

**What it is:** Measure of engine power. Used to calculate acceleration and top speed.

| Engine Type | Power Factors |
|-------------|---------------|
| Small Gas | 300-800 |
| Medium Gas | 1000-1500 |
| Large Gas | 2000-3000 |
| Small Electric | 800 |
| Medium Electric | 1400 |
| Large Electric | 2600 |

### Acceleration Classes

Acceleration is determined by the Power Factor to Weight ratio:

| PF/Weight Ratio | Acceleration | 0-60 Time |
|-----------------|--------------|-----------|
| < 0.33 | Won't move | - |
| 0.33 - 0.49 | 5 mph/s | 12.0s |
| 0.50 - 0.99 | 10 mph/s | 6.0s |
| 1.00 - 1.99 | 15 mph/s | 4.0s |
| 2.00 - 2.99 | 20 mph/s | 3.0s |
| ≥ 3.00 | 25 mph/s | 2.4s |

### Top Speed Formulas

**Gas engines:**
```
Top Speed (mph) = 240 × PF / (PF + weight_lbs)
```

**Electric motors:**
```
Top Speed (mph) = 360 × PF / (PF + weight_lbs)
```

---

## Vehicle Classes

### Handling Class (HC)

**What it is:** Base modifier for maneuver difficulty. Higher = better handling.

| HC | Handling |
|----|----------|
| -1 | Poor (vans) |
| 0 | Normal |
| 1 | Good (subcompact) |
| 2 | Great (sport suspension) |
| 3+ | Excellent (racing setup) |

**Affected by:**
- Chassis type (vans -1, subcompacts +1)
- Suspension upgrades (+1 to +3)
- Tire type (sport +1, racing +2)

---

## Vehicle JSON Structure

Full vehicle configuration file:

```json
{
    "class": "wheeled",
    "version": 1,
    "name": "my_vehicle",

    "chassis": {
        "mass": 590,
        "length": 4.2,
        "width": 1.7,
        "height": 1.4,
        "center_of_mass": [0, -1.1, 0]
    },

    "defaults": {
        "wheel": {
            "radius": 0.32,
            "width": 0.25,
            "mass": 18,
            "friction": 2.0
        },
        "suspension": {
            "frequency": 3.0,
            "damping": 0.8,
            "travel": 0.10
        }
    },

    "wheels": [
        { "id": "FL", "position": [-0.85, -0.5, 1.1] },
        { "id": "FR", "position": [0.85, -0.5, 1.1] },
        { "id": "RL", "position": [-0.85, -0.5, -1.1] },
        { "id": "RR", "position": [0.85, -0.5, -1.1] }
    ],

    "axles": [
        {
            "name": "front",
            "wheels": ["FL", "FR"],
            "steering": true,
            "axle_power": 0.0,
            "max_steer_angle": 0.6
        },
        {
            "name": "rear",
            "wheels": ["RL", "RR"],
            "steering": false,
            "axle_power": 1.0
        }
    ],

    "scripts": [
        {
            "name": "tcs",
            "path": "scripts/modules/tcs.lua",
            "enabled": true
        }
    ]
}
```

### Top-Level Fields

| Field | Description |
|-------|-------------|
| `class` | Vehicle type: `wheeled`, `tracked`, `hover` (future) |
| `version` | Config format version |
| `name` | Identifier used in scene configs |

### Wheel Position Convention

Wheel positions are relative to chassis center:

| Axis | Positive | Negative |
|------|----------|----------|
| X | Right | Left |
| Y | Up | Down |
| Z | Front | Rear |

Standard wheel IDs:
- `FL` = Front Left
- `FR` = Front Right
- `RL` = Rear Left
- `RR` = Rear Right

### Axle Power Distribution

Power distribution is controlled per-axle using `axle_power` (0.0 to 1.0):

| Configuration | Front | Rear | Description |
|---------------|-------|------|-------------|
| RWD | 0.0 | 1.0 | Rear Wheel Drive |
| FWD | 1.0 | 0.0 | Front Wheel Drive |
| AWD | 0.5 | 0.5 | All Wheel Drive (50/50) |
| AWD Rear-Bias | 0.3 | 0.7 | Rear-biased AWD |

### Steering Angle Reference

| Radians | Degrees | Use Case |
|---------|---------|----------|
| 0.3 | ~17° | Truck (slow steering) |
| 0.5 | ~29° | Normal car |
| 0.6 | ~34° | Sports car |
| 0.8 | ~46° | Agile vehicle |
| 1.0 | ~57° | Go-kart |

---

## Scene JSON Structure

Scene files define the arena and spawn vehicles:

```json
{
    "name": "arena_01",

    "arena": {
        "size": 100,
        "wall_height": 3,
        "wall_thickness": 0.5,
        "ground_y": 0
    },

    "world": {
        "gravity": -9.81,
        "contact": {
            "friction": 1.5,
            "restitution": 0.3
        }
    },

    "vehicles": [
        {
            "type": "sports_car",
            "team": "red",
            "position": [10, 0, 0],
            "rotation": 0
        }
    ],

    "obstacles": [
        {
            "type": "ramp",
            "position": [0, 0, 20],
            "size": [4, 1.5, 8],
            "rotation_y": 0
        }
    ]
}
```

### Arena Fields

| Field | Description |
|-------|-------------|
| `size` | Arena width/length in meters (square) |
| `wall_height` | Boundary wall height |
| `wall_thickness` | Wall collision thickness |
| `ground_y` | Ground plane Y position |

### World Physics

| Field | Description |
|-------|-------------|
| `gravity` | Acceleration due to gravity (negative = down) |
| `contact.friction` | Default surface friction |
| `contact.restitution` | Bounciness (0 = no bounce, 1 = full bounce) |

### Obstacle Types

| Type | Description |
|------|-------------|
| `box` | Rectangular solid |
| `ramp` | Inclined surface (size: width × height × length) |

---

## Script Configuration

Vehicles can have Lua scripts attached for custom behavior:

```json
"scripts": [
    {
        "name": "tcs",
        "path": "scripts/modules/tcs.lua",
        "enabled": true,
        "options": {
            "slip_threshold": 0.12,
            "reduction_rate": 0.7
        }
    },
    {
        "name": "maneuver",
        "path": "scripts/modules/maneuver.lua",
        "enabled": true
    }
]
```

### Script Fields

| Field | Description |
|-------|-------------|
| `name` | Script identifier (for logging/debugging) |
| `path` | Path to Lua file (relative to assets/) |
| `enabled` | Whether to run this script |
| `options` | Key-value pairs passed to script |

### Built-in Scripts

| Script | Purpose |
|--------|---------|
| `tcs.lua` | Traction Control - reduces throttle on wheel slip |
| `maneuver.lua` | Turn-based maneuver execution |

---

## Quick Reference

### Common Conversions

| From | To | Multiply By |
|------|----|-------------|
| lbs | kg | 0.454 |
| mph | m/s | 0.447 |
| degrees | radians | 0.0175 (π/180) |
| feet | meters | 0.305 |

### Default Values Summary

| Parameter | Value | Notes |
|-----------|-------|-------|
| Suspension frequency | 3.0 Hz | Very stiff (race car) |
| Suspension damping | 0.8 | Well-damped, fast response |
| Suspension travel | 0.10 m | Short stroke |
| Tire friction (mu) | 2.0 | Standard grip |
| Center of mass Y | -0.9 to -1.1 | Low for stability |
| Gravity | -9.81 m/s² | Earth standard |