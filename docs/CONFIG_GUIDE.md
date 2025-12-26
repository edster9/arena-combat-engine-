# Arena Configuration Guide

This guide explains all configuration values for vehicles and scenes in the Arena game engine.

## Vehicle Configuration (`assets/vehicles/*.json`)

### Basic Structure

```json
{
    "class": "wheeled",
    "version": 1,
    "name": "my_vehicle",
    "chassis": { ... },
    "defaults": { ... },
    "wheels": [ ... ],
    "axles": [ ... ],
    "drivetrain": { ... }
}
```

### Top-Level Fields

| Field | Type | Description |
|-------|------|-------------|
| `class` | string | Vehicle type: `"wheeled"` (cars/trucks), future: `"tracked"`, `"hover"` |
| `version` | int | Config format version (currently `1`) |
| `name` | string | Vehicle identifier used in scene configs |

---

## Chassis

The main body of the vehicle.

```json
"chassis": {
    "mass": 1200,
    "length": 3.5,
    "width": 1.8,
    "height": 0.5
}
```

| Field | Unit | Description | Typical Range |
|-------|------|-------------|---------------|
| `mass` | kg | Total chassis mass (heavier = more momentum, slower acceleration) | 600-2000 |
| `length` | meters | Front-to-back dimension (Z axis) | 2.5-5.0 |
| `width` | meters | Side-to-side dimension (X axis) | 1.5-2.5 |
| `height` | meters | Vertical dimension (Y axis) - affects center of gravity | 0.3-1.5 |

**Tips:**
- Lower `height` = lower center of gravity = less likely to flip
- Higher `mass` = more stable but slower to accelerate/brake
- `length`/`width` ratio affects turning behavior

---

## Defaults

Default values used when wheels/axles don't specify their own. Reduces redundancy in config.

```json
"defaults": {
    "wheel": {
        "radius": 0.32,
        "width": 0.25,
        "mass": 18,
        "friction": 3.0,
        "slip": 0.0001
    },
    "suspension": {
        "erp": 0.6,
        "cfm": 0.008,
        "travel": 0.15
    }
}
```

### Wheel Defaults

| Field | Unit | Description | Typical Range |
|-------|------|-------------|---------------|
| `radius` | meters | Wheel radius (larger = higher ground clearance) | 0.25-0.5 |
| `width` | meters | Wheel width (wider = more contact area) | 0.15-0.35 |
| `mass` | kg | Wheel mass (affects rotational inertia) | 10-30 |
| `friction` | coefficient | Tire grip (higher = more traction, less sliding) | 1.0-5.0 |
| `slip` | coefficient | Allowed tire slip (lower = grippier, higher = driftier) | 0.0001-0.01 |

**Friction & Slip Explained:**
- `friction` (mu): How much lateral force the tire can generate before sliding
  - 1.0 = slippery (ice)
  - 3.0 = normal road tire
  - 5.0 = racing slick
- `slip`: How much the tire can slide before losing grip
  - 0.0001 = very grippy, almost no slip allowed
  - 0.001 = normal
  - 0.01 = drift-friendly, tires slide easily

### Suspension Defaults

These are ODE physics engine parameters that control how the suspension behaves.

| Field | Unit | Description | Typical Range |
|-------|------|-------------|---------------|
| `erp` | 0.0-1.0 | Error Reduction Parameter (stiffness) | 0.2-0.8 |
| `cfm` | coefficient | Constraint Force Mixing (softness/damping) | 0.001-0.05 |
| `travel` | meters | Maximum suspension compression distance | 0.1-0.4 |

#### Understanding ERP (Error Reduction Parameter)

ERP controls how aggressively the suspension tries to return to its rest position.

```
ERP = 0.1  →  Very soft, slow to respond (boat-like)
ERP = 0.4  →  Soft, comfortable ride (luxury car)
ERP = 0.6  →  Medium stiffness (sports car)
ERP = 0.8  →  Stiff, responsive (race car)
ERP = 1.0  →  Rigid (no suspension give)
```

**What it does:** When the wheel is pushed up (hitting a bump), ERP determines what fraction of the position error is corrected each physics step. Higher values = faster correction = stiffer feel.

#### Understanding CFM (Constraint Force Mixing)

CFM controls how "soft" the constraint is - essentially the damping.

```
CFM = 0.001  →  Very stiff, little give (race suspension)
CFM = 0.005  →  Firm but compliant (sports car)
CFM = 0.01   →  Soft, absorbs bumps well (comfort)
CFM = 0.05   →  Very soft, bouncy (off-road)
```

**What it does:** CFM adds "softness" to the constraint. Higher values allow more movement before the constraint fully engages. Think of it like the damper fluid viscosity.

#### ERP + CFM Combinations

| Driving Feel | ERP | CFM | Description |
|--------------|-----|-----|-------------|
| Race/Track | 0.7-0.8 | 0.001-0.005 | Stiff, responsive, little body roll |
| Sports | 0.5-0.7 | 0.005-0.01 | Balanced, good feedback |
| Comfort | 0.3-0.5 | 0.01-0.02 | Soft, absorbs bumps, some body roll |
| Off-road | 0.2-0.4 | 0.02-0.05 | Very soft, lots of travel |

#### Suspension Travel

`travel` is simply how far the wheel can move up into the wheel well.

- 0.1m (10cm) = Low-profile sports car
- 0.15m (15cm) = Normal car
- 0.3m (30cm) = SUV/truck
- 0.4m+ = Off-road/monster truck

---

## Wheels

Individual wheel definitions. Values not specified fall back to `defaults.wheel`.

```json
"wheels": [
    { "id": "FL", "position": [-1, -0.9, 1.2], "radius": 0.30, "width": 0.22 },
    { "id": "FR", "position": [1, -0.9, 1.2], "radius": 0.30, "width": 0.22 },
    { "id": "RL", "position": [-1, -0.9, -1.2], "radius": 0.33, "width": 0.28, "mass": 20 },
    { "id": "RR", "position": [1, -0.9, -1.2], "radius": 0.33, "width": 0.28, "mass": 20 }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `id` | string | Wheel identifier: `FL`, `FR`, `RL`, `RR` (Front/Rear, Left/Right) |
| `position` | [x, y, z] | Position relative to chassis center (meters) |
| `radius` | float | Override default radius (optional) |
| `width` | float | Override default width (optional) |
| `mass` | float | Override default mass (optional) |

**Position Coordinates:**
- X: Positive = right, Negative = left
- Y: Vertical offset from chassis center (usually negative = below)
- Z: Positive = front, Negative = rear

**Example:** `[-1, -0.9, 1.2]` = 1m left, 0.9m below chassis center, 1.2m forward

---

## Axles

Group wheels into axles and define their behavior.

```json
"axles": [
    {
        "name": "front",
        "wheels": ["FL", "FR"],
        "steering": true,
        "driven": false,
        "max_steer_angle": 0.6,
        "suspension": {
            "erp": 0.7,
            "cfm": 0.005,
            "travel": 0.12
        }
    },
    {
        "name": "rear",
        "wheels": ["RL", "RR"],
        "steering": false,
        "driven": true,
        "suspension": {
            "erp": 0.7,
            "cfm": 0.005,
            "travel": 0.12
        }
    }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | Axle identifier |
| `wheels` | [string] | List of wheel IDs on this axle |
| `steering` | bool | Can this axle steer? |
| `driven` | bool | Is this axle powered by the motor? |
| `max_steer_angle` | radians | Max steering angle (only if `steering: true`) |
| `suspension` | object | Per-axle suspension override (optional) |

**Steering Angle Reference:**
- 0.3 rad = ~17 degrees (truck, slow steering)
- 0.5 rad = ~29 degrees (normal car)
- 0.7 rad = ~40 degrees (sports car, tight turns)
- 1.0 rad = ~57 degrees (go-kart)

**Per-Axle Suspension:**
You can tune front and rear suspension independently:
- Stiffer front = more responsive steering, less understeer
- Stiffer rear = more stable at speed, less oversteer
- Softer front = more comfortable, more understeer
- Softer rear = tail-happy, easier to drift

---

## Drivetrain

Engine and brake configuration.

```json
"drivetrain": {
    "type": "RWD",
    "motor_force": 15000,
    "brake_force": 25000,
    "max_speed": 80
}
```

| Field | Type | Description | Typical Range |
|-------|------|-------------|---------------|
| `type` | string | `RWD`, `FWD`, `AWD`, `4WD` | - |
| `motor_force` | Newtons | Maximum engine force | 5000-30000 |
| `brake_force` | Newtons | Maximum braking force | 10000-40000 |
| `max_speed` | units/sec | Top speed limiter | 40-120 |

**Drivetrain Types:**
- `RWD` (Rear Wheel Drive): Power to rear wheels. Classic sports car feel, can oversteer.
- `FWD` (Front Wheel Drive): Power to front wheels. Tends to understeer, more stable.
- `AWD` (All Wheel Drive): Power to all wheels. Best traction, balanced handling.
- `4WD` (Four Wheel Drive): Like AWD but with transfer case (future feature).

**Force Guidelines:**
- Economy car: motor 5000-8000N, brake 15000N
- Sports car: motor 15000-20000N, brake 25000N
- Race car: motor 25000-35000N, brake 35000N
- Truck: motor 30000-50000N, brake 50000N

---

## Scene Configuration (`assets/scenes/*.json`)

### Basic Structure

```json
{
    "name": "my_arena",
    "arena": { ... },
    "world": { ... },
    "vehicles": [ ... ],
    "obstacles": [ ... ]
}
```

### Arena

The play area boundaries.

```json
"arena": {
    "size": 100,
    "wall_height": 3,
    "wall_thickness": 0.5,
    "ground_y": 0
}
```

| Field | Unit | Description |
|-------|------|-------------|
| `size` | meters | Arena is `size x size` square |
| `wall_height` | meters | Height of boundary walls |
| `wall_thickness` | meters | Thickness of walls |
| `ground_y` | meters | Y position of ground plane |

### World Physics

Global physics parameters.

```json
"world": {
    "gravity": -9.81,
    "erp": 0.8,
    "cfm": 0.0001,
    "contact": {
        "friction": 1.5,
        "slip": 0.001,
        "soft_erp": 0.5,
        "soft_cfm": 0.001
    }
}
```

| Field | Description |
|-------|-------------|
| `gravity` | Gravity strength (negative = down) |
| `erp` | Global error reduction (world stiffness) |
| `cfm` | Global constraint mixing |
| `contact.friction` | Default surface friction |
| `contact.slip` | Default surface slip |
| `contact.soft_erp` | Contact point stiffness |
| `contact.soft_cfm` | Contact point softness |

### Vehicles

Spawn vehicles in the scene.

```json
"vehicles": [
    {
        "type": "sports_car",
        "team": "red",
        "position": [10, 0, 0],
        "rotation": 0
    }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `type` | string | References vehicle JSON `name` field |
| `team` | string | `"red"` or `"blue"` |
| `position` | [x, y, z] | Spawn position |
| `rotation` | radians | Initial Y-axis rotation |

### Obstacles

Static objects in the arena.

```json
"obstacles": [
    {
        "type": "box",
        "position": [0, 1, 0],
        "size": [2, 2, 2],
        "color": [0.5, 0.5, 0.5]
    }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `type` | string | Currently only `"box"` |
| `position` | [x, y, z] | Center position |
| `size` | [x, y, z] | Dimensions |
| `color` | [r, g, b] | RGB color (0.0-1.0) |

---

## Quick Tuning Recipes

### Stable Cruiser
```json
"defaults": {
    "wheel": { "friction": 2.5, "slip": 0.0005 },
    "suspension": { "erp": 0.4, "cfm": 0.015, "travel": 0.2 }
}
```

### Responsive Sports Car
```json
"defaults": {
    "wheel": { "friction": 3.5, "slip": 0.0002 },
    "suspension": { "erp": 0.7, "cfm": 0.005, "travel": 0.12 }
}
```

### Drift Machine
```json
"defaults": {
    "wheel": { "friction": 2.0, "slip": 0.005 },
    "suspension": { "erp": 0.6, "cfm": 0.008, "travel": 0.15 }
}
```

### Off-Road Truck
```json
"defaults": {
    "wheel": { "friction": 4.0, "slip": 0.001, "radius": 0.45 },
    "suspension": { "erp": 0.3, "cfm": 0.03, "travel": 0.35 }
}
```
