# Physics Tuning Status

## Current State (2025-12-27)

We migrated from ODE to Jolt Physics and implemented a **linear acceleration mode** for strict Car Wars physics.

## Linear Acceleration Mode (NEW)

Instead of tuning engine/transmission parameters to achieve correct acceleration, we now apply direct force:

```
F = mass × target_acceleration
```

This "matchbox car" approach directly implements Car Wars rules:
- Acceleration is constant regardless of current speed (no air resistance)
- Top speed is enforced from Car Wars tables
- No tire spin, clutch slip, or drivetrain losses
- Physics engine still handles collisions, flips, and environmental interaction

### How It Works
1. Calculate target acceleration from PF/weight ratio (5/10/15 mph/s classes)
2. Apply direct body force: `F = m × a × throttle`
3. Cap at top speed from Car Wars tables
4. Wheels are cosmetic - they don't affect movement

### Configuration
In strict Car Wars mode (`physics_modes.json`):
- `use_linear_accel = true` (set automatically)
- `linear_accel_force = mass × target_accel_ms2`
- `top_speed_ms` from power plant tables

## Test Results

### 50 cid Gas Engine (5 mph/s class)
- **Chassis**: Compact (712 kg total)
- **PF/weight ratio**: 700/1570 = 0.45
- **Target**: 12.0s 0-60 (2.24 m/s²)
- **Linear force**: 1595N
- **Top speed**: 74 mph

### Previous K-factor approach (deprecated)
We tried calibrating K factors per acceleration class:
- 15 mph/s class: K=1.23
- 10 mph/s class: K=0.31
- 5 mph/s class: K=0.16 (failed - car topped out at 42 mph)

The linear approach is cleaner and doesn't require per-class calibration.

## Key Files

### Physics Implementation
- `client/src/physics/jolt_physics.cpp` - Linear accel force application
- `client/src/physics/jolt_physics.h` - VehicleConfig with linear accel fields

### Configuration
- `client/src/game/config_loader.cpp` - Enables linear mode in strict Car Wars
- `assets/config/physics_modes.json` - Mode selection

## Future: Full "Matchbox Car" Mode

The current implementation adds linear force but still runs the engine simulation.
A cleaner approach would be:

### Strict Car Wars Mode (matchbox car)
- **Acceleration**: Direct body force F = m × a
- **Steering**: Direct heading rotation (no tire slip angles)
- **Braking**: Direct deceleration force
- **Physics for**: Collisions, flips, jumps, rams, gravity
- **Wheels**: Pure decoration (spin visually, no physics effect)
- No `SetDriverInput()` - bypass engine simulation entirely

### Extended Physics Mode (real simulation)
- Full drivetrain simulation
- Tire friction models
- Engine/transmission/clutch
- Gear ratios matter
- Traction control

## Next Steps

### Immediate (for strict Car Wars)
1. [ ] Test linear accel mode with 0-60 test (should hit 12.0s target exactly)
2. [ ] Implement linear steering (rotate heading directly)
3. [ ] Remove engine simulation dependency in strict mode
4. [ ] Verify all acceleration classes work (50/100/150 cid)

### Later (for extended physics)
1. [ ] Create separate code path for "real physics" mode
2. [ ] Move current engine/transmission tuning to extended mode
3. [ ] Add tire slip angle modeling
4. [ ] Per-tire friction based on equipment

## Quick Test Commands
```bash
cd client && ./build.sh && ./run.sh
```
- **F2** = Start acceleration test
- **Up arrow** = Throttle
- **R** = Reload configs
- Edit `sports_car.json` to change power_plant (gas_50cid, gas_100cid, gas_150cid)