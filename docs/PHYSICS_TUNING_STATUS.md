# Physics Tuning Status

## Current State (2025-12-28)

**Matchbox car physics fully validated** - all 21 power plants tested and passing.

## Matchbox Car Physics Model

Instead of simulating engine physics, we apply direct force to the vehicle body:

```
F = mass × target_acceleration × K
```

Where:
- **mass** = Total vehicle weight in kg
- **target_acceleration** = From Power Factor/weight ratio (m/s²)
- **K** = Per-bucket compensation factor for Jolt wheel resistance + engine RPM ramp

### Engine RPM Ramp-Up

Throttle response simulates engine spin-up for more realistic feel:
- **Ramp up**: 1.5 seconds from idle to full RPM
- **Decay**: 3 seconds from full to idle when coasting
- **Re-engagement**: Fast response if engine still "warm" (RPM > 0)

This means tapping the throttle gives a short burst, while holding gives full power after 1.5s. Quick re-press at speed is near-instant (engine still spun up).

### Per-Bucket K Compensation Factors

K values compensate for:
1. Jolt wheel rolling resistance (constant, not speed-proportional)
2. Engine RPM ramp-up (loses ~0.75s effective acceleration time)

| Bucket | Target 0-60 | K Factor |
|--------|-------------|----------|
| 5 mph/s | 12.0s | 1.75 |
| 10 mph/s | 6.0s | 1.59 |
| 15 mph/s | 4.0s | 1.62 |
| 20 mph/s | 3.0s | 1.55 |

### Why This Works

- Acceleration is constant regardless of speed (no air resistance modeling)
- Top speed comes from Power Factor formula, not physics simulation
- Wheels are cosmetic - steering uses Jolt constraint, but propulsion is body force
- Physics engine handles collisions, flips, jumps, gravity

### Top Speed Formulas

- **Gas**: `240 × PF / (PF + weight)` mph
- **Electric**: `360 × PF / (PF + weight)` mph

## Test Results

All tests performed on Compact chassis (590kg base) with standard tires (mu=2.0).

### Gas Power Plants (13/13 PASS)

| Engine | PF | Weight | PF/Wt | Bucket | Target | Actual | Accuracy | Result |
|--------|-----|--------|-------|--------|--------|--------|----------|--------|
| 10 cid | 300 | 1480 lbs | 0.20 | 5 mph/s | 8.0s (0-40)* | 7.33s | 110% | PASS |
| 30 cid | 500 | 1535 lbs | 0.33 | 5 mph/s | 11.8s (0-59)* | 11.98s | 98% | PASS |
| 50 cid | 700 | 1570 lbs | 0.45 | 5 mph/s | 12.0s | 12.27s | 98% | PASS |
| 100 cid | 1300 | 1685 lbs | 0.77 | 10 mph/s | 6.0s | 6.17s | 97% | PASS |
| 150 cid | 1900 | 1795 lbs | 1.06 | 15 mph/s | 4.0s | 4.08s | 98% | PASS |
| 200 cid | 2500 | 1900 lbs | 1.32 | 15 mph/s | 4.0s | 4.07s | 98% | PASS |
| 250 cid | 3200 | 2135 lbs | 1.50 | 15 mph/s | 4.0s | 4.05s | 99% | PASS |
| 300 cid | 4000 | 2245 lbs | 1.78 | 15 mph/s | 4.0s | 4.05s | 99% | PASS |
| 350 cid | 5000 | 2395 lbs | 2.09 | 15 mph/s | 4.0s | 4.03s | 99% | PASS |
| 400 cid | 6300 | 2470 lbs | 2.55 | 15 mph/s | 4.0s | 4.03s | 99% | PASS |
| 450 cid | 7800 | 2545 lbs | 3.06 | 15 mph/s | 4.0s | 4.02s | 100% | PASS |
| 500 cid | 9500 | 2620 lbs | 3.63 | 15 mph/s | 4.0s | 4.02s | 100% | PASS |
| 700 cid | 13000 | 2695 lbs | 4.82 | 15 mph/s | 4.0s | 4.02s | 100% | PASS |

*Top-speed limited tests - scaled proportionally from 0-60 baseline

### Electric Power Plants (8/8 PASS)

| Power Plant | PF | Weight | PF/Wt | Bucket | Target | Actual | Accuracy | Result |
|-------------|-----|--------|-------|--------|--------|--------|----------|--------|
| Small | 800 | 1920 lbs | 0.42 | 5 mph/s | 12.0s | 12.03s | 100% | PASS |
| Medium | 1400 | 2120 lbs | 0.66 | 10 mph/s | 6.0s | 6.08s | 99% | PASS |
| Large | 2000 | 2320 lbs | 0.86 | 10 mph/s | 6.0s | 6.05s | 99% | PASS |
| Super | 2600 | 2520 lbs | 1.03 | 15 mph/s | 4.0s | 4.03s | 99% | PASS |
| Sport | 3000 | 2420 lbs | 1.24 | 15 mph/s | 4.0s | 4.03s | 99% | PASS |
| Thundercat | 6700 | 3420 lbs | 1.96 | 15 mph/s | 4.0s | 3.98s | 100% | PASS |
| Nuclear | 20000 | 3920 lbs | 5.10 | 15 mph/s | 4.0s | 3.97s | 101% | PASS |
| Nuclear Truck | 50000 | 7420 lbs | 6.74 | 15 mph/s | 4.0s | 3.93s | 102% | PASS |

### Summary

| Type | Tested | Passed | Accuracy Range |
|------|--------|--------|----------------|
| Gas (10-700 cid) | 13 | 13 | 97-110% |
| Electric | 8 | 8 | 99-102% |
| **Total** | **21** | **21** | **97-110%** |

### Not Yet Tested

- **cycle_electric** (5 power plants) - Small cycle, Medium cycle, Large cycle, Super cycle, Super trike

## Key Files

### Physics Implementation
- `client/src/physics/jolt_physics.cpp` - Matchbox force application (lines 280-312)
- `client/src/physics/jolt_physics.h` - VehicleConfig with physics fields

### Configuration
- `client/src/game/config_loader.cpp` - Per-bucket K factors (lines 591-609)
- `assets/data/equipment/power_plants.json` - Power plant definitions
- `assets/data/vehicles/sports_car.json` - Test vehicle config

## Acceleration Test System

Press **T** to start 0-60 acceleration test:
1. Vehicle respawns at origin, stopped
2. Timer starts when throttle is applied
3. Test completes at target speed (60 mph or top speed if lower)
4. Reports time, acceleration, and PASS/FAIL based on bucket range

Press **R** to reload configs and test different power plants.

## Top-Speed-Aware Testing

For vehicles that can't reach 60 mph (e.g., 10 cid gas = 40 mph max):
- Test target = min(60 mph, top_speed)
- Bucket ranges scaled proportionally
- Example: 0-40 test uses 67% of normal time ranges

## Acceleration Buckets

| PF/Weight Ratio | Acceleration | 0-60 Time |
|-----------------|--------------|-----------|
| < 0.5 | 5 mph/s | 12.0s |
| 0.5 - 0.99 | 10 mph/s | 6.0s |
| 1.0 - 1.99 | 15 mph/s | 4.0s |
| 2.0 - 2.99 | 20 mph/s | 3.0s |
| >= 3.0 (nuclear) | 25 mph/s | 2.4s |