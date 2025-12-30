# Car Wars Arena - Development Roadmap

## Current Status (2025-12-29)

### Phase 1: Core Movement System - COMPLETE
- [x] Jolt Physics integration with wheeled vehicles
- [x] Matchbox physics model (direct force application)
- [x] All 21 power plants validated
- [x] Cruise control system
- [x] Turn-based pause/execute flow

### Phase 2: Basic Maneuvers - COMPLETE
- [x] STRAIGHT (D0) - forward movement
- [x] DRIFT (D1) - 1/4" lateral shift
- [x] STEEP_DRIFT (D3) - 1/2" lateral shift
- [x] BEND (D1-D6) - arc turns 15°-90°
- [x] Kinematic animation system
- [x] Multi-phase turn execution (1-5 phases based on speed)
- [x] Speed changes during turns (ACCEL/BRAKE)
- [x] 0 mph start support

### Phase 3: Advanced Maneuvers - IN PROGRESS
- [ ] SWERVE - Drift + opposite bend (lane change with correction)
- [ ] CONTROLLED_SKID - Powerslide (D+1 to D+4)
- [ ] PIVOT - 5 mph only, pivot around rear corner
- [ ] T_STOP - Emergency 90° brake at 10 mph
- [ ] BOOTLEGGER - J-turn 180° at 20-35 mph

### Phase 4: Crash System - PLANNED
- [ ] Crash Table 1-3 implementation
- [ ] Collision detection during maneuvers
- [ ] Hazard triggers (obstacles, debris, oil slicks)
- [ ] Kinematic crash animations (spinouts, rolls, flips)
- [ ] Physics-based crash resolution
- [ ] Vehicle damage model
- [ ] Tire blowouts and handling effects

### Phase 5: Combat System - PLANNED
- [ ] Weapon mounting points
- [ ] Weapon types (MG, laser, rocket, etc.)
- [ ] Firing arcs and targeting
- [ ] Damage resolution
- [ ] Armor and component damage
- [ ] Fire and explosion effects

### Phase 6: Multiplayer - PLANNED
- [ ] Network architecture design
- [ ] Turn synchronization
- [ ] Player lobby system
- [ ] Spectator mode
- [ ] Replay system

### Phase 7: Car Shop Module - PLANNED
- [ ] Vehicle builder GUI
- [ ] 3D body model import
- [ ] Component placement
- [ ] Weight/cost/space calculations
- [ ] Vehicle save/load
- [ ] Pre-built vehicle templates

### Phase 8: Polish - PLANNED
- [ ] Particle effects (smoke, fire, sparks, debris)
- [ ] Sound effects (engines, weapons, crashes)
- [ ] Music and ambient audio
- [ ] UI polish and themes
- [ ] Tutorial/help system

---

## Technical Architecture

### Physics Layer
- **Jolt Physics** for rigid body simulation
- **Matchbox model** for simplified vehicle dynamics
- **Kinematic animation** for predictable maneuver execution
- **Hybrid mode** switches between kinematic (turns) and dynamic (free movement)

### Rendering
- OpenGL 4.6 with Mesa/D3D12 backend
- Box renderer for vehicles and arena
- Line renderer for paths and debug
- Text renderer for UI
- UI panel system

### Game Logic
- Turn-based with 1-second phases
- Car Wars rules for handling, maneuvers, and combat
- Handling Status (HS) tracking with control rolls
- Difficulty values (D0-D7) for maneuvers

---

## File Structure

```
client/
├── src/
│   ├── main.cpp           # Main loop, input, UI
│   ├── game/
│   │   ├── maneuver.cpp   # Maneuver calculations
│   │   ├── maneuver.h     # Maneuver types and structs
│   │   ├── handling.cpp   # HS and control rolls
│   │   └── turn_planning.h # Turn planning structures
│   ├── physics/
│   │   ├── jolt_physics.cpp  # Jolt integration
│   │   └── jolt_physics.h    # Physics API
│   └── render/
│       └── ...             # Rendering systems
└── build/
    └── carwars            # Executable
```

---

## References
- Car Wars Compendium (2nd Edition)
- Car Wars Vehicle Guide
- Jolt Physics documentation
