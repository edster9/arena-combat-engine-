# Arena Combat Engine - Development Roadmap

## Current Status

**Version: 0.1.0-alpha** (January 2025)

The engine has achieved its core architectural goals: physics-based turn execution with Lua scripting. The foundation is solid and ready for gameplay feature development.

---

## Completed Features

### Physics Foundation
- [x] Jolt Physics integration with wheeled vehicles
- [x] Real drivetrain simulation (engine, gearbox, differential)
- [x] Configurable tire friction curves
- [x] Suspension tuning (frequency, damping, travel)
- [x] Arena with walls, obstacles, and ramps
- [x] Cruise control system

### Lua Scripting System ("Reflex Scripts")
- [x] Sol3 integration for C++/Lua binding
- [x] Per-vehicle script instances with isolated state
- [x] Hot-reload support (F5 key)
- [x] Vehicle telemetry API (speed, heading, wheel slip)
- [x] Control output API (throttle, brake, steering)
- [x] Event system for C++ to Lua communication
- [x] Script configuration in vehicle JSON

### Built-in Scripts
- [x] Traction Control System (TCS) - slip-based throttle reduction
- [x] Maneuver Module - turn-based maneuver execution
  - Straight, Drift, Steep Drift, Bend, Swerve
  - Speed change integration (accelerate, hold, brake)

### Turn-Based Mode
- [x] Single maneuver per 1-second turn
- [x] GUI for maneuver and speed selection
- [x] Physics execution with auto-pause on completion
- [x] Speed snapshotting at pause

### Rendering & UI
- [x] SDL2/OpenGL 4.x renderer
- [x] Chase camera with orbit controls
- [x] Turn planning GUI panel
- [x] HUD gauges (speed, RPM, throttle, slip)
- [x] Debug visualization (physics bodies, wheels)

---

## In Progress

### Turn-Based Refinement
- [ ] Ghost path preview for planned maneuvers
- [ ] Maneuver success/failure detection
- [ ] Handling Status system (optional arcade mode)

### Physics Tuning
- [ ] Per-vehicle friction curve tuning
- [ ] Improved suspension presets
- [ ] Weight transfer visualization

---

## Upcoming Features

### Phase 1: Combat System
- [ ] Weapon mounting points
- [ ] Projectile physics
- [ ] Damage model (armor, components)
- [ ] Fire and explosion effects

### Phase 2: Advanced Maneuvers
- [ ] Bootlegger reverse
- [ ] T-stop
- [ ] Controlled skid
- [ ] Pivot turn
- [ ] Player-tunable maneuver profiles

### Phase 3: AI & Automation
- [ ] Basic AI driver scripts
- [ ] Pathfinding for AI
- [ ] Automated testing framework

### Phase 4: Multiplayer
- [ ] Turn synchronization protocol
- [ ] Server authority model
- [ ] Script sharing/validation
- [ ] Lobby and matchmaking

### Phase 5: Polish
- [ ] Sound effects
- [ ] Particle systems
- [ ] Vehicle damage visuals
- [ ] In-game script editor

---

## Technical Architecture

### Physics Layer
- **Jolt Physics** - Rigid body simulation, vehicle constraints
- **Real drivetrain** - Engine torque, gear ratios, differential
- **Tire model** - Slip angles, longitudinal slip, grip limits

### Script Layer
- **Lua via Sol3** - Fast, sandboxed scripting
- **Per-vehicle contexts** - Isolated Lua states per vehicle
- **Event-driven** - C++ sends events, Lua responds with controls

### Game Logic
- **Single-phase turns** - One maneuver per 1-second turn
- **Physics-determined outcomes** - No dice rolls, physics decides
- **Hybrid mode** - Optional Handling Status for arcade feel

---

## Design Principles

1. **Physics First** - All movement comes from physics simulation
2. **Scripts Adapt** - Lua scripts provide closed-loop control
3. **Emergent Outcomes** - Results emerge from physics, not animation
4. **Accessible Depth** - Simple to play, deep to master
5. **VTT Feel** - Virtual tabletop aesthetic, 3D physics reality

---

## Contributing

See the main README for build instructions. Key areas welcoming contribution:

- Maneuver script implementations
- Physics tuning and vehicle configs
- UI/UX improvements
- Documentation

---

## References

- [CONFIG_GUIDE.md](CONFIG_GUIDE.md) - Vehicle and physics configuration
- [vision.md](vision.md) - Visual and gameplay design direction
- [proposals/](proposals/) - Original design proposals (historical reference)