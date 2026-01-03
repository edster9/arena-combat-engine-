# Design Proposals

This folder contains the original design proposals that shaped the Arena Combat Engine architecture.

## Status

These proposals are **historical reference documents**. The core ideas have been implemented:

| Proposal | Status |
|----------|--------|
| [Physics-Based Turn System](physics-based-turn-system.md) | **Implemented** - Single-phase turns with physics execution |
| [Maneuver Scripting System](maneuver-scripting-system.md) | **Implemented** - Lua Reflex Scripts for vehicle control |

## What Was Implemented

From the original proposals:

- Single maneuver per 1-second turn (not 5-phase)
- Real physics execution via Jolt Physics
- Lua scripting via Sol3 for vehicle control
- Per-vehicle script instances with isolated state
- Event system for C++ to Lua communication
- Hot-reload support for development

## What Changed

Some ideas evolved during implementation:

- **Script update rate**: 60 Hz (every physics frame) instead of 15 Hz
- **Maneuver profiles**: Simplified to script-based execution instead of timeline curves
- **Handling Status**: Deferred to future phase (optional arcade mode)

## For Contributors

If you're contributing to the project, the current architecture is documented in:

- [ROADMAP.md](../ROADMAP.md) - Current status and planned features
- [CONFIG_GUIDE.md](../CONFIG_GUIDE.md) - Configuration reference
- [vision.md](../vision.md) - Design direction

The proposals remain useful for understanding the "why" behind architectural decisions.