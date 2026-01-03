# Arena Combat Engine

A 3D turn-based vehicular combat game with real physics execution and player-programmable vehicle control scripts.

## Overview

Arena Combat Engine combines the strategic depth of tabletop-style turn-based gameplay with the excitement of real physics simulation. Players declare maneuvers, then watch as physics determines the outcome - creating emergent moments where skill, preparation, and a bit of luck decide victory.

**Key Features:**
- **Turn-Based Tactics** - Plan your moves carefully; physics executes them
- **Real Physics** - Jolt Physics engine with full drivetrain simulation
- **Lua Scripting** - Write "Reflex Scripts" that control your vehicle during maneuvers
- **VTT Aesthetic** - Virtual tabletop feel with 3D physics reality

## The Vision

Instead of predetermined animation arcs, maneuvers are executed by **Lua Reflex Scripts** that run during physics simulation:

```lua
-- Example: Traction Control Script
function update(vehicle, dt)
    local slip = vehicle:get_wheel_slip()

    if slip.rear_left > 0.15 or slip.rear_right > 0.15 then
        -- Reduce throttle to prevent wheelspin
        return { throttle = 0.5 }
    end

    return { throttle = 1.0 }
end
```

Scripts read vehicle telemetry (position, speed, slip angles) and output controls (steering, throttle, brake). This creates **closed-loop control** that adapts to physics variance - just like real vehicle stability systems.

## Current Features

### Physics Engine
- Jolt Physics with wheeled vehicle simulation
- Real drivetrain: engine, gearbox, differential
- Configurable tire friction curves
- Collision detection with arena walls and obstacles

### Turn-Based Mode
- Single maneuver per 1-second turn
- Speed control (accelerate, hold, brake)
- Physics-based execution with Lua scripts
- Auto-pause when turn completes

### Reflex Scripts (Lua)
- Per-vehicle script instances
- Hot-reload during development (F5)
- Built-in modules: TCS, maneuver execution
- Event system for turn-based integration

### Rendering
- SDL2 window with OpenGL 4.x
- Chase camera with smooth follow and mouse orbit
- Arena with walls, obstacles, and ramps
- Debug visualization for physics bodies
- HUD with speed, RPM, throttle, and wheel slip gauges

### Controls

**Freestyle Mode:**
- Arrow Keys: Throttle, brake, steer
- Space: E-brake
- V: Toggle cruise control
- [/]: Cruise speed down/up
- M: Toggle steering mode (analog/discrete)

**Turn-Based Mode:**
- TAB: Enter/exit turn planning
- Click GUI to select maneuver and speed
- Execute button runs the turn

**Camera:**
- 1-3: Select vehicle + chase camera
- C: Toggle chase camera
- Middle-click + drag: Orbit camera
- Scroll: Zoom in/out

**Debug:**
- P: Toggle physics debug visualization
- R: Reload vehicle config
- F5: Hot-reload scripts
- F1: Show controls help

## Building

### Requirements

- CMake 3.16+
- C++17 compiler (GCC 9+, Clang 10+, or MSVC 2019+)
- SDL2
- GLEW
- Lua 5.4 (fetched automatically if not found)

### Ubuntu / Debian / WSL2

```bash
# Install dependencies
sudo apt update
sudo apt install build-essential cmake libsdl2-dev libglew-dev liblua5.4-dev

# Build
cd client
./build.sh

# Run (auto-detects WSL2 GPU acceleration)
./run.sh
```

### macOS

```bash
# Install dependencies
brew install cmake sdl2 glew lua

# Build and run
cd client
./build.sh
./run.sh
```

### Windows (Visual Studio)

```powershell
# Install vcpkg dependencies
vcpkg install sdl2 glew lua

# Configure and build
cd client
cmake -B build -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

Note: Jolt Physics and Sol3 (Lua bindings) are fetched automatically by CMake.

## Project Structure

```
client/                 # Game client (C++/OpenGL)
  src/
    game/               # Game logic, config loaders
    physics/            # Jolt Physics integration
    render/             # Shaders, camera, mesh rendering
    ui/                 # UI panels and text
    platform/           # SDL2 window/input
    math/               # Vector and matrix math
    script/             # Lua Reflex Script engine

assets/                 # Game assets
  data/
    vehicles/           # Vehicle JSON configs
    equipment/          # Chassis, power plants, tires
  config/scenes/        # Arena definitions
  shaders/              # GLSL shaders
  scripts/              # Lua Reflex Scripts
    modules/            # Reusable script modules

docs/                   # Design documents
  proposals/            # Architecture proposals
  ROADMAP.md            # Development status
  CONFIG_GUIDE.md       # Configuration reference

server/                 # Game server (planned)
```

## Documentation

- [ROADMAP.md](docs/ROADMAP.md) - Development status and planned features
- [CONFIG_GUIDE.md](docs/CONFIG_GUIDE.md) - Vehicle and physics configuration reference
- [vision.md](docs/vision.md) - Visual and gameplay design direction

## Scripting System

Vehicles can have multiple Lua scripts attached that run during physics simulation:

**Built-in Scripts:**
- `tcs.lua` - Traction Control System (prevents wheelspin)
- `maneuver.lua` - Turn-based maneuver execution

**Script API:**
```lua
-- Read vehicle state
local speed = vehicle:get_speed()           -- mph
local slip = vehicle:get_wheel_slip()       -- per-wheel slip ratios
local heading = vehicle:get_heading()       -- degrees

-- Output controls (merged with other scripts)
return {
    throttle = 0.8,     -- 0.0 to 1.0
    brake = 0.0,        -- 0.0 to 1.0
    steering = 0.0,     -- -1.0 to 1.0
}
```

Scripts are configured per-vehicle in JSON:
```json
{
    "scripts": [
        {
            "name": "tcs",
            "path": "scripts/modules/tcs.lua",
            "enabled": true,
            "options": {
                "slip_threshold": 0.12,
                "reduction_rate": 0.7
            }
        }
    ]
}
```

## License

This project uses a dual licensing model:

- **Client (this repository)**: Open source under the MIT License
- **Server and Network Components**: Proprietary (not included in this repository)

See [LICENSE](LICENSE) for details.

The open-source client can connect to official game servers or be used for local single-player and LAN play.