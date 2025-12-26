# Arena Combat Engine

A learning project exploring 3D game engine development in C with OpenGL.

## Project Status

**This is an educational/prototype project.** It is not affiliated with, endorsed by, or connected to any commercial game or trademark holder. The project explores concepts inspired by classic tabletop vehicular combat games for learning purposes only.

## What This Is

- A personal learning project for 3D graphics programming
- An exploration of game engine architecture in C
- A prototype/proof-of-concept, not a commercial product
- Built from scratch using SDL2, OpenGL, and GLEW

## What This Is NOT

- A commercial product
- An official game or licensed adaptation
- Ready for production use

## Current Features

- SDL2 window with OpenGL 3.3+ context
- Chase camera with smooth follow and mouse orbit
- Procedural floor grid with distance fog
- Box renderer with directional lighting
- Arena walls and obstacles
- ODE physics engine integration with:
  - Rigid body vehicle dynamics
  - Per-axle suspension (configurable ERP/CFM)
  - Friction and slip simulation
  - Collision detection
- JSON-based vehicle and scene configuration
- Ghost path preview for planned maneuvers
- Debug visualization for physics bodies

## Building

### Ubuntu / Debian / WSL2

Install dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake libsdl2-dev libglew-dev libode-dev
```

Build and run (OS-aware scripts handle WSL2/Linux/Windows):

```bash
cd client
./build.sh        # Build the client
./run.sh          # Run (auto-detects WSL2 GPU acceleration)
./run.sh --build  # Build and run in one step
```

### Windows (MSYS2 / MinGW64)

1. Install [MSYS2](https://www.msys2.org/)

2. Open **MINGW64** terminal (not MSYS2 or UCRT) and install dependencies:

```bash
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake mingw-w64-x86_64-SDL2 mingw-w64-x86_64-glew mingw-w64-x86_64-ode git
```

3. Fix ODE library naming issue (MSYS2 package bug):

```bash
ln -s /mingw64/lib/libode_double.dll.a /mingw64/lib/libode.dll.a
```

4. Clone and build:

```bash
git clone git@github.com:your-username/arena-combat-engine.git
cd arena-combat-engine/client
./build.sh        # Build (detects MinGW)
./run.sh          # Run
./run.sh --build  # Build and run
```

## Controls

### Driving Mode (Default)

- **W/S**: Accelerate / Brake
- **A/D**: Steer left / right
- **Right-click + drag**: Orbit camera around vehicle
- **Scroll**: Zoom in / out

### Fly Camera Mode

- **Right-click + drag**: Look around
- **WASD**: Move
- **E/Space**: Move up
- **Q/Ctrl**: Move down
- **Shift**: Move faster
- **Scroll**: Adjust speed

### General

- **Tab**: Toggle between driving and fly camera modes
- **G**: Toggle ghost path preview
- **P**: Toggle physics debug visualization
- **F11**: Toggle fullscreen
- **ESC**: Quit

## Project Structure

```
assets/                 # Shared game assets
  config/               # JSON vehicle and scene configs
    vehicles/           # Vehicle definitions
    scenes/             # Scene/arena definitions
  fonts/                # TTF fonts
  models/               # 3D models (OBJ)
  shaders/              # GLSL shaders
  textures/             # Texture images
  audio/                # Sound effects and music

client/                 # Game client (C/OpenGL)
  src/
    platform/           # SDL2 window/input handling
    math/               # Vector and matrix math
    render/             # Shaders, camera, mesh rendering
    physics/            # ODE physics integration
    game/               # Entity system, config loader
    ui/                 # UI rendering and text
    vendor/             # Third-party code (cJSON)

server/                 # Game server (Go)
  cmd/server/           # Server entry point
  internal/
    net/                # Client connections, messaging
    lobby/              # Room management, matchmaking
    game/               # Game state, turn logic

docs/                   # Design documents and guides
```

## Configuration

Vehicle and scene parameters are defined in JSON files under `assets/config/`. See [CONFIG_GUIDE.md](docs/CONFIG_GUIDE.md) for detailed documentation on all configuration options including suspension tuning (ERP/CFM).

## License

See [LICENSE](LICENSE) for details.

## Third-Party Assets

- Vehicle models pending (placeholder rendering currently used)

## Disclaimer

This project references and is inspired by classic tabletop games for educational purposes. All trademarks belong to their respective owners. This is a fan learning project with no commercial intent at this stage.
