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
- Fly camera with mouse look and WASD movement
- Procedural floor grid with distance fog
- Box renderer with directional lighting
- Arena walls and obstacles
- Placeholder vehicles for scale testing

## Building

Requires: CMake 3.16+, SDL2, OpenGL, GLEW

```bash
cd arena
mkdir build && cd build
cmake ..
make
./arena
```

### WSL2 with GPU

```bash
GALLIUM_DRIVER=d3d12 ./arena
```

## Controls

- **Right-click + drag**: Look around
- **WASD**: Move
- **E/Space**: Move up
- **Q/Ctrl**: Move down
- **Shift**: Move faster
- **Scroll**: Adjust speed
- **F11**: Toggle fullscreen
- **ESC**: Quit

## Project Structure

```
arena/            # Combined game and editor
  src/
    platform/     # SDL2 window/input handling
    math/         # Vector and matrix math
    render/       # Shaders, camera, rendering
assets/           # 3D models, textures (CC0 licensed)
docs/             # Design documents and notes
```

## License

See [LICENSE](LICENSE) for details.

## Third-Party Assets

- **Kenney Car Kit** - CC0 (Public Domain) - https://kenney.nl/assets/car-kit

## Disclaimer

This project references and is inspired by classic tabletop games for educational purposes. All trademarks belong to their respective owners. This is a fan learning project with no commercial intent at this stage.
