# Assets: Art, Models, and Audio

## Philosophy

Assets should be developed in parallel with code, but **placeholders are acceptable** to keep momentum. A working game with cubes is better than a stalled project waiting for art.

---

## Asset Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                     ASSET WORKFLOW                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  SOURCE FILES          EXPORT              IN-GAME          │
│  ────────────          ──────              ───────          │
│                                                             │
│  Blender (.blend)  ──► OBJ/glTF ─────────► Mesh             │
│                                                             │
│  Photoshop/GIMP    ──► PNG (power of 2) ─► Texture          │
│  (.psd/.xcf)                                                │
│                                                             │
│  Audacity          ──► WAV/OGG ──────────► Sound            │
│  (.aup)                                                     │
│                                                             │
│  Hand-written      ──► GLSL ─────────────► Shader           │
│  (.vert, .frag)                                             │
│                                                             │
│  JSON files        ──► JSON ─────────────► Game Data        │
│  Lua scripts       ──► Lua ──────────────► Reflex Scripts   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3D Models Required

### Current Assets

| Asset | Description | Status |
|-------|-------------|--------|
| **Vehicles** | From Kenney Car Kit | Placeholder |
| **Arena Floor** | Procedural shader | Working |
| **Arena Walls** | Box primitives | Working |
| **Obstacles** | Box/ramp primitives | Working |

### Future Assets

| Asset | Description |
|-------|-------------|
| **Combat Vehicles** | Stylized cars with visible weapons |
| **Weapon Models** | Machine guns, rockets, lasers |
| **Debris** | Scattered parts for destruction |
| **Wrecks** | Destroyed vehicle husks |

### Model Specifications

```
Polycount targets (per vehicle):
  - Placeholder: 12 triangles (box)
  - Low-poly: 500-1000 triangles
  - Final: 2000-5000 triangles

Coordinate system:
  - Y-up
  - Front of vehicle faces +Z
  - Origin at vehicle center, on ground plane

Scale:
  - 1 unit = 1 meter
  - Typical car: ~4.5 units long
```

---

## Textures Required

| Asset | Size | Description |
|-------|------|-------------|
| **Arena floor** | 512x512 | Concrete with grid lines |
| **Arena wall** | 256x256 | Metal/concrete barrier |
| **Vehicle textures** | 256x256 | Team colors with weathering |
| **Damage overlay** | 256x256 | Scratches, dents, burns |
| **Effects** | 128x128 | Fire, smoke, muzzle flash |

### Texture Guidelines

```
Format: PNG (RGBA)
Size: Power of 2 (64, 128, 256, 512, 1024)
Style: Stylized, not photo-realistic
Mip-maps: Generated at load time
```

---

## Shaders

### Current Shaders

| Shader | Purpose |
|--------|---------|
| `grid.vert/frag` | Arena floor with procedural grid |
| `solid_color.vert/frag` | Debug and placeholder shapes |
| `vehicle.vert/frag` | Lit vehicles with team colors |

### Planned Shaders

| Shader | Purpose |
|--------|---------|
| `fire.vert/frag` | Animated fire effect |
| `particle.vert/frag` | Particle systems |
| `ui.vert/frag` | 2D UI elements |
| `ghost.vert/frag` | Movement preview paths |

---

## Audio (Planned)

### Sound Effects

| Sound | Description |
|-------|-------------|
| **Engine idle** | Vehicle humming |
| **Engine rev** | Acceleration |
| **Collision** | Metal crash |
| **Tire screech** | Hard turning |
| **Weapons** | Machine gun, rockets |
| **Explosion** | Vehicle destruction |

### Audio Specifications

```
Format: WAV (16-bit, 44.1kHz) or OGG Vorbis
Mono for 3D sounds, Stereo for music/UI
Normalize to -3dB peak
```

---

## Scripts

### Lua Reflex Scripts

Located in `assets/scripts/`:

| Script | Purpose |
|--------|---------|
| `master.lua` | Main script coordinator |
| `modules/tcs.lua` | Traction control system |
| `modules/maneuver.lua` | Turn-based maneuver execution |

Scripts are hot-reloadable with F5 during development.

---

## Current Resources

### Kenney Car Kit (CC0)

- **Location:** `assets/models/vehicles/kenney-car-kit/`
- **License:** CC0 (Public Domain) - Free for commercial use
- **Format:** OBJ, FBX, GLB
- **Website:** https://kenney.nl/assets/car-kit

**Available Models:**

| Vehicle Type | Models |
|--------------|--------|
| Cars | sedan, sedan-sports, hatchback-sports, taxi, police |
| Racing | race, race-future |
| Trucks | truck, truck-flat, delivery |
| SUVs | suv, suv-luxury |
| Utility | ambulance, firetruck, van |

**Bonus Assets:**
- Multiple wheel variants
- Debris pieces (doors, tires, plates, spoilers)
- Traffic props (cones, boxes)

---

## Asset Sources

### Free/CC Resources

| Source | Type | License |
|--------|------|---------|
| Kenney.nl | Game assets | CC0 |
| OpenGameArt.org | Various | Check individual |
| Quaternius | Low-poly 3D | CC0 |
| Freesound.org | Sound effects | Various |

### Paid Resources (Future)

| Source | Type |
|--------|------|
| Sketchfab | Stylized vehicles |
| CGTrader | Game-ready assets |
| TurboSquid | Professional models |

---

## Art Direction

### Vehicle Style

- Chunky, solid, weighted
- Visible weapons (not hidden)
- Bright team colors
- Wear and tear details
- Stylized, not realistic

### Arena Style

- Industrial, concrete
- Grid visible but integrated
- Stains, cracks, history
- Barriers feel heavy
- Harsh overhead lighting

---

## Open Questions

- [ ] Custom vehicle models: Commission or purchase?
- [ ] Audio: In-house or licensed?
- [ ] Style guide for consistent look?