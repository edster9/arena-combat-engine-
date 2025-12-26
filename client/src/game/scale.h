#ifndef SCALE_H
#define SCALE_H

/*
 * CAR WARS SCALE REFERENCE
 * ========================
 *
 * Car Wars Tabletop Scale:
 *   - 1 tabletop inch = 15 real feet (scale 1:180)
 *   - Car counter = 1 inch long (represents ~15 foot car)
 *   - Grid squares = 1/4 inch (3.75 feet)
 *   - Major grid lines = 1 inch apart
 *
 * Game Unit Mapping:
 *   We use: 1 GAME UNIT = 1 CAR WARS INCH = 15 FEET
 *
 *   This means:
 *   - A car is ~1 unit long (the "game size" of a car counter)
 *   - Grid cells = 1 unit (1 inch on tabletop)
 *   - Movement of "1 inch" in rules = 1 unit in game
 *   - At 50 mph, vehicle moves 5 units per turn (5 inches)
 *
 * Visual Scale (for 3D rendering):
 *   To make things look good, we render cars larger than 1 unit.
 *   The VISUAL_SCALE multiplies rendered size without affecting game logic.
 */

// === GAME LOGIC SCALE (matches Car Wars rules) ===
// 1 unit = 1 Car Wars inch = 15 feet

#define CW_INCH_TO_FEET     15.0f           // 1 tabletop inch = 15 real feet
#define CW_FEET_TO_INCH     (1.0f/15.0f)    // Convert feet to tabletop inches

// Vehicle game dimensions (in Car Wars inches / game units)
#define CW_CAR_LENGTH       1.0f            // Car counter is 1 inch long
#define CW_CAR_WIDTH        0.5f            // Car counter is ~0.5 inch wide
#define CW_CYCLE_LENGTH     0.5f            // Cycle counter is 0.5 inch long

// Grid dimensions
#define CW_GRID_CELL        1.0f            // Major grid = 1 inch
#define CW_GRID_MINOR       0.25f           // Minor grid = 1/4 inch

// Arena sizes (in game units = inches)
#define CW_ARENA_SMALL      40.0f           // 40x40 inch arena
#define CW_ARENA_MEDIUM     60.0f           // 60x60 inch arena
#define CW_ARENA_LARGE      100.0f          // 100x100 inch arena

// === VISUAL RENDERING SCALE ===
// To make cars visible and good-looking in 3D, we render them larger
// This ONLY affects rendering, not game logic

#define VISUAL_SCALE        4.5f            // Render cars 4.5x larger for visibility

// Visual car dimensions (what you actually see rendered)
#define VISUAL_CAR_LENGTH   (CW_CAR_LENGTH * VISUAL_SCALE)   // 4.5 units
#define VISUAL_CAR_WIDTH    (CW_CAR_WIDTH * VISUAL_SCALE)    // 2.25 units
#define VISUAL_CAR_HEIGHT   (0.3f * VISUAL_SCALE)            // 1.35 units

// Visual grid (render grid at game scale, not visual scale)
#define VISUAL_GRID_CELL    CW_GRID_CELL    // Grid stays at 1 unit

// === MOVEMENT CONVERSION ===
// Speed in mph -> distance in game units per turn

// At speed X mph, vehicle moves X/10 game units per turn
// (because 10 mph = 1 inch/turn in Car Wars)
#define MPH_TO_UNITS_PER_TURN(mph)  ((mph) / 10.0f)
#define UNITS_PER_TURN_TO_MPH(u)    ((u) * 10.0f)

// Per-phase movement (5 phases per turn)
// At 50 mph: 5 units/turn = 1 unit/phase
#define MPH_TO_UNITS_PER_PHASE(mph) (MPH_TO_UNITS_PER_TURN(mph) / 5.0f)

// === REAL WORLD REFERENCE ===
// For context, here's what things represent in "real" feet:
//
// 1 game unit = 15 feet
// Car (1 unit) = 15 feet long (realistic for a sedan)
// 50 mph movement (5 units/turn) = 75 feet/second = 51 mph (close enough!)
// Arena 60x60 units = 900x900 feet = about 3 football fields

#endif // SCALE_H
