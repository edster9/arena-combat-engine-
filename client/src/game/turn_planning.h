#ifndef TURN_PLANNING_H
#define TURN_PLANNING_H

#include <stdbool.h>
#include "../math/vec3.h"

/*
 * TURN PLANNING DATA STRUCTURES
 * =============================
 *
 * These structures represent the player's input for a single turn.
 * The player plans their entire turn before execution begins.
 *
 * Turn Flow:
 *   1. PLANNING PHASE - Player fills in TurnPlan
 *   2. EXECUTION PHASE - System executes all plans simultaneously
 *   3. RESOLUTION PHASE - Damage, crashes, etc. resolved
 *   4. END PHASE - Handling recovery, turn counter advances
 */

// Maximum phases per turn (Car Wars standard)
#define MAX_PHASES 5

// === MANEUVER TYPES ===
typedef enum {
    MANEUVER_STRAIGHT = 0,      // No turn, go straight
    MANEUVER_DRIFT,             // Slight sideways movement, same heading (D1)
    MANEUVER_STEEP_DRIFT,       // More sideways movement (D3)
    MANEUVER_BEND_15,           // 15 degree turn (D1)
    MANEUVER_BEND_30,           // 30 degree turn (D2)
    MANEUVER_BEND_45,           // 45 degree turn (D3)
    MANEUVER_BEND_60,           // 60 degree turn (D4)
    MANEUVER_BEND_75,           // 75 degree turn (D5)
    MANEUVER_BEND_90,           // 90 degree turn (D6)
    MANEUVER_SWERVE,            // Drift + opposite bend
    MANEUVER_CONTROLLED_SKID,   // Bend/swerve + skid
    MANEUVER_BOOTLEGGER,        // J-turn (20-35 mph only)
    MANEUVER_T_STOP,            // Emergency stop (rotate 90° + skid)
    MANEUVER_PIVOT,             // Low-speed pivot (5 mph only)
    MANEUVER_COUNT
} ManeuverType;

// Direction for maneuvers
typedef enum {
    DIR_NONE = 0,
    DIR_LEFT,
    DIR_RIGHT
} ManeuverDirection;

// === SPEED CHANGE ===
typedef enum {
    SPEED_MAINTAIN = 0,         // No change
    SPEED_ACCELERATE,           // Speed up
    SPEED_DECELERATE,           // Slow down (safe braking)
    SPEED_HARD_BRAKE            // Emergency braking (risky)
} SpeedChangeType;

// === WEAPON TARGET ===
typedef enum {
    TARGET_NONE = 0,            // Don't fire
    TARGET_VEHICLE,             // Target specific vehicle
    TARGET_POINT,               // Target a point on the map
    TARGET_AUTOFIRE             // Fire straight ahead (no aiming)
} WeaponTargetType;

// Weapon targeting info
typedef struct {
    WeaponTargetType type;
    int target_entity_id;       // If TARGET_VEHICLE
    Vec3 target_point;          // If TARGET_POINT
    bool fire_this_turn;        // Whether to fire at all
} WeaponTarget;

// === PHASE PLAN ===
// What happens in a single phase (1/5 of a turn)
typedef struct {
    // Movement for this phase
    float move_distance;        // Distance to move (from movement chart)
    bool can_maneuver;          // True if full inch move, false if half-move

    // Chosen maneuver (if can_maneuver is true)
    ManeuverType maneuver;
    ManeuverDirection direction;

    // Controlled skid parameters (if MANEUVER_CONTROLLED_SKID)
    float skid_distance;        // 0.25, 0.5, 0.75, or 1.0 inches

    // Computed values (filled in after planning)
    int difficulty;             // Maneuver difficulty (D value)
    Vec3 end_position;          // Where vehicle ends up
    float end_rotation;         // Vehicle facing after maneuver
} PhasePlan;

// === TURN PLAN ===
// Complete plan for one vehicle for one turn
typedef struct {
    int entity_id;              // Which vehicle this plan is for

    // Speed change (once per turn)
    SpeedChangeType speed_change;
    int speed_delta;            // +/- mph (within vehicle's limits)
    int planned_speed;          // Speed after change

    // Phase plans (up to 5)
    int active_phases;          // How many phases this vehicle moves
    PhasePlan phases[MAX_PHASES];

    // Weapon targets (up to 4 weapons)
    #define MAX_WEAPONS 4
    WeaponTarget weapons[MAX_WEAPONS];

    // Computed totals
    int total_difficulty;       // Sum of all maneuver difficulties
    int handling_cost;          // How much handling status will drop
    bool requires_control_roll; // Will need to roll for control?

    // Preview path (computed from phases)
    #define MAX_PATH_POINTS 32
    Vec3 path_points[MAX_PATH_POINTS];
    int path_point_count;
} TurnPlan;

// === PLANNING UI STATE ===
typedef enum {
    PLAN_STEP_SPEED = 0,        // Choosing speed change
    PLAN_STEP_MANEUVERS,        // Planning maneuvers for each phase
    PLAN_STEP_WEAPONS,          // Selecting weapon targets
    PLAN_STEP_CONFIRM,          // Review and confirm
    PLAN_STEP_DONE              // Plan locked in
} PlanningStep;

typedef struct {
    PlanningStep current_step;
    int selected_vehicle_id;    // Which vehicle we're planning for
    int current_phase;          // Which phase we're editing (0-4)
    int current_weapon;         // Which weapon we're targeting

    TurnPlan plan;              // The plan being built

    // UI helpers
    bool show_path_preview;     // Show ghost line of planned path
    bool show_firing_arcs;      // Show weapon arcs
    bool show_difficulty_warning; // Highlight risky maneuvers
} PlanningUIState;

// === FUNCTIONS ===

// Initialize a turn plan for a vehicle
void turn_plan_init(TurnPlan* plan, int entity_id, int current_speed, int max_accel, int max_decel);

// Get movement chart data for a given speed
void turn_plan_get_phases(TurnPlan* plan, int speed);

// Set maneuver for a specific phase
void turn_plan_set_maneuver(TurnPlan* plan, int phase, ManeuverType maneuver, ManeuverDirection dir);

// Calculate path preview from current plan
void turn_plan_calculate_path(TurnPlan* plan, Vec3 start_pos, float start_rotation);

// Get difficulty for a maneuver type
int maneuver_get_difficulty(ManeuverType type);

// Get maneuver name as string (for UI)
const char* maneuver_get_name(ManeuverType type);

// === PLANNING UI ===

// Initialize planning UI state
void planning_ui_init(PlanningUIState* ui);

// Start planning for a vehicle
void planning_ui_start(PlanningUIState* ui, int entity_id, int current_speed, int max_accel, int max_decel);

// Handle input during planning
void planning_ui_handle_input(PlanningUIState* ui, int key, bool mouse_click, Vec3 mouse_world_pos);

// Advance to next step
void planning_ui_next_step(PlanningUIState* ui);

// Go back to previous step
void planning_ui_prev_step(PlanningUIState* ui);

// Check if planning is complete
bool planning_ui_is_complete(PlanningUIState* ui);

// Get the completed plan
TurnPlan* planning_ui_get_plan(PlanningUIState* ui);

#endif // TURN_PLANNING_H
