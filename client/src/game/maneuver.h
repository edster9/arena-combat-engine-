/*
 * Car Wars Maneuver System - Kinematic Path Animation
 *
 * Executes maneuvers using kinematic interpolation along a calculated path.
 * Vehicle is switched to kinematic mode during maneuver, then back to dynamic.
 *
 * Flow:
 * 1. Player requests maneuver while paused
 * 2. System validates speed requirements
 * 3. System calculates target position/heading from Car Wars rules
 * 4. Vehicle switches to KINEMATIC mode
 * 5. Each frame: interpolate position/heading along path, use MoveKinematic
 * 6. When path complete: switch back to DYNAMIC, set velocity to match
 * 7. Control returns to player
 *
 * Interruption: If collision/hazard detected mid-maneuver, immediately
 * switch to dynamic mode and let physics handle the chaos.
 */

#ifndef MANEUVER_H
#define MANEUVER_H

#include <stdbool.h>
#include "../math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// Car Wars scale: 1" = 15 feet = 4.572 meters
#define CW_INCH_TO_METERS 4.572f
#define CW_QUARTER_INCH   (CW_INCH_TO_METERS * 0.25f)  // 1.143m
#define CW_HALF_INCH      (CW_INCH_TO_METERS * 0.5f)   // 2.286m
#define CW_THREE_QUARTER  (CW_INCH_TO_METERS * 0.75f)  // 3.429m

// Maneuver types
typedef enum {
    MANEUVER_NONE = 0,

    // Basic maneuvers (Phase 2)
    MANEUVER_DRIFT,           // D1: 1/4" lateral, keep heading
    MANEUVER_STEEP_DRIFT,     // D3: 1/2" lateral, keep heading
    MANEUVER_BEND,            // D1-D6: turn with heading change

    // Advanced maneuvers (Phase 3+)
    MANEUVER_SWERVE,          // Drift + opposite bend
    MANEUVER_CONTROLLED_SKID, // D+1 to D+4: powerslide

    // Special maneuvers (Phase 4)
    MANEUVER_PIVOT,           // D0: 5 mph only, pivot around rear corner
    MANEUVER_T_STOP,          // D1/10mph: emergency 90° brake
    MANEUVER_BOOTLEGGER,      // D7: 20-35 mph, J-turn 180°

    MANEUVER_COUNT
} ManeuverType;

// Direction for lateral maneuvers
typedef enum {
    MANEUVER_LEFT = -1,
    MANEUVER_RIGHT = 1
} ManeuverDirection;

// Autopilot state
typedef enum {
    AUTOPILOT_IDLE,           // No maneuver active
    AUTOPILOT_STARTING,       // Just started, initializing
    AUTOPILOT_EXECUTING,      // Steering toward target
    AUTOPILOT_COMPLETING,     // Near target, settling
    AUTOPILOT_CORRECTING,     // Smoothly correcting heading (animation phase)
    AUTOPILOT_FINISHED,       // Done, returning control
    AUTOPILOT_FAILED          // Timeout or physics failure
} AutopilotState;

// Maneuver request (what the player wants to do)
typedef struct {
    ManeuverType type;
    ManeuverDirection direction;  // LEFT or RIGHT
    int bend_angle;               // For BEND: degrees (15, 30, 45, 60, 75, 90)
    int skid_distance;            // For CONTROLLED_SKID: 1-4 (quarter inches)
} ManeuverRequest;

// Pose for kinematic interpolation
typedef struct {
    Vec3 position;
    float heading;      // Radians
} ManeuverPose;

// Autopilot controller state
typedef struct {
    AutopilotState state;
    ManeuverRequest request;

    // Start state (captured when maneuver begins)
    Vec3 start_position;
    float start_heading;
    float start_speed_ms;

    // Target state (calculated from Car Wars rules)
    Vec3 target_position;
    float target_heading;         // Radians

    // Timing
    float elapsed;                // Seconds since maneuver started
    float duration;               // Total maneuver duration
    float progress;               // 0.0 to 1.0 normalized time

    // Current interpolated pose (updated each frame)
    ManeuverPose current_pose;

    // Debug info
    float lateral_displacement;   // Current lateral offset from start
    float forward_displacement;   // Current forward offset from start
} ManeuverAutopilot;

// Validate if a maneuver can be performed at current speed
// Returns true if allowed, false if not (with reason in out_reason if provided)
bool maneuver_validate(ManeuverType type, float speed_ms, const char** out_reason);

// Calculate the difficulty (D value) for a maneuver
int maneuver_get_difficulty(ManeuverType type, ManeuverDirection dir, int param);

// Start a maneuver - calculates path and activates kinematic animation
// Returns false if maneuver not allowed at current speed
bool maneuver_start(ManeuverAutopilot* ap,
                    const ManeuverRequest* request,
                    Vec3 current_pos,
                    float current_heading,
                    float current_speed_ms);

// Update autopilot - called each physics frame
// Calculates interpolated pose for this frame
// Sets *out_complete to true when maneuver is done
// Returns the current pose to move the vehicle to
ManeuverPose maneuver_update(ManeuverAutopilot* ap,
                             float dt,
                             bool* out_complete);

// Cancel a maneuver in progress
void maneuver_cancel(ManeuverAutopilot* ap);

// Check if autopilot is active (vehicle should be kinematic)
bool maneuver_is_active(const ManeuverAutopilot* ap);

// Get the exit velocity (direction and speed for when switching back to dynamic)
Vec3 maneuver_get_exit_velocity(const ManeuverAutopilot* ap);

// Get maneuver name for display
const char* maneuver_get_name(ManeuverType type);

// Get maneuver status string for display
const char* maneuver_get_status(const ManeuverAutopilot* ap);

#ifdef __cplusplus
}
#endif

#endif // MANEUVER_H
