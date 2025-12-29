/*
 * Car Wars Maneuver System
 *
 * Implements the "autopilot" that executes maneuvers by steering the vehicle
 * through physics to reach the target position defined by Car Wars rules.
 *
 * Flow:
 * 1. Player requests maneuver while paused (or in turn-based mode)
 * 2. System validates speed requirements
 * 3. System calculates target position/heading from Car Wars rules
 * 4. Physics unpauses, autopilot takes control
 * 5. Autopilot steers vehicle toward target
 * 6. When target reached (or timeout), autopilot ends
 * 7. Control returns to player
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

// Autopilot controller state
typedef struct {
    AutopilotState state;
    ManeuverRequest request;

    // Target (calculated from Car Wars rules at start)
    Vec3 target_position;
    float target_heading;         // Radians
    float target_speed_ms;        // Target speed at end (usually same as start)

    // Start state (captured when maneuver begins)
    Vec3 start_position;
    float start_heading;
    float start_speed_ms;

    // Timing
    float elapsed;                // Seconds since maneuver started
    float timeout;                // Max time allowed (safety)
    float expected_duration;      // How long maneuver should take

    // Steering profile (tuned per maneuver type)
    float steer_intensity;        // -1 to 1, current steering input
    float steer_phase;            // Where we are in the steering sequence

    // Completion detection
    float position_tolerance;     // How close is "close enough" (meters)
    float heading_tolerance;      // How close heading needs to be (radians)

    // Debug info
    float lateral_displacement;   // Current lateral offset from start
    float forward_displacement;   // Current forward offset from start

    // Pending corrections (applied by physics system after each phase)
    bool lateral_nudge_pending;   // True if lateral correction needed
    float lateral_nudge_amount;   // Meters to nudge (positive = right)
    bool heading_nudge_pending;   // True if heading correction needed
    float heading_nudge_target;   // Target heading in radians
    bool phase1_complete;         // Track if Phase 1 nudge already applied

    // Smooth heading correction animation
    float correction_elapsed;     // Time spent in correction phase
    float correction_duration;    // Total time for correction animation
    float correction_start_heading; // Heading when correction started
} ManeuverAutopilot;

// Validate if a maneuver can be performed at current speed
// Returns true if allowed, false if not (with reason in out_reason if provided)
bool maneuver_validate(ManeuverType type, float speed_ms, const char** out_reason);

// Calculate the difficulty (D value) for a maneuver
int maneuver_get_difficulty(ManeuverType type, ManeuverDirection dir, int param);

// Start a maneuver - calculates target and activates autopilot
// Returns false if maneuver not allowed at current speed
bool maneuver_start(ManeuverAutopilot* ap,
                    const ManeuverRequest* request,
                    Vec3 current_pos,
                    float current_heading,
                    float current_speed_ms);

// Update autopilot - called each physics frame
// Returns the steering input to apply (-1 to 1)
// Sets *out_complete to true when maneuver is done
float maneuver_update(ManeuverAutopilot* ap,
                      Vec3 current_pos,
                      float current_heading,
                      float current_speed_ms,
                      float dt,
                      bool* out_complete);

// Cancel a maneuver in progress
void maneuver_cancel(ManeuverAutopilot* ap);

// Check if autopilot is active
bool maneuver_is_active(const ManeuverAutopilot* ap);

// Get maneuver name for display
const char* maneuver_get_name(ManeuverType type);

// Get maneuver status string for display
const char* maneuver_get_status(const ManeuverAutopilot* ap);

#ifdef __cplusplus
}
#endif

#endif // MANEUVER_H
