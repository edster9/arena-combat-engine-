/*
 * Car Wars Maneuver System - Kinematic Path Animation
 *
 * Uses linear interpolation (lerp) between start and end poses.
 * Vehicle is in kinematic mode during maneuver execution.
 */

#include "maneuver.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// Speed conversion
#define MS_TO_MPH 2.237f
#define MPH_TO_MS 0.447f
#define PI 3.14159265359f

// Speed requirements (in m/s)
static const float PIVOT_SPEED_EXACT = 5.0f * MPH_TO_MS;
static const float PIVOT_SPEED_TOLERANCE = 1.0f * MPH_TO_MS;
static const float T_STOP_MIN_SPEED = 10.0f * MPH_TO_MS;
static const float BOOTLEGGER_MIN_SPEED = 20.0f * MPH_TO_MS;
static const float BOOTLEGGER_MAX_SPEED = 35.0f * MPH_TO_MS;

const char* maneuver_get_name(ManeuverType type) {
    switch (type) {
        case MANEUVER_NONE:           return "None";
        case MANEUVER_DRIFT:          return "Drift";
        case MANEUVER_STEEP_DRIFT:    return "Steep Drift";
        case MANEUVER_BEND:           return "Bend";
        case MANEUVER_SWERVE:         return "Swerve";
        case MANEUVER_CONTROLLED_SKID: return "Controlled Skid";
        case MANEUVER_PIVOT:          return "Pivot";
        case MANEUVER_T_STOP:         return "T-Stop";
        case MANEUVER_BOOTLEGGER:     return "Bootlegger";
        default:                      return "Unknown";
    }
}

const char* maneuver_get_status(const ManeuverAutopilot* ap) {
    switch (ap->state) {
        case AUTOPILOT_IDLE:       return "Idle";
        case AUTOPILOT_STARTING:   return "Starting";
        case AUTOPILOT_EXECUTING:  return "Executing";
        case AUTOPILOT_COMPLETING: return "Completing";
        case AUTOPILOT_CORRECTING: return "Correcting";
        case AUTOPILOT_FINISHED:   return "Finished";
        case AUTOPILOT_FAILED:     return "Failed";
        default:                   return "Unknown";
    }
}

bool maneuver_validate(ManeuverType type, float speed_ms, const char** out_reason) {
    switch (type) {
        case MANEUVER_PIVOT:
            if (fabsf(speed_ms - PIVOT_SPEED_EXACT) > PIVOT_SPEED_TOLERANCE) {
                if (out_reason) *out_reason = "Pivot requires exactly 5 mph";
                return false;
            }
            break;

        case MANEUVER_T_STOP:
            if (speed_ms < T_STOP_MIN_SPEED) {
                if (out_reason) *out_reason = "T-Stop requires at least 10 mph";
                return false;
            }
            break;

        case MANEUVER_BOOTLEGGER:
            if (speed_ms < BOOTLEGGER_MIN_SPEED) {
                if (out_reason) *out_reason = "Bootlegger requires at least 20 mph";
                return false;
            }
            if (speed_ms > BOOTLEGGER_MAX_SPEED) {
                if (out_reason) *out_reason = "Bootlegger max speed is 35 mph";
                return false;
            }
            break;

        case MANEUVER_DRIFT:
        case MANEUVER_STEEP_DRIFT:
        case MANEUVER_BEND:
        case MANEUVER_SWERVE:
        case MANEUVER_CONTROLLED_SKID:
            // These can be performed at any speed
            break;

        default:
            if (out_reason) *out_reason = "Unknown maneuver type";
            return false;
    }

    if (out_reason) *out_reason = NULL;
    return true;
}

int maneuver_get_difficulty(ManeuverType type, ManeuverDirection dir, int param) {
    (void)dir;  // Unused for now

    switch (type) {
        case MANEUVER_DRIFT:
            return 1;  // D1

        case MANEUVER_STEEP_DRIFT:
            return 3;  // D3

        case MANEUVER_BEND:
            // D value based on angle: 15°=D1, 30°=D2, 45°=D3, 60°=D4, 75°=D5, 90°=D6
            return (param + 14) / 15;

        case MANEUVER_PIVOT:
            return 0;  // D0

        case MANEUVER_T_STOP:
            return param;  // D1 per 10 mph

        case MANEUVER_BOOTLEGGER:
            return 7;  // D7

        case MANEUVER_SWERVE:
            return param + 1;  // Drift + Bend, +1 difficulty

        case MANEUVER_CONTROLLED_SKID:
            return param;  // 1-4 quarter inches

        default:
            return 0;
    }
}

// Simple linear interpolation
static float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// Ease in-out curve for smoother motion
static float ease_in_out(float t) {
    // Smoothstep: 3t² - 2t³
    return t * t * (3.0f - 2.0f * t);
}

// Shortest angle lerp (handles wrapping)
static float angle_lerp(float a, float b, float t) {
    float diff = b - a;
    // Normalize to -PI to PI
    while (diff > PI) diff -= 2.0f * PI;
    while (diff < -PI) diff += 2.0f * PI;
    return a + diff * t;
}

// Calculate target position and heading for a maneuver
static void calculate_target(ManeuverAutopilot* ap) {
    float dir = (float)ap->request.direction;  // -1 or 1

    // Calculate lateral offset in world coordinates
    float sin_h = sinf(ap->start_heading);
    float cos_h = cosf(ap->start_heading);

    // Right vector (perpendicular to heading)
    float right_x = cos_h;
    float right_z = -sin_h;

    // Forward vector
    float fwd_x = sin_h;
    float fwd_z = cos_h;

    // Calculate lateral displacement based on maneuver type
    float lateral = 0.0f;
    float heading_change = 0.0f;

    switch (ap->request.type) {
        case MANEUVER_DRIFT:
            lateral = CW_QUARTER_INCH * dir;
            heading_change = 0.0f;  // Maintain heading
            break;

        case MANEUVER_STEEP_DRIFT:
            lateral = CW_HALF_INCH * dir;
            heading_change = 0.0f;
            break;

        case MANEUVER_BEND:
            heading_change = (float)ap->request.bend_angle * (PI / 180.0f) * dir;
            // TODO: Calculate arc path
            lateral = 0.0f;
            break;

        case MANEUVER_BOOTLEGGER:
            heading_change = PI * dir;  // 180° turn
            lateral = 0.0f;
            break;

        default:
            lateral = 0.0f;
            heading_change = 0.0f;
            break;
    }

    // Forward distance during maneuver (based on speed and duration)
    float fwd_dist = ap->start_speed_ms * ap->duration;

    // Calculate target position
    ap->target_position.x = ap->start_position.x + right_x * lateral + fwd_x * fwd_dist;
    ap->target_position.y = ap->start_position.y;  // Keep Y constant
    ap->target_position.z = ap->start_position.z + right_z * lateral + fwd_z * fwd_dist;

    // Calculate target heading
    ap->target_heading = ap->start_heading + heading_change;
}

bool maneuver_start(ManeuverAutopilot* ap,
                    const ManeuverRequest* request,
                    Vec3 current_pos,
                    float current_heading,
                    float current_speed_ms) {
    // Validate
    const char* reason = NULL;
    if (!maneuver_validate(request->type, current_speed_ms, &reason)) {
        printf("[Maneuver] Cannot start %s: %s\n",
               maneuver_get_name(request->type), reason);
        return false;
    }

    // Initialize autopilot state
    memset(ap, 0, sizeof(*ap));
    ap->state = AUTOPILOT_EXECUTING;
    ap->request = *request;

    // Capture start state
    ap->start_position = current_pos;
    ap->start_heading = current_heading;
    ap->start_speed_ms = current_speed_ms;

    // Calculate duration based on forward distance covered
    // At 20 mph (~9 m/s), covering ~10m takes ~1.1s
    float forward_dist = 10.0f;  // meters to travel
    ap->duration = forward_dist / fmaxf(current_speed_ms, 2.0f);
    ap->elapsed = 0.0f;
    ap->progress = 0.0f;

    // Calculate target
    calculate_target(ap);

    // Initialize current pose to start
    ap->current_pose.position = current_pos;
    ap->current_pose.heading = current_heading;

    // Get lateral displacement for display
    float lateral_target = 0.0f;
    switch (request->type) {
        case MANEUVER_DRIFT: lateral_target = CW_QUARTER_INCH; break;
        case MANEUVER_STEEP_DRIFT: lateral_target = CW_HALF_INCH; break;
        default: lateral_target = 0.0f; break;
    }

    // Debug output
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║            KINEMATIC MANEUVER START                          ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║ Type: %-20s Direction: %-10s          ║\n",
           maneuver_get_name(request->type),
           request->direction == MANEUVER_LEFT ? "LEFT" : "RIGHT");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║ START:  pos=(%.1f, %.1f, %.1f) heading=%.1f°               \n",
           current_pos.x, current_pos.y, current_pos.z,
           current_heading * 180.0f / PI);
    printf("║ TARGET: pos=(%.1f, %.1f, %.1f) heading=%.1f°               \n",
           ap->target_position.x, ap->target_position.y, ap->target_position.z,
           ap->target_heading * 180.0f / PI);
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║ Lateral: %.2fm %s | Duration: %.2fs | Speed: %.1f mph       \n",
           lateral_target,
           request->direction == MANEUVER_LEFT ? "LEFT" : "RIGHT",
           ap->duration,
           current_speed_ms * MS_TO_MPH);
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    fflush(stdout);

    return true;
}

ManeuverPose maneuver_update(ManeuverAutopilot* ap,
                             float dt,
                             bool* out_complete) {
    *out_complete = false;

    // Return current pose if not active
    if (ap->state != AUTOPILOT_EXECUTING) {
        return ap->current_pose;
    }

    // Update timing
    ap->elapsed += dt;
    ap->progress = ap->elapsed / ap->duration;

    // Clamp progress
    if (ap->progress >= 1.0f) {
        ap->progress = 1.0f;
    }

    // Apply easing curve for smoother motion
    float t = ease_in_out(ap->progress);

    // Interpolate position and heading
    ap->current_pose.position = vec3_lerp(ap->start_position, ap->target_position, t);
    ap->current_pose.heading = angle_lerp(ap->start_heading, ap->target_heading, t);

    // Add realistic heading wobble for drift maneuvers
    // Car's nose dips into the drift direction, then straightens out
    if (ap->request.type == MANEUVER_DRIFT || ap->request.type == MANEUVER_STEEP_DRIFT) {
        // Sine wave peaks at 50% progress, returns to 0 at 100%
        // This creates a smooth "nose-in, straighten-out" motion
        float wobble_amount = 8.0f * (PI / 180.0f);  // 8 degrees max
        float wobble = sinf(ap->progress * PI) * wobble_amount;
        // Direction: negative wobble for left drift, positive for right
        wobble *= (float)ap->request.direction;  // -1 for left, +1 for right
        ap->current_pose.heading += wobble;
    }

    // Calculate displacement for debug
    float dx = ap->current_pose.position.x - ap->start_position.x;
    float dz = ap->current_pose.position.z - ap->start_position.z;
    float sin_h = sinf(ap->start_heading);
    float cos_h = cosf(ap->start_heading);
    ap->forward_displacement = dx * sin_h + dz * cos_h;
    ap->lateral_displacement = dx * cos_h - dz * sin_h;

    // Debug output at 25% intervals
    static int last_quarter = -1;
    int quarter = (int)(ap->progress * 4.0f);
    if (quarter != last_quarter && quarter <= 4) {
        printf("[Kinematic] %.0f%% | pos=(%.1f, %.1f) | heading=%.1f° | lat=%.2fm\n",
               ap->progress * 100.0f,
               ap->current_pose.position.x, ap->current_pose.position.z,
               ap->current_pose.heading * 180.0f / PI,
               ap->lateral_displacement);
        fflush(stdout);
        last_quarter = quarter;
    }

    // Check completion
    if (ap->progress >= 1.0f) {
        ap->state = AUTOPILOT_FINISHED;
        *out_complete = true;
        last_quarter = -1;  // Reset for next maneuver

        // Final position exactly at target
        ap->current_pose.position = ap->target_position;
        ap->current_pose.heading = ap->target_heading;

        printf("\n");
        printf("╔══════════════════════════════════════════════════════════════╗\n");
        printf("║            KINEMATIC MANEUVER COMPLETE                       ║\n");
        printf("╠══════════════════════════════════════════════════════════════╣\n");
        printf("║ Duration: %.2fs                                             \n",
               ap->elapsed);
        printf("║ Final: pos=(%.1f, %.1f, %.1f) heading=%.1f°                \n",
               ap->current_pose.position.x, ap->current_pose.position.y,
               ap->current_pose.position.z,
               ap->current_pose.heading * 180.0f / PI);
        printf("║ Lateral: %.2fm | Forward: %.2fm                            \n",
               ap->lateral_displacement, ap->forward_displacement);
        printf("╚══════════════════════════════════════════════════════════════╝\n");
        fflush(stdout);
    }

    return ap->current_pose;
}

void maneuver_cancel(ManeuverAutopilot* ap) {
    if (ap->state != AUTOPILOT_IDLE) {
        printf("[Maneuver] Cancelled after %.2fs\n", ap->elapsed);
    }
    ap->state = AUTOPILOT_IDLE;
}

bool maneuver_is_active(const ManeuverAutopilot* ap) {
    return ap->state == AUTOPILOT_EXECUTING;
}

Vec3 maneuver_get_exit_velocity(const ManeuverAutopilot* ap) {
    // Calculate velocity in the direction of target heading at original speed
    float sin_h = sinf(ap->target_heading);
    float cos_h = cosf(ap->target_heading);

    Vec3 velocity;
    velocity.x = sin_h * ap->start_speed_ms;
    velocity.y = 0.0f;
    velocity.z = cos_h * ap->start_speed_ms;

    return velocity;
}
