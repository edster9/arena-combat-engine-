/*
 * Car Wars Maneuver System Implementation
 */

#include "maneuver.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// Speed conversion
#define MS_TO_MPH 2.237f
#define MPH_TO_MS 0.447f

// Tuning parameters for autopilot steering
// These will need empirical adjustment like the K acceleration factors
typedef struct {
    float steer_duration;     // How long to steer (seconds)
    float steer_intensity;    // How hard to steer (0-1)
    float return_duration;    // How long to counter-steer to straighten
    float return_intensity;   // Counter-steer intensity
} SteeringProfile;

// Steering profiles per maneuver type (tuned empirically)
// Format: { steer_duration, steer_intensity, return_duration, return_intensity }
// Test 1 @ 20mph: 0.5/0.3 → 57% accuracy, -7.9° heading drift
// Test 2 @ 20mph: 0.85/0.6 → 57% (early complete)
// Test 3 @ 20mph: 0.85/0.6 + tight tol → 81% accuracy, -10.7° heading
// Test 4 @ 20mph: 1.0/0.9, 50%/90% → 81% accuracy, -10.9° heading
// Test 5 @ 20mph: 1.0/0.9, 35%/95% → 31% accuracy, +0.9° heading (overcorrected!)
// Test 6 @ 20mph: 1.0/0.7, 45%/90% → ?
static const SteeringProfile s_drift_profile = { 0.3f, 1.0f, 0.2f, 0.7f };
static const SteeringProfile s_steep_drift_profile = { 0.4f, 1.0f, 0.25f, 0.8f };

// Speed requirements (in m/s)
static const float PIVOT_SPEED_EXACT = 5.0f * MPH_TO_MS;    // 5 mph exactly
static const float PIVOT_SPEED_TOLERANCE = 1.0f * MPH_TO_MS; // +/- 1 mph
static const float T_STOP_MIN_SPEED = 10.0f * MPH_TO_MS;     // 10 mph minimum
static const float BOOTLEGGER_MIN_SPEED = 20.0f * MPH_TO_MS; // 20 mph
static const float BOOTLEGGER_MAX_SPEED = 35.0f * MPH_TO_MS; // 35 mph

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
    float speed_mph = speed_ms * MS_TO_MPH;

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
            // These can be performed at any speed (handling penalties apply at high speed)
            break;

        default:
            if (out_reason) *out_reason = "Unknown maneuver type";
            return false;
    }

    if (out_reason) *out_reason = NULL;
    return true;
}

int maneuver_get_difficulty(ManeuverType type, ManeuverDirection dir, int param) {
    switch (type) {
        case MANEUVER_DRIFT:
            return 1;  // D1

        case MANEUVER_STEEP_DRIFT:
            return 3;  // D3

        case MANEUVER_BEND:
            // D value based on angle: 15°=D1, 30°=D2, 45°=D3, 60°=D4, 75°=D5, 90°=D6
            return (param + 14) / 15;  // Integer division rounds down

        case MANEUVER_PIVOT:
            return 0;  // D0

        case MANEUVER_T_STOP:
            // D1 per 10 mph of speed
            return param;  // param should be speed/10

        case MANEUVER_BOOTLEGGER:
            return 7;  // D7

        case MANEUVER_SWERVE:
            // Drift + Bend, +1 difficulty
            return param + 1;  // param is the bend difficulty

        case MANEUVER_CONTROLLED_SKID:
            // +1 to +4 based on distance
            return param;  // 1-4 quarter inches

        default:
            return 0;
    }
}

// Calculate target position for a maneuver
static void calculate_target(ManeuverAutopilot* ap,
                             Vec3 start_pos,
                             float start_heading,
                             float speed_ms) {
    float dir = (float)ap->request.direction;  // -1 or 1

    // Default: maintain speed and heading
    ap->target_speed_ms = speed_ms;
    ap->target_heading = start_heading;

    // Calculate lateral offset in world coordinates
    // heading = 0 means facing +Z, so right is +X
    float sin_h = sinf(start_heading);
    float cos_h = cosf(start_heading);

    // Right vector (perpendicular to heading)
    float right_x = cos_h;
    float right_z = -sin_h;

    // Forward vector
    float fwd_x = sin_h;
    float fwd_z = cos_h;

    // Forward distance during maneuver (based on speed and expected duration)
    float fwd_dist = 0.0f;

    switch (ap->request.type) {
        case MANEUVER_DRIFT:
            // 1/4" lateral displacement, maintain heading
            ap->target_position.x = start_pos.x + right_x * CW_QUARTER_INCH * dir;
            ap->target_position.y = start_pos.y;
            ap->target_position.z = start_pos.z + right_z * CW_QUARTER_INCH * dir;

            // Add forward movement based on speed
            fwd_dist = speed_ms * ap->expected_duration;
            ap->target_position.x += fwd_x * fwd_dist;
            ap->target_position.z += fwd_z * fwd_dist;
            break;

        case MANEUVER_STEEP_DRIFT:
            // 1/2" lateral displacement, maintain heading
            ap->target_position.x = start_pos.x + right_x * CW_HALF_INCH * dir;
            ap->target_position.y = start_pos.y;
            ap->target_position.z = start_pos.z + right_z * CW_HALF_INCH * dir;

            // Add forward movement
            fwd_dist = speed_ms * ap->expected_duration;
            ap->target_position.x += fwd_x * fwd_dist;
            ap->target_position.z += fwd_z * fwd_dist;
            break;

        case MANEUVER_BEND:
            // TODO: Implement arc movement with heading change
            ap->target_heading = start_heading + (float)ap->request.bend_angle * (3.14159f / 180.0f) * dir;
            ap->target_position = start_pos;  // Placeholder
            break;

        default:
            // Placeholder for other maneuvers
            ap->target_position = start_pos;
            break;
    }

    printf("[Maneuver] Target: pos=(%.2f, %.2f, %.2f) heading=%.1f°\n",
           ap->target_position.x, ap->target_position.y, ap->target_position.z,
           ap->target_heading * 180.0f / 3.14159f);
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
    ap->state = AUTOPILOT_STARTING;
    ap->request = *request;

    // Capture start state
    ap->start_position = current_pos;
    ap->start_heading = current_heading;
    ap->start_speed_ms = current_speed_ms;

    // Calculate expected duration based on speed and maneuver distance
    // Maneuver covers some forward distance while executing
    float target_lateral = 0.0f;
    switch (request->type) {
        case MANEUVER_DRIFT:
            target_lateral = CW_QUARTER_INCH;
            break;
        case MANEUVER_STEEP_DRIFT:
            target_lateral = CW_HALF_INCH;
            break;
        default:
            target_lateral = CW_QUARTER_INCH;  // Default
            break;
    }

    // Duration: time to cover forward while drifting
    // At 20mph (8.8 m/s), 6m = 0.68s was too short
    // Try 10m = 1.14s for more time to complete steering phases
    float forward_dist = 10.0f;  // ~2.5 car lengths
    ap->expected_duration = forward_dist / fmaxf(current_speed_ms, 5.0f);
    ap->timeout = ap->expected_duration * 3.0f;  // 3x safety margin

    // Tolerances for completion detection
    // Use ~10% of target as tolerance (0.15m for 1.14m drift target)
    ap->position_tolerance = 0.15f;  // 0.15 meter tolerance (~10%)
    ap->heading_tolerance = 0.1f;    // ~6 degrees

    // Calculate target
    calculate_target(ap, current_pos, current_heading, current_speed_ms);

    // ========== DETAILED DEBUG OUTPUT ==========
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║               MANEUVER AUTOPILOT START                       ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║ Type: %-20s Direction: %-10s          ║\n",
           maneuver_get_name(request->type),
           request->direction == MANEUVER_LEFT ? "LEFT" : "RIGHT");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║ START STATE:                                                 ║\n");
    printf("║   Position: (%.2f, %.2f, %.2f)                            \n",
           current_pos.x, current_pos.y, current_pos.z);
    printf("║   Heading:  %.1f°                                           \n",
           current_heading * 180.0f / 3.14159f);
    printf("║   Speed:    %.1f mph (%.1f m/s)                             \n",
           current_speed_ms * MS_TO_MPH, current_speed_ms);
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║ TARGET:                                                      ║\n");
    printf("║   Lateral:  %.2fm %s (Car Wars: %.2f\")                    \n",
           target_lateral, request->direction == MANEUVER_LEFT ? "LEFT" : "RIGHT",
           target_lateral / CW_INCH_TO_METERS);
    printf("║   Duration: %.2fs (timeout: %.2fs)                          \n",
           ap->expected_duration, ap->timeout);
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    fflush(stdout);

    ap->state = AUTOPILOT_EXECUTING;
    return true;
}

// Track last progress print to avoid spamming
static float s_last_progress_print = -1.0f;

float maneuver_update(ManeuverAutopilot* ap,
                      Vec3 current_pos,
                      float current_heading,
                      float current_speed_ms,
                      float dt,
                      bool* out_complete) {
    *out_complete = false;

    if (ap->state == AUTOPILOT_IDLE ||
        ap->state == AUTOPILOT_FINISHED ||
        ap->state == AUTOPILOT_FAILED) {
        return 0.0f;
    }

    // Handle smooth heading correction phase
    if (ap->state == AUTOPILOT_CORRECTING) {
        ap->correction_elapsed += dt;
        float t = ap->correction_elapsed / ap->correction_duration;

        if (t >= 1.0f) {
            // Correction complete - set final heading exactly
            ap->heading_nudge_pending = true;
            ap->heading_nudge_target = ap->start_heading;
            ap->state = AUTOPILOT_FINISHED;
            *out_complete = true;
            printf("[Autopilot] Heading correction complete (%.1f° -> %.1f°)\n",
                   ap->correction_start_heading * 180.0f / 3.14159f,
                   ap->start_heading * 180.0f / 3.14159f);
            fflush(stdout);
            return 0.0f;
        }

        // Smooth interpolation using ease-out curve: 1 - (1-t)^2
        float ease_t = 1.0f - (1.0f - t) * (1.0f - t);

        // Interpolate heading
        float target_heading = ap->correction_start_heading +
                               (ap->start_heading - ap->correction_start_heading) * ease_t;

        // Debug: print progress at 50%
        static bool printed_50 = false;
        if (t > 0.5f && !printed_50) {
            printf("[Autopilot] Correction animating: %.0f%% (heading=%.1f°)\n",
                   t * 100.0f, target_heading * 180.0f / 3.14159f);
            fflush(stdout);
            printed_50 = true;
        }
        if (t < 0.1f) {
            printed_50 = false;  // Reset for next correction
        }

        // Request this frame's heading adjustment
        ap->heading_nudge_pending = true;
        ap->heading_nudge_target = target_heading;

        return 0.0f;  // No steering during correction
    }

    ap->elapsed += dt;

    // Calculate current displacement from start
    float dx = current_pos.x - ap->start_position.x;
    float dz = current_pos.z - ap->start_position.z;

    // Project onto forward/lateral directions
    float sin_h = sinf(ap->start_heading);
    float cos_h = cosf(ap->start_heading);

    ap->forward_displacement = dx * sin_h + dz * cos_h;
    ap->lateral_displacement = dx * cos_h - dz * sin_h;

    // Target lateral displacement (signed)
    float target_lateral = 0.0f;
    float target_lateral_unsigned = 0.0f;
    switch (ap->request.type) {
        case MANEUVER_DRIFT:
            target_lateral_unsigned = CW_QUARTER_INCH;
            break;
        case MANEUVER_STEEP_DRIFT:
            target_lateral_unsigned = CW_HALF_INCH;
            break;
        default:
            target_lateral_unsigned = CW_QUARTER_INCH;
            break;
    }
    target_lateral = target_lateral_unsigned * (float)ap->request.direction;

    // Simple steering profile for drift:
    // Phase 1: Steer in direction to initiate lateral movement
    // Phase 2: Counter-steer to stop lateral movement and maintain heading

    const SteeringProfile* profile = &s_drift_profile;
    if (ap->request.type == MANEUVER_STEEP_DRIFT) {
        profile = &s_steep_drift_profile;
    }

    float steer = 0.0f;
    // Invert direction for physics steering convention
    // Testing showed: MANEUVER_LEFT(-1) with negative steer caused car to go RIGHT
    // So we invert: LEFT drift needs positive steer, RIGHT drift needs negative steer
    float dir = -(float)ap->request.direction;

    float progress = ap->elapsed / ap->expected_duration;

    // Phase boundaries (tuned for balance)
    // Test 7: 45%/90%, full duration -> 2.14m (overdrift by 2x!), -9.2° heading
    // Problem: too much lateral. Reduce phase1 to 30%, keep counter-steer
    const float PHASE1_END = 0.30f;  // Steer phase: 0-30%
    const float PHASE2_END = 0.85f;  // Counter-steer phase: 30-85%

    if (progress < PHASE1_END) {
        // Phase 1: Steer to initiate drift
        // Use full intensity at start, taper off
        float phase_progress = progress / PHASE1_END;
        steer = dir * profile->steer_intensity * (1.0f - phase_progress * 0.5f);
    } else if (progress < PHASE2_END) {
        // Phase 2: Counter-steer to straighten heading
        float phase_progress = (progress - PHASE1_END) / (PHASE2_END - PHASE1_END);
        steer = -dir * profile->return_intensity * (1.0f - phase_progress * 0.5f);

        // At start of Phase 2 (just crossed PHASE1_END), request lateral correction
        if (!ap->phase1_complete) {
            ap->phase1_complete = true;
            // Calculate how much lateral correction is needed
            float lateral_error = target_lateral - ap->lateral_displacement;
            if (fabsf(lateral_error) > 0.05f) {  // Only nudge if > 5cm off
                ap->lateral_nudge_pending = true;
                ap->lateral_nudge_amount = lateral_error;
                printf("[Autopilot] Phase 1 complete: lateral=%.2fm, target=%.2fm, nudge=%.2fm\n",
                       ap->lateral_displacement, target_lateral, lateral_error);
            }
        }
    } else {
        // Phase 3: Center steering, let physics settle
        steer = 0.0f;
    }

    ap->steer_intensity = steer;

    // ========== PERIODIC PROGRESS OUTPUT ==========
    // Print every 25% progress
    float progress_pct = progress * 100.0f;
    if ((int)(progress_pct / 25.0f) > (int)(s_last_progress_print / 25.0f) || s_last_progress_print < 0) {
        printf("[Autopilot] t=%.2fs (%.0f%%) | steer=%.2f | lat=%.2fm | fwd=%.2fm | heading=%.1f°\n",
               ap->elapsed, progress_pct, steer,
               ap->lateral_displacement, ap->forward_displacement,
               current_heading * 180.0f / 3.14159f);
        fflush(stdout);
        s_last_progress_print = progress_pct;
    }

    // Check timeout
    if (ap->elapsed > ap->timeout) {
        printf("\n");
        printf("╔══════════════════════════════════════════════════════════════╗\n");
        printf("║               MANEUVER AUTOPILOT TIMEOUT                     ║\n");
        printf("╠══════════════════════════════════════════════════════════════╣\n");
        printf("║ Elapsed: %.2fs (timeout: %.2fs)                             \n",
               ap->elapsed, ap->timeout);
        printf("║ Final lateral:  %.2fm (target: %.2fm)                       \n",
               ap->lateral_displacement, target_lateral);
        printf("║ Final forward:  %.2fm                                       \n",
               ap->forward_displacement);
        printf("╚══════════════════════════════════════════════════════════════╝\n");
        fflush(stdout);
        ap->state = AUTOPILOT_FAILED;
        *out_complete = true;
        s_last_progress_print = -1.0f;
        return 0.0f;
    }

    // Check completion: only when full duration elapsed
    // Don't complete early - let all steering phases run so heading recovers
    if (progress >= 1.0f) {

        // Calculate accuracy - how close to target (100% = perfect, 0% = no movement or way off)
        float lateral_error = fabsf(fabsf(ap->lateral_displacement) - target_lateral_unsigned);
        float accuracy = 100.0f * fmaxf(0.0f, 1.0f - lateral_error / target_lateral_unsigned);

        printf("\n");
        printf("╔══════════════════════════════════════════════════════════════╗\n");
        printf("║               MANEUVER AUTOPILOT COMPLETE                    ║\n");
        printf("╠══════════════════════════════════════════════════════════════╣\n");
        printf("║ Duration: %.2fs                                             \n",
               ap->elapsed);
        printf("╠══════════════════════════════════════════════════════════════╣\n");
        printf("║ END STATE:                                                   ║\n");
        printf("║   Position: (%.2f, %.2f, %.2f)                            \n",
               current_pos.x, current_pos.y, current_pos.z);
        printf("║   Heading:  %.1f° (start: %.1f°, delta: %.1f°)              \n",
               current_heading * 180.0f / 3.14159f,
               ap->start_heading * 180.0f / 3.14159f,
               (current_heading - ap->start_heading) * 180.0f / 3.14159f);
        printf("╠══════════════════════════════════════════════════════════════╣\n");
        printf("║ DISPLACEMENT:                                                ║\n");
        printf("║   Lateral:  %.2fm (target: %.2fm) = %.0f%% accuracy        \n",
               ap->lateral_displacement, target_lateral, accuracy);
        printf("║   Forward:  %.2fm                                           \n",
               ap->forward_displacement);
        printf("╠══════════════════════════════════════════════════════════════╣\n");

        // Evaluate based on both lateral AND heading recovery
        // Heading recovery is priority - drift should end pointing straight
        float heading_delta = fabsf(current_heading - ap->start_heading) * 180.0f / 3.14159f;
        bool heading_ok = heading_delta < 10.0f;  // Within 10 degrees
        bool lateral_ok = accuracy > 60.0f;       // At least 60% of target

        if (heading_ok && lateral_ok) {
            printf("║   STATUS: ✓ GOOD - drift complete, heading recovered        ║\n");
        } else if (!heading_ok) {
            printf("║   STATUS: ✗ HEADING - car still turned %.0f° (need <10°)     \n", heading_delta);
        } else {
            printf("║   STATUS: ~ LATERAL - only %.0f%% of target drift            \n", accuracy);
        }
        printf("╚══════════════════════════════════════════════════════════════╝\n");
        fflush(stdout);

        // Check if heading correction is needed (> 2 degrees off)
        float heading_error = current_heading - ap->start_heading;
        if (fabsf(heading_error) > 0.035f) {  // ~2 degrees
            // Store correction for post-maneuver animation (happens when unpaused)
            ap->state = AUTOPILOT_CORRECTING;  // Mark that correction is pending
            ap->correction_elapsed = 0.0f;
            ap->correction_duration = 0.25f;  // 0.25 seconds for smooth animation
            ap->correction_start_heading = current_heading;
            printf("[Autopilot] Heading correction queued: %.1f° -> %.1f° (will animate on unpause)\n",
                   current_heading * 180.0f / 3.14159f,
                   ap->start_heading * 180.0f / 3.14159f);
        } else {
            ap->state = AUTOPILOT_FINISHED;
        }

        // Signal complete - physics will pause, correction runs on next unpause
        *out_complete = true;
        s_last_progress_print = -1.0f;
        return 0.0f;
    }

    return steer;
}

void maneuver_cancel(ManeuverAutopilot* ap) {
    if (ap->state != AUTOPILOT_IDLE) {
        printf("[Maneuver] Cancelled after %.2fs\n", ap->elapsed);
    }
    ap->state = AUTOPILOT_IDLE;
    ap->steer_intensity = 0.0f;
}

bool maneuver_is_active(const ManeuverAutopilot* ap) {
    return ap->state == AUTOPILOT_STARTING ||
           ap->state == AUTOPILOT_EXECUTING ||
           ap->state == AUTOPILOT_COMPLETING ||
           ap->state == AUTOPILOT_CORRECTING;
}
