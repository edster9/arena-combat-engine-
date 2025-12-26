/*
 * ODE Physics Implementation
 * Vehicle physics with independent wheel suspension using Hinge2 joints
 */

#include "ode_physics.h"
#include "../render/line_render.h"
#include <ode/ode.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Contact parameters
#define MAX_CONTACTS 10
#define CONTACT_SURFACE_MU 3.0f       // High friction for good traction
#define CONTACT_SURFACE_SLIP1 0.0001f // Minimal slip
#define CONTACT_SURFACE_SLIP2 0.0001f
#define CONTACT_SOFT_ERP 0.8f         // Stiffer contacts
#define CONTACT_SOFT_CFM 0.0001f

// Static world pointer for collision callback
static PhysicsWorld* g_current_world = NULL;

// Find vehicle that owns a given geometry (returns NULL if not a vehicle wheel)
static PhysicsVehicle* find_vehicle_for_geom(dGeomID geom) {
    if (!g_current_world) return NULL;

    for (int i = 0; i < g_current_world->vehicle_count; i++) {
        PhysicsVehicle* v = &g_current_world->vehicles[i];
        if (!v->active) continue;

        // Check if this geom is one of the vehicle's wheels
        for (int w = 0; w < 4; w++) {
            if (v->wheel_geoms[w] == geom) {
                return v;
            }
        }
    }
    return NULL;
}

// Collision callback
static void near_callback(void* data, dGeomID o1, dGeomID o2) {
    (void)data;

    // Get the bodies attached to the geometries
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    // Skip if same body or both static
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
        return;
    }

    // Create contact joints
    dContact contacts[MAX_CONTACTS];
    int num_contacts = dCollide(o1, o2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact));

    // Check if either geometry is a wheel - use its vehicle's tire friction
    PhysicsVehicle* v1 = find_vehicle_for_geom(o1);
    PhysicsVehicle* v2 = find_vehicle_for_geom(o2);

    // Use vehicle tire friction if available, otherwise default
    float friction = CONTACT_SURFACE_MU;
    float slip = CONTACT_SURFACE_SLIP1;

    if (v1) {
        friction = v1->config.tire_friction;
        slip = v1->config.tire_slip;
    } else if (v2) {
        friction = v2->config.tire_friction;
        slip = v2->config.tire_slip;
    }

    for (int i = 0; i < num_contacts; i++) {
        contacts[i].surface.mode = dContactSlip1 | dContactSlip2 |
                                   dContactSoftERP | dContactSoftCFM |
                                   dContactApprox1;
        contacts[i].surface.mu = friction;
        contacts[i].surface.slip1 = slip;
        contacts[i].surface.slip2 = slip;
        contacts[i].surface.soft_erp = CONTACT_SOFT_ERP;
        contacts[i].surface.soft_cfm = CONTACT_SOFT_CFM;

        dJointID c = dJointCreateContact(g_current_world->world,
                                          g_current_world->contact_group,
                                          &contacts[i]);
        dJointAttach(c, b1, b2);
    }
}

bool physics_init(PhysicsWorld* pw) {
    memset(pw, 0, sizeof(PhysicsWorld));

    // Initialize ODE
    dInitODE();

    // Create world
    pw->world = dWorldCreate();
    dWorldSetGravity(pw->world, 0, -9.81f, 0);
    dWorldSetERP(pw->world, 0.8f);
    dWorldSetCFM(pw->world, 1e-5f);

    // Auto-disable for performance (bodies at rest stop simulating)
    dWorldSetAutoDisableFlag(pw->world, 1);
    dWorldSetAutoDisableLinearThreshold(pw->world, 0.01f);
    dWorldSetAutoDisableAngularThreshold(pw->world, 0.01f);
    dWorldSetAutoDisableSteps(pw->world, 10);

    // Create collision space
    pw->space = dHashSpaceCreate(0);

    // Create contact joint group
    pw->contact_group = dJointGroupCreate(0);

    // Default timestep (60 Hz physics)
    pw->step_size = 1.0f / 60.0f;
    pw->accumulator = 0;

    printf("ODE Physics initialized\n");
    return true;
}

void physics_destroy(PhysicsWorld* pw) {
    // Destroy all vehicles
    for (int i = 0; i < pw->vehicle_count; i++) {
        if (pw->vehicles[i].active) {
            physics_destroy_vehicle(pw, i);
        }
    }

    // Destroy ODE objects
    if (pw->ground) dGeomDestroy(pw->ground);
    dJointGroupDestroy(pw->contact_group);
    dSpaceDestroy(pw->space);
    dWorldDestroy(pw->world);
    dCloseODE();

    printf("ODE Physics destroyed\n");
}

void physics_step(PhysicsWorld* pw, float dt) {
    g_current_world = pw;

    // Accumulate time
    pw->accumulator += dt;

    // Fixed timestep simulation
    while (pw->accumulator >= pw->step_size) {
        // Apply vehicle controls
        for (int i = 0; i < pw->vehicle_count; i++) {
            PhysicsVehicle* v = &pw->vehicles[i];
            if (!v->active) continue;

            // Apply steering to steering wheels
            for (int w = 0; w < 4; w++) {
                bool can_steer = v->config.use_per_wheel_config
                               ? v->config.wheel_steering[w]
                               : (w == WHEEL_FL || w == WHEEL_FR);
                if (can_steer) {
                    float max_steer = v->config.use_per_wheel_config
                                    ? v->config.wheel_steer_angles[w]
                                    : v->config.max_steer_angle;
                    float steer_angle = v->steering * max_steer;
                    dJointSetHinge2Param(v->suspensions[w], dParamLoStop, steer_angle);
                    dJointSetHinge2Param(v->suspensions[w], dParamHiStop, steer_angle);
                }
            }

            // Calculate motor parameters
            float motor_speed = 0.0f;
            float motor_force = 0.0f;

            if (v->throttle > 0.01f) {
                // Accelerate forward (negative because of wheel axis direction)
                motor_speed = -v->throttle * 80.0f;  // Target angular velocity
                motor_force = v->config.max_motor_force;
            } else if (v->reverse > 0.01f) {
                // Reverse (positive = backwards)
                motor_speed = v->reverse * 40.0f;  // Slower reverse speed
                motor_force = v->config.max_motor_force * 0.5f;  // Less power in reverse
            } else if (v->brake > 0.01f) {
                // Brake - apply force to stop wheels
                motor_speed = 0;
                motor_force = v->config.max_brake_force * v->brake;
            }

            // Apply motor to driven wheels
            for (int w = 0; w < 4; w++) {
                bool is_driven = v->config.use_per_wheel_config
                               ? v->config.wheel_driven[w]
                               : (w == WHEEL_RL || w == WHEEL_RR);  // Legacy: RWD
                if (is_driven) {
                    dJointSetHinge2Param(v->suspensions[w], dParamVel2, motor_speed);
                    dJointSetHinge2Param(v->suspensions[w], dParamFMax2, motor_force);
                }
            }
        }

        // Collision detection
        dSpaceCollide(pw->space, NULL, near_callback);

        // Step simulation
        dWorldQuickStep(pw->world, pw->step_size);

        // Clear contact joints
        dJointGroupEmpty(pw->contact_group);

        // Update wheel rotation (integrate angular velocity) inside physics loop
        for (int i = 0; i < pw->vehicle_count; i++) {
            PhysicsVehicle* v = &pw->vehicles[i];
            if (!v->active) continue;

            for (int w = 0; w < 4; w++) {
                // Get wheel angular velocity and integrate for cumulative rotation
                float angular_vel = (float)dJointGetHinge2Angle2Rate(v->suspensions[w]);
                v->wheel_states[w].rotation += angular_vel * pw->step_size;

                // Keep angle in reasonable range to avoid floating point issues
                if (v->wheel_states[w].rotation > 6.28318f) v->wheel_states[w].rotation -= 6.28318f;
                if (v->wheel_states[w].rotation < -6.28318f) v->wheel_states[w].rotation += 6.28318f;
            }
        }

        pw->accumulator -= pw->step_size;
    }

    // Update wheel states for rendering (positions and steering)
    for (int i = 0; i < pw->vehicle_count; i++) {
        PhysicsVehicle* v = &pw->vehicles[i];
        if (!v->active) continue;

        for (int w = 0; w < 4; w++) {
            const dReal* pos = dBodyGetPosition(v->wheels[w]);
            v->wheel_states[w].position = vec3((float)pos[0], (float)pos[1], (float)pos[2]);

            // Get steering angle for front wheels
            if (w == WHEEL_FL || w == WHEEL_FR) {
                v->wheel_states[w].steer_angle = (float)dJointGetHinge2Angle1(v->suspensions[w]);
            }
        }
    }
}

void physics_set_ground(PhysicsWorld* pw, float y_level) {
    if (pw->ground) {
        dGeomDestroy(pw->ground);
    }
    // Create infinite ground plane at y_level, normal pointing up
    pw->ground = dCreatePlane(pw->space, 0, 1, 0, y_level);
}

void physics_add_box_obstacle(PhysicsWorld* pw, Vec3 pos, Vec3 size) {
    dGeomID box = dCreateBox(pw->space, size.x, size.y, size.z);
    dGeomSetPosition(box, pos.x, pos.y, pos.z);
    // Static obstacle (no body attached)
}

void physics_add_arena_walls(PhysicsWorld* pw, float arena_size, float wall_height, float wall_thickness) {
    float half = arena_size / 2.0f;
    float y = wall_height / 2.0f;
    float t = wall_thickness;

    // North wall (positive Z)
    physics_add_box_obstacle(pw, vec3(0, y, half + t/2), vec3(arena_size + t*2, wall_height, t));
    // South wall (negative Z)
    physics_add_box_obstacle(pw, vec3(0, y, -half - t/2), vec3(arena_size + t*2, wall_height, t));
    // East wall (positive X)
    physics_add_box_obstacle(pw, vec3(half + t/2, y, 0), vec3(t, wall_height, arena_size));
    // West wall (negative X)
    physics_add_box_obstacle(pw, vec3(-half - t/2, y, 0), vec3(t, wall_height, arena_size));

    printf("Added arena walls: size=%.1f height=%.1f thickness=%.1f\n", arena_size, wall_height, t);
}

int physics_create_vehicle(PhysicsWorld* pw, Vec3 position, float rotation_y,
                           const VehicleConfig* config) {
    if (pw->vehicle_count >= MAX_PHYSICS_VEHICLES) {
        fprintf(stderr, "Max vehicles reached\n");
        return -1;
    }

    int id = pw->vehicle_count++;
    PhysicsVehicle* v = &pw->vehicles[id];
    memset(v, 0, sizeof(PhysicsVehicle));
    v->id = id;
    v->active = true;
    v->config = *config;

    // Store spawn position for respawn
    v->spawn_position = position;
    v->spawn_rotation = rotation_y;

    // Determine wheel positions (v2 per-wheel or legacy calculated)
    Vec3 wheel_offsets[4];
    float wheel_radii[4];
    float wheel_widths[4];
    float wheel_masses[4];

    if (config->use_per_wheel_config) {
        // V2: Use explicit per-wheel positions from JSON
        for (int w = 0; w < 4; w++) {
            wheel_offsets[w] = config->wheel_positions[w];
            wheel_radii[w] = config->wheel_radii[w];
            wheel_widths[w] = config->wheel_widths[w];
            wheel_masses[w] = config->wheel_masses[w];
        }
    } else {
        // Legacy: Calculate wheel positions from wheelbase/track_width or chassis dimensions
        float wz = (config->wheelbase > 0.0f)
                   ? config->wheelbase * 0.5f
                   : config->chassis_length * 0.35f;
        float wx = (config->track_width > 0.0f)
                   ? config->track_width * 0.5f
                   : config->chassis_width * 0.5f + config->wheel_width * 0.6f;
        float wy = (config->wheel_mount_height > 0.0f)
                   ? -config->wheel_mount_height
                   : -config->chassis_height * 0.5f;

        wheel_offsets[WHEEL_FL] = vec3(-wx, wy,  wz);
        wheel_offsets[WHEEL_FR] = vec3( wx, wy,  wz);
        wheel_offsets[WHEEL_RL] = vec3(-wx, wy, -wz);
        wheel_offsets[WHEEL_RR] = vec3( wx, wy, -wz);

        for (int w = 0; w < 4; w++) {
            wheel_radii[w] = config->wheel_radius;
            wheel_widths[w] = config->wheel_width;
            wheel_masses[w] = config->wheel_mass;
        }
    }

    // Calculate chassis spawn height:
    // For each wheel, find how high the chassis needs to be for that wheel to touch ground
    // chassis_y = max(wheel_radius - wheel_position.y) + position.y
    // This ensures the largest wheel (relative to its mount point) touches ground
    float max_lift = 0.0f;
    for (int w = 0; w < 4; w++) {
        // wheel_position.y is relative to chassis center (negative = below)
        // wheel needs to be at wheel_radius above ground
        // so chassis center needs to be at: wheel_radius - wheel_position.y
        float lift = wheel_radii[w] - wheel_offsets[w].y;
        if (lift > max_lift) max_lift = lift;
    }
    float chassis_y = position.y + max_lift;

    // Create chassis body
    v->chassis = dBodyCreate(pw->world);
    dBodySetPosition(v->chassis, position.x, chassis_y, position.z);

    // Set chassis rotation
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 1, 0, rotation_y);
    dBodySetRotation(v->chassis, R);

    // Chassis mass
    dMass mass;
    dMassSetBoxTotal(&mass, config->chassis_mass,
                     config->chassis_width, config->chassis_height, config->chassis_length);
    dBodySetMass(v->chassis, &mass);

    // Disable auto-disable for chassis (we want it always active)
    dBodySetAutoDisableFlag(v->chassis, 0);

    // Chassis collision geometry
    v->chassis_geom = dCreateBox(pw->space,
                                  config->chassis_width,
                                  config->chassis_height,
                                  config->chassis_length);
    dGeomSetBody(v->chassis_geom, v->chassis);

    // Create wheels
    for (int w = 0; w < 4; w++) {
        v->wheels[w] = dBodyCreate(pw->world);

        // Transform wheel offset by chassis rotation
        float wx_rotated = wheel_offsets[w].x * cosf(rotation_y) - wheel_offsets[w].z * sinf(rotation_y);
        float wz_rotated = wheel_offsets[w].x * sinf(rotation_y) + wheel_offsets[w].z * cosf(rotation_y);

        // Position wheels relative to chassis center
        float wheel_x = position.x + wx_rotated;
        float wheel_y = chassis_y + wheel_offsets[w].y;
        float wheel_z = position.z + wz_rotated;

        dBodySetPosition(v->wheels[w], wheel_x, wheel_y, wheel_z);

        // Wheel mass (cylinder) - use per-wheel values
        dMass wheel_mass;
        dMassSetCylinderTotal(&wheel_mass, wheel_masses[w],
                              3,  // Z-axis aligned cylinder
                              wheel_radii[w], wheel_widths[w]);
        dBodySetMass(v->wheels[w], &wheel_mass);

        // Disable auto-disable for wheels
        dBodySetAutoDisableFlag(v->wheels[w], 0);

        // Wheel collision (sphere) - use per-wheel radius
        v->wheel_geoms[w] = dCreateSphere(pw->space, wheel_radii[w]);
        dGeomSetBody(v->wheel_geoms[w], v->wheels[w]);

        // Create Hinge2 joint (suspension + steering)
        v->suspensions[w] = dJointCreateHinge2(pw->world, 0);
        dJointAttach(v->suspensions[w], v->chassis, v->wheels[w]);
        dJointSetHinge2Anchor(v->suspensions[w], wheel_x, wheel_y, wheel_z);

        // Axis 1: Suspension/steering axis (points up)
        // Axis 2: Wheel rotation axis (lateral)
        float right_x = cosf(rotation_y);
        float right_z = -sinf(rotation_y);

        dReal axis1[3] = {0, 1, 0};
        dReal axis2[3] = {right_x, 0, right_z};
        dJointSetHinge2Axes(v->suspensions[w], axis1, axis2);

        // Suspension parameters - use per-wheel values if available
        float susp_erp = config->use_per_wheel_config
                       ? config->wheel_suspension_erp[w]
                       : config->suspension_erp;
        float susp_cfm = config->use_per_wheel_config
                       ? config->wheel_suspension_cfm[w]
                       : config->suspension_cfm;
        dJointSetHinge2Param(v->suspensions[w], dParamSuspensionERP, susp_erp);
        dJointSetHinge2Param(v->suspensions[w], dParamSuspensionCFM, susp_cfm);

        // Steering limits - use per-wheel config or legacy
        if (config->use_per_wheel_config) {
            if (config->wheel_steering[w]) {
                float steer = config->wheel_steer_angles[w];
                dJointSetHinge2Param(v->suspensions[w], dParamLoStop, -steer);
                dJointSetHinge2Param(v->suspensions[w], dParamHiStop, steer);
            } else {
                dJointSetHinge2Param(v->suspensions[w], dParamLoStop, 0);
                dJointSetHinge2Param(v->suspensions[w], dParamHiStop, 0);
            }
        } else {
            // Legacy: front wheels steer
            if (w == WHEEL_FL || w == WHEEL_FR) {
                dJointSetHinge2Param(v->suspensions[w], dParamLoStop, -config->max_steer_angle);
                dJointSetHinge2Param(v->suspensions[w], dParamHiStop, config->max_steer_angle);
            } else {
                dJointSetHinge2Param(v->suspensions[w], dParamLoStop, 0);
                dJointSetHinge2Param(v->suspensions[w], dParamHiStop, 0);
            }
        }
    }

    // Initialize wheel states for rendering
    for (int w = 0; w < 4; w++) {
        const dReal* wpos = dBodyGetPosition(v->wheels[w]);
        v->wheel_states[w].position = vec3((float)wpos[0], (float)wpos[1], (float)wpos[2]);
        v->wheel_states[w].rotation = 0;
        v->wheel_states[w].steer_angle = 0;
        v->wheel_states[w].suspension_compression = 0;
    }

    printf("Created physics vehicle %d at (%.1f, %.1f, %.1f) chassis_y=%.2f\n",
           id, position.x, position.y, position.z, chassis_y);
    return id;
}

void physics_destroy_vehicle(PhysicsWorld* pw, int vehicle_id) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    // Destroy joints
    for (int w = 0; w < 4; w++) {
        if (v->suspensions[w]) dJointDestroy(v->suspensions[w]);
    }

    // Destroy geometries
    if (v->chassis_geom) dGeomDestroy(v->chassis_geom);
    for (int w = 0; w < 4; w++) {
        if (v->wheel_geoms[w]) dGeomDestroy(v->wheel_geoms[w]);
    }

    // Destroy bodies
    if (v->chassis) dBodyDestroy(v->chassis);
    for (int w = 0; w < 4; w++) {
        if (v->wheels[w]) dBodyDestroy(v->wheels[w]);
    }

    v->active = false;
    printf("Destroyed physics vehicle %d\n", vehicle_id);
}

void physics_vehicle_set_steering(PhysicsWorld* pw, int vehicle_id, float steering) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->steering = steering;
    if (v->steering > 1.0f) v->steering = 1.0f;
    if (v->steering < -1.0f) v->steering = -1.0f;
}

void physics_vehicle_set_throttle(PhysicsWorld* pw, int vehicle_id, float throttle) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->throttle = throttle;
    if (v->throttle > 1.0f) v->throttle = 1.0f;
    if (v->throttle < 0.0f) v->throttle = 0.0f;
}

void physics_vehicle_set_reverse(PhysicsWorld* pw, int vehicle_id, float reverse) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->reverse = reverse;
    if (v->reverse > 1.0f) v->reverse = 1.0f;
    if (v->reverse < 0.0f) v->reverse = 0.0f;
}

void physics_vehicle_set_brake(PhysicsWorld* pw, int vehicle_id, float brake) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->brake = brake;
    if (v->brake > 1.0f) v->brake = 1.0f;
    if (v->brake < 0.0f) v->brake = 0.0f;
}

void physics_vehicle_respawn(PhysicsWorld* pw, int vehicle_id) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    VehicleConfig* config = &v->config;
    Vec3 pos = v->spawn_position;
    float rotation_y = v->spawn_rotation;

    // Determine wheel positions (v2 per-wheel or legacy)
    Vec3 wheel_offsets[4];
    float wheel_radii[4];

    if (config->use_per_wheel_config) {
        for (int w = 0; w < 4; w++) {
            wheel_offsets[w] = config->wheel_positions[w];
            wheel_radii[w] = config->wheel_radii[w];
        }
    } else {
        float wz = (config->wheelbase > 0.0f)
                   ? config->wheelbase * 0.5f
                   : config->chassis_length * 0.35f;
        float wx = (config->track_width > 0.0f)
                   ? config->track_width * 0.5f
                   : config->chassis_width * 0.5f + config->wheel_width * 0.6f;
        float wy = (config->wheel_mount_height > 0.0f)
                   ? -config->wheel_mount_height
                   : -config->chassis_height * 0.5f;

        wheel_offsets[WHEEL_FL] = vec3(-wx, wy,  wz);
        wheel_offsets[WHEEL_FR] = vec3( wx, wy,  wz);
        wheel_offsets[WHEEL_RL] = vec3(-wx, wy, -wz);
        wheel_offsets[WHEEL_RR] = vec3( wx, wy, -wz);

        for (int w = 0; w < 4; w++) {
            wheel_radii[w] = config->wheel_radius;
        }
    }

    // Calculate chassis spawn height
    float max_lift = 0.0f;
    for (int w = 0; w < 4; w++) {
        float lift = wheel_radii[w] - wheel_offsets[w].y;
        if (lift > max_lift) max_lift = lift;
    }
    float chassis_y = pos.y + max_lift;

    // Reset chassis position, rotation, and velocities
    dBodySetPosition(v->chassis, pos.x, chassis_y, pos.z);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 1, 0, rotation_y);
    dBodySetRotation(v->chassis, R);
    dBodySetLinearVel(v->chassis, 0, 0, 0);
    dBodySetAngularVel(v->chassis, 0, 0, 0);

    // Reset wheels
    for (int w = 0; w < 4; w++) {
        float wx_rotated = wheel_offsets[w].x * cosf(rotation_y) - wheel_offsets[w].z * sinf(rotation_y);
        float wz_rotated = wheel_offsets[w].x * sinf(rotation_y) + wheel_offsets[w].z * cosf(rotation_y);

        float wheel_x = pos.x + wx_rotated;
        float wheel_y = chassis_y + wheel_offsets[w].y;
        float wheel_z = pos.z + wz_rotated;

        dBodySetPosition(v->wheels[w], wheel_x, wheel_y, wheel_z);
        dBodySetLinearVel(v->wheels[w], 0, 0, 0);
        dBodySetAngularVel(v->wheels[w], 0, 0, 0);
    }

    // Clear inputs
    v->steering = 0;
    v->throttle = 0;
    v->brake = 0;

    printf("Respawned vehicle %d at (%.1f, %.1f, %.1f)\n", vehicle_id, pos.x, pos.y, pos.z);
}

void physics_vehicle_get_position(PhysicsWorld* pw, int vehicle_id, Vec3* pos) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    const dReal* p = dBodyGetPosition(v->chassis);
    pos->x = (float)p[0];
    pos->y = (float)p[1];
    pos->z = (float)p[2];
}

void physics_vehicle_get_rotation(PhysicsWorld* pw, int vehicle_id, float* rotation_y) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    const dReal* R = dBodyGetRotation(v->chassis);
    // Extract Y rotation from rotation matrix
    // R is 3x4 matrix in row-major: R[0..3] = row0, R[4..7] = row1, R[8..11] = row2
    *rotation_y = atan2f((float)R[2], (float)R[0]);
}

void physics_vehicle_get_rotation_matrix(PhysicsWorld* pw, int vehicle_id, float* rot_matrix) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    const dReal* R = dBodyGetRotation(v->chassis);
    // ODE stores 3x4 row-major (with padding), extract 3x3 row-major
    // Row 0
    rot_matrix[0] = (float)R[0];
    rot_matrix[1] = (float)R[1];
    rot_matrix[2] = (float)R[2];
    // Row 1
    rot_matrix[3] = (float)R[4];
    rot_matrix[4] = (float)R[5];
    rot_matrix[5] = (float)R[6];
    // Row 2
    rot_matrix[6] = (float)R[8];
    rot_matrix[7] = (float)R[9];
    rot_matrix[8] = (float)R[10];
}

void physics_vehicle_get_velocity(PhysicsWorld* pw, int vehicle_id, float* speed_ms) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    const dReal* vel = dBodyGetLinearVel(v->chassis);
    // Speed in m/s (magnitude of horizontal velocity)
    *speed_ms = sqrtf((float)(vel[0]*vel[0] + vel[2]*vel[2]));
}

void physics_vehicle_get_wheel_states(PhysicsWorld* pw, int vehicle_id, WheelState* wheels) {
    if (vehicle_id < 0 || vehicle_id >= pw->vehicle_count) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    memcpy(wheels, v->wheel_states, sizeof(WheelState) * 4);
}

VehicleConfig physics_default_vehicle_config(void) {
    VehicleConfig cfg = {
        .chassis_mass = 600.0f,      // 600 kg (lighter = more responsive)
        .chassis_length = 3.5f,      // 3.5 meters (smaller car)
        .chassis_width = 1.8f,       // 1.8 meters
        .chassis_height = 0.8f,      // 0.8 meters (lower center of gravity)

        .wheel_mass = 15.0f,         // 15 kg per wheel
        .wheel_radius = 0.35f,       // 0.35 meter radius
        .wheel_width = 0.2f,         // 0.2 meter wide

        .suspension_erp = 0.7f,      // Stiffer suspension
        .suspension_cfm = 0.005f,    // Less soft
        .suspension_travel = 0.2f,   // 20cm travel

        .max_steer_angle = 0.7f,     // ~40 degrees (sharper turns)
        .max_motor_force = 20000.0f, // 20000 N (powerful motor)
        .max_brake_force = 15000.0f, // 15000 N

        .tire_friction = 3.0f,       // High friction (standard tires)
        .tire_slip = 0.0001f,        // Minimal slip (good grip)

        // Wheel mount points (0 = calculate from chassis dimensions)
        .wheelbase = 0.0f,
        .track_width = 0.0f,
        .wheel_mount_height = 0.0f,
    };
    return cfg;
}

// Helper to draw a wireframe box at a given position/rotation
static void draw_box_wireframe(LineRenderer* lr, const dReal* pos, const dReal* R,
                                float lx, float ly, float lz, Vec3 color) {
    // Half sizes
    float hx = lx * 0.5f;
    float hy = ly * 0.5f;
    float hz = lz * 0.5f;

    // 8 corners in local space
    float corners_local[8][3] = {
        {-hx, -hy, -hz}, { hx, -hy, -hz}, { hx, -hy,  hz}, {-hx, -hy,  hz},
        {-hx,  hy, -hz}, { hx,  hy, -hz}, { hx,  hy,  hz}, {-hx,  hy,  hz}
    };

    // Transform to world space
    Vec3 corners[8];
    for (int i = 0; i < 8; i++) {
        float lx_i = corners_local[i][0];
        float ly_i = corners_local[i][1];
        float lz_i = corners_local[i][2];
        // R is 3x4 row-major rotation matrix
        corners[i].x = (float)pos[0] + R[0]*lx_i + R[1]*ly_i + R[2]*lz_i;
        corners[i].y = (float)pos[1] + R[4]*lx_i + R[5]*ly_i + R[6]*lz_i;
        corners[i].z = (float)pos[2] + R[8]*lx_i + R[9]*ly_i + R[10]*lz_i;
    }

    // Draw 12 edges
    // Bottom face
    line_renderer_draw_line(lr, corners[0], corners[1], color, 1.0f);
    line_renderer_draw_line(lr, corners[1], corners[2], color, 1.0f);
    line_renderer_draw_line(lr, corners[2], corners[3], color, 1.0f);
    line_renderer_draw_line(lr, corners[3], corners[0], color, 1.0f);
    // Top face
    line_renderer_draw_line(lr, corners[4], corners[5], color, 1.0f);
    line_renderer_draw_line(lr, corners[5], corners[6], color, 1.0f);
    line_renderer_draw_line(lr, corners[6], corners[7], color, 1.0f);
    line_renderer_draw_line(lr, corners[7], corners[4], color, 1.0f);
    // Vertical edges
    line_renderer_draw_line(lr, corners[0], corners[4], color, 1.0f);
    line_renderer_draw_line(lr, corners[1], corners[5], color, 1.0f);
    line_renderer_draw_line(lr, corners[2], corners[6], color, 1.0f);
    line_renderer_draw_line(lr, corners[3], corners[7], color, 1.0f);
}

// Draw a wheel with rotating spokes
static void draw_wheel_with_spokes(LineRenderer* lr, Vec3 center, float radius,
                                    float spin_angle, float steer_angle, float chassis_yaw,
                                    Vec3 rim_color, Vec3 spoke_color) {
    // Number of spokes
    const int NUM_SPOKES = 4;

    // Wheel rotates around the lateral (right) axis of the chassis
    // The spin_angle is how much the wheel has rotated
    // For visualization, we draw spokes as lines from center to rim

    // Calculate the wheel's "right" direction (lateral axis) in world space
    // This is affected by steering for front wheels
    float total_yaw = chassis_yaw + steer_angle;
    float right_x = cosf(total_yaw);
    float right_z = -sinf(total_yaw);

    // The wheel's "forward" direction (perpendicular to lateral)
    float fwd_x = -right_z;
    float fwd_z = right_x;

    // Draw rim circle segments (on the vertical plane perpendicular to lateral axis)
    const int RIM_SEGMENTS = 16;
    for (int i = 0; i < RIM_SEGMENTS; i++) {
        float a1 = (float)i / RIM_SEGMENTS * 2.0f * 3.14159f;
        float a2 = (float)(i + 1) / RIM_SEGMENTS * 2.0f * 3.14159f;

        // Points on rim in wheel's local coords (Y = up, forward = perpendicular to axle)
        float y1 = sinf(a1) * radius;
        float f1 = cosf(a1) * radius;  // forward offset
        float y2 = sinf(a2) * radius;
        float f2 = cosf(a2) * radius;

        Vec3 p1 = vec3(center.x + fwd_x * f1, center.y + y1, center.z + fwd_z * f1);
        Vec3 p2 = vec3(center.x + fwd_x * f2, center.y + y2, center.z + fwd_z * f2);
        line_renderer_draw_line(lr, p1, p2, rim_color, 1.0f);
    }

    // Draw spokes - these rotate with spin_angle
    for (int s = 0; s < NUM_SPOKES; s++) {
        float spoke_angle = spin_angle + (float)s * (2.0f * 3.14159f / NUM_SPOKES);

        // Spoke endpoint in wheel's rotating frame
        float spoke_y = sinf(spoke_angle) * radius * 0.9f;
        float spoke_f = cosf(spoke_angle) * radius * 0.9f;  // forward offset

        Vec3 spoke_end = vec3(
            center.x + fwd_x * spoke_f,
            center.y + spoke_y,
            center.z + fwd_z * spoke_f
        );

        line_renderer_draw_line(lr, center, spoke_end, spoke_color, 1.0f);
    }
}

void physics_debug_draw(PhysicsWorld* pw, LineRenderer* lr) {
    Vec3 chassis_color = vec3(1.0f, 1.0f, 0.0f);  // Yellow for chassis
    Vec3 rim_color = vec3(0.0f, 1.0f, 1.0f);      // Cyan for wheel rim
    Vec3 spoke_color = vec3(1.0f, 1.0f, 1.0f);    // White for spokes
    Vec3 ground_color = vec3(0.3f, 0.8f, 0.3f);   // Green for ground

    // Draw ground plane indicator (just a few lines at y=0)
    for (float x = -30; x <= 30; x += 10) {
        line_renderer_draw_line(lr, vec3(x, 0.01f, -30), vec3(x, 0.01f, 30), ground_color, 0.3f);
    }
    for (float z = -30; z <= 30; z += 10) {
        line_renderer_draw_line(lr, vec3(-30, 0.01f, z), vec3(30, 0.01f, z), ground_color, 0.3f);
    }

    // Draw vehicles
    for (int i = 0; i < pw->vehicle_count; i++) {
        PhysicsVehicle* v = &pw->vehicles[i];
        if (!v->active) continue;

        // Draw chassis box (X=width, Y=height, Z=length to match car model)
        const dReal* chassis_pos = dBodyGetPosition(v->chassis);
        const dReal* chassis_R = dBodyGetRotation(v->chassis);
        draw_box_wireframe(lr, chassis_pos, chassis_R,
                          v->config.chassis_width,
                          v->config.chassis_height,
                          v->config.chassis_length,
                          chassis_color);

        // Get chassis yaw for wheel orientation
        float chassis_yaw = atan2f((float)chassis_R[2], (float)chassis_R[0]);

        // Draw wheels with rotating spokes (use per-wheel radii if available)
        for (int w = 0; w < 4; w++) {
            Vec3 center = v->wheel_states[w].position;
            float spin = v->wheel_states[w].rotation;
            float steer = v->wheel_states[w].steer_angle;
            float radius = v->config.use_per_wheel_config
                         ? v->config.wheel_radii[w]
                         : v->config.wheel_radius;

            draw_wheel_with_spokes(lr, center, radius,
                                   spin, steer, chassis_yaw,
                                   rim_color, spoke_color);
        }
    }
}
