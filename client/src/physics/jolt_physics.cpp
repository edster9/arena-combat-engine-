/*
 * Jolt Physics Integration
 * Vehicle physics with WheeledVehicleController
 */

#include "jolt_physics.h"
#include "../render/line_render.h"

#include <iostream>
#include <cmath>
#include <thread>
#include <cstring>
#include <cstdarg>

// Jolt Physics
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>

using namespace JPH;
using namespace JPH::literals;

// Layer definitions
namespace Layers
{
    static constexpr ObjectLayer NON_MOVING = 0;
    static constexpr ObjectLayer MOVING = 1;
    static constexpr ObjectLayer NUM_LAYERS = 2;
};

namespace BroadPhaseLayers
{
    static constexpr BroadPhaseLayer NON_MOVING(0);
    static constexpr BroadPhaseLayer MOVING(1);
    static constexpr uint NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl()
    {
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }

    virtual uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }

    virtual BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
    {
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
    {
        switch ((BroadPhaseLayer::Type)inLayer)
        {
        case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
        case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING: return "MOVING";
        default: return "INVALID";
        }
    }
#endif

private:
    BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

// ObjectVsBroadPhaseLayerFilter
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
    virtual bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case Layers::NON_MOVING:
            return inLayer2 == BroadPhaseLayers::MOVING;
        case Layers::MOVING:
            return true;
        default:
            return false;
        }
    }
};

// ObjectLayerPairFilter
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
    virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
        case Layers::NON_MOVING:
            return inObject2 == Layers::MOVING;
        case Layers::MOVING:
            return true;
        default:
            return false;
        }
    }
};

// Internal vehicle implementation
struct PhysicsVehicleImpl
{
    BodyID bodyId;
    VehicleConstraint* constraint;
};

// Internal world implementation
struct PhysicsWorldImpl
{
    TempAllocatorImpl* tempAllocator;
    JobSystemThreadPool* jobSystem;
    PhysicsSystem* physicsSystem;
    BPLayerInterfaceImpl* broadPhaseLayerInterface;
    ObjectVsBroadPhaseLayerFilterImpl* objectVsBroadPhaseLayerFilter;
    ObjectLayerPairFilterImpl* objectLayerPairFilter;

    BodyID groundBodyId;
    float groundLevel;
};

static void TraceImpl(const char* inFMT, ...)
{
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);
    std::cout << "[Jolt] " << buffer << std::endl;
}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, uint inLine)
{
    std::cout << inFile << ":" << inLine << ": (" << inExpression << ") "
              << (inMessage ? inMessage : "") << std::endl;
    return true;
}
#endif

static bool sJoltInitialized = false;

// No traction control needed - wheels are unpowered in matchbox mode

extern "C" {

bool physics_init(PhysicsWorld* pw)
{
    if (!sJoltInitialized)
    {
        RegisterDefaultAllocator();
        Trace = TraceImpl;
        JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)
        Factory::sInstance = new Factory();
        RegisterTypes();
        sJoltInitialized = true;
    }

    pw->impl = new PhysicsWorldImpl();
    auto* impl = pw->impl;

    impl->tempAllocator = new TempAllocatorImpl(10 * 1024 * 1024);
    impl->jobSystem = new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers,
        std::thread::hardware_concurrency() - 1);

    impl->broadPhaseLayerInterface = new BPLayerInterfaceImpl();
    impl->objectVsBroadPhaseLayerFilter = new ObjectVsBroadPhaseLayerFilterImpl();
    impl->objectLayerPairFilter = new ObjectLayerPairFilterImpl();

    const uint cMaxBodies = 1024;
    const uint cNumBodyMutexes = 0;
    const uint cMaxBodyPairs = 1024;
    const uint cMaxContactConstraints = 1024;

    impl->physicsSystem = new PhysicsSystem();
    impl->physicsSystem->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        *impl->broadPhaseLayerInterface, *impl->objectVsBroadPhaseLayerFilter, *impl->objectLayerPairFilter);

    impl->physicsSystem->SetGravity(JPH::Vec3(0, -9.81f, 0));

    pw->vehicle_count = 0;
    pw->step_size = 1.0f / 60.0f;
    pw->accumulator = 0.0f;

    for (int i = 0; i < MAX_PHYSICS_VEHICLES; i++)
    {
        pw->vehicles[i].active = false;
        pw->vehicles[i].id = i;
        pw->vehicles[i].impl = nullptr;
    }

    impl->groundLevel = 0.0f;

    std::cout << "[Jolt] Physics initialized" << std::endl;
    return true;
}

void physics_destroy(PhysicsWorld* pw)
{
    if (!pw || !pw->impl) return;

    // Destroy all vehicles first
    for (int i = 0; i < MAX_PHYSICS_VEHICLES; i++)
    {
        if (pw->vehicles[i].active)
        {
            physics_destroy_vehicle(pw, i);
        }
    }

    auto* impl = pw->impl;

    delete impl->physicsSystem;
    delete impl->objectLayerPairFilter;
    delete impl->objectVsBroadPhaseLayerFilter;
    delete impl->broadPhaseLayerInterface;
    delete impl->jobSystem;
    delete impl->tempAllocator;
    delete impl;

    pw->impl = nullptr;
    std::cout << "[Jolt] Physics destroyed" << std::endl;
}

void physics_step(PhysicsWorld* pw, float dt)
{
    if (!pw || !pw->impl) return;

    auto* impl = pw->impl;

    pw->accumulator += dt;
    while (pw->accumulator >= pw->step_size)
    {
        // Debug: print active vehicles every 2 seconds
        static float vehicleDebugTimer = 0;
        vehicleDebugTimer += pw->step_size;
        if (vehicleDebugTimer >= 2.0f) {
            printf("Active vehicles: ");
            for (int vi = 0; vi < MAX_PHYSICS_VEHICLES; vi++) {
                if (pw->vehicles[vi].active) {
                    printf("[%d: thr=%.1f] ", vi, pw->vehicles[vi].throttle);
                }
            }
            printf("\n");
            fflush(stdout);
            vehicleDebugTimer = 0;
        }

        // Update vehicle inputs before stepping
        for (int i = 0; i < MAX_PHYSICS_VEHICLES; i++)
        {
            PhysicsVehicle* v = &pw->vehicles[i];
            if (!v->active || !v->impl) continue;

            auto* vimpl = v->impl;
            BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

            // ========== ENGINE RPM MODEL ==========
            // Engine "spins up" when throttle pressed, decays slowly when released
            // This allows quick re-engagement at speed (engine already spun up)
            const float rpmRampUp = 1.0f / 1.5f;   // Full RPM in 1.5 seconds
            const float rpmDecay = 1.0f / 3.0f;    // Decay to zero in 3 seconds
            float stepDt = pw->step_size;          // Use fixed physics timestep

            // Engine RPM: ramps up when throttle pressed, decays slowly when released
            if (v->throttle > 0.0f) {
                // Accelerating: ramp RPM toward 1.0
                v->engine_rpm = fminf(v->engine_rpm + rpmRampUp * stepDt, 1.0f);
            } else {
                // Coasting: decay RPM slowly (engine stays "warm")
                v->engine_rpm = fmaxf(v->engine_rpm - rpmDecay * stepDt, 0.0f);
            }

            // Same for reverse
            if (v->reverse > 0.0f) {
                v->reverse_rpm = fminf(v->reverse_rpm + rpmRampUp * stepDt, 1.0f);
            } else {
                v->reverse_rpm = fmaxf(v->reverse_rpm - rpmDecay * stepDt, 0.0f);
            }

            // Calculate forward input: throttle * engine_rpm (instant power cut, but RPM persists)
            // This means releasing throttle = instant zero force, but re-pressing is fast
            float forward = (v->throttle * v->engine_rpm) - (v->reverse * v->reverse_rpm);

            // ========== CRUISE CONTROL ==========
            // Cancel cruise when brake is pressed
            if (v->cruise_enabled && v->brake > 0.0f) {
                v->cruise_enabled = false;
                v->cruise_target_ms = 0;
                printf("Cruise: OFF (brake pressed)\n");
                fflush(stdout);
            }

            // Apply cruise control if enabled and no manual input
            if (v->cruise_enabled && forward == 0.0f) {
                JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
                float speed = vel.Length();
                float target = v->cruise_target_ms;

                // Proportional cruise control - always apply some force to maintain speed
                float speedDiff = target - speed;
                const float tolerance = 0.1f;  // m/s tolerance (~0.2 mph) - tight for accuracy

                if (speedDiff > tolerance) {
                    // Need to accelerate - proportional throttle with minimum
                    forward = fminf(speedDiff / 3.0f + 0.1f, 1.0f);  // More aggressive + minimum
                } else if (speedDiff < -tolerance) {
                    // Need to slow down - coast (friction will handle it)
                    forward = 0.0f;
                } else {
                    // Within tolerance - apply tiny maintenance force to overcome rolling resistance
                    forward = 0.05f;
                }
            }
            // ========== END CRUISE CONTROL ==========

            // ========== WHEEL TRACTION CHECK ==========
            // Count wheels in contact with ground - scale force by traction
            // This prevents acceleration while airborne or flipped
            WheeledVehicleController* controller = static_cast<WheeledVehicleController*>(
                vimpl->constraint->GetController());
            const Wheels& wheels = vimpl->constraint->GetWheels();

            int wheelsInContact = 0;
            for (size_t w = 0; w < wheels.size() && w < 4; w++)
            {
                if (wheels[w]->HasContact())
                    wheelsInContact++;
            }
            float tractionFactor = (float)wheelsInContact / 4.0f;  // 0.0 to 1.0

            // Store traction for status bar display
            v->last_traction = tractionFactor;
            v->last_applied_force = 0.0f;  // Reset, will accumulate below

            // ========== MATCHBOX CAR PHYSICS ==========
            // Apply direct body force for acceleration - wheels are unpowered
            // F = m * a gives constant acceleration regardless of speed
            // Force is scaled by traction (wheels on ground)
            JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
            float speed = vel.Length();

            if (forward > 0.0f && speed < v->config.top_speed_ms && tractionFactor > 0.0f)
            {
                // Get vehicle's forward direction (Z axis in local space)
                JPH::Quat rot = bodyInterface.GetRotation(vimpl->bodyId);
                JPH::Vec3 forwardDir = rot.RotateAxisZ();

                // Apply acceleration force proportional to throttle AND traction
                float force = v->config.accel_force * forward * tractionFactor;
                bodyInterface.AddForce(vimpl->bodyId, forwardDir * force);
                v->last_applied_force = force;  // Store for debug display
            }
            else if (forward < 0.0f && tractionFactor > 0.0f)
            {
                // Reverse - apply force in opposite direction (scaled by traction)
                JPH::Quat rot = bodyInterface.GetRotation(vimpl->bodyId);
                JPH::Vec3 forwardDir = rot.RotateAxisZ();
                float force = v->config.accel_force * (-forward) * tractionFactor;
                bodyInterface.AddForce(vimpl->bodyId, -forwardDir * force);
                v->last_applied_force = -force;  // Negative for reverse
            }

            // Apply braking as reverse force (also scaled by traction)
            if (v->brake > 0.0f && speed > 0.1f && tractionFactor > 0.0f)
            {
                // Brake force opposes current velocity direction
                JPH::Vec3 velDir = vel.Normalized();
                float brakeForce = v->config.brake_force * v->brake * tractionFactor;
                bodyInterface.AddForce(vimpl->bodyId, -velDir * brakeForce);
                // Don't add brake to last_applied_force - keep it throttle-only
            }

            // Steering - use the vehicle constraint for wheel angles
            // (controller already obtained above for traction check)
            // Only pass steering to controller, no throttle (wheels unpowered)
            controller->SetDriverInput(0.0f, v->steering, 0.0f, v->brake);

            // ========== ACCELERATION TEST ==========
            // Key-triggered test (T key), auto-ends at 60 mph
            // Timer only starts when throttle is applied (forward > 0)
            if (v->accel_test_active)
            {
                BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();
                JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
                float speed = vel.Length();
                RVec3 pos = bodyInterface.GetPosition(vimpl->bodyId);

                // Start timing only when throttle is applied
                if (!v->accel_test_timing_started && forward > 0)
                {
                    v->accel_test_timing_started = true;
                    v->accel_test_start_pos = (::Vec3){(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ()};
                    v->accel_test_last_speed = speed;
                    printf("[ACCEL TEST] Timing started (throttle detected)\n");
                    fflush(stdout);
                }

                // Only count time once timing has started
                if (v->accel_test_timing_started)
                {
                    v->accel_test_elapsed += pw->step_size;

                    // Stop conditions: target speed (may be less than 60 for slow cars), collision, or timeout
                    float targetSpeed = v->accel_test_target_ms;
                    bool reachedTarget = speed >= targetSpeed;
                    bool collision = (v->accel_test_last_speed > 5.0f && speed < v->accel_test_last_speed * 0.7f);
                    bool timeout = v->accel_test_elapsed > 30.0f;

                    if (reachedTarget || collision || timeout)
                    {
                        float avgAccel = speed / v->accel_test_elapsed;
                        float speedMph = speed * 2.23694f;
                        float targetMph = targetSpeed * 2.23694f;

                        // Use vehicle's configured target (from PF/weight ratio)
                        float targetAccel = v->config.target_accel_ms2;
                        float target060 = v->config.target_0_60_seconds;
                        if (targetAccel <= 0.0f) {
                            targetAccel = 4.47f;  // Fallback: 10 mph/s
                            target060 = 6.0f;
                        }

                        // Scale factor for vehicles that can't reach 60 mph
                        // If testing 0-40 instead of 0-60, times should be 40/60 = 0.667x
                        const float SIXTY_MPH_MS = 26.82f;
                        float speedScale = targetSpeed / SIXTY_MPH_MS;

                        // Determine bucket range based on target 0-60 time
                        // Car Wars buckets with midpoint boundaries:
                        //   5 mph/s = 12s, 10 mph/s = 6s, 15 mph/s = 4s, 20 mph/s = 3s
                        // Midpoints: 9s (5/10), 5s (10/15), 3.5s (15/20)
                        float rangeMin, rangeMax;
                        const char* bucketName;
                        if (target060 >= 9.0f) {
                            // 5 mph/s bucket: 9s to 15s (midpoint to 1.25× target)
                            rangeMin = 9.0f;
                            rangeMax = 15.0f;
                            bucketName = "5 mph/s";
                        } else if (target060 >= 5.0f) {
                            // 10 mph/s bucket: 5s to 9s (midpoints)
                            rangeMin = 5.0f;
                            rangeMax = 9.0f;
                            bucketName = "10 mph/s";
                        } else if (target060 >= 3.5f) {
                            // 15 mph/s bucket: 3.5s to 5s (midpoints)
                            rangeMin = 3.5f;
                            rangeMax = 5.0f;
                            bucketName = "15 mph/s";
                        } else {
                            // 20 mph/s bucket: 2.5s to 3.5s
                            rangeMin = 2.5f;
                            rangeMax = 3.5f;
                            bucketName = "20 mph/s";
                        }

                        // Scale ranges for reduced target speed
                        rangeMin *= speedScale;
                        rangeMax *= speedScale;

                        // Calculate result - pass if within bucket range
                        float accelPercent = (avgAccel / targetAccel) * 100.0f;
                        bool passed = (v->accel_test_elapsed >= rangeMin && v->accel_test_elapsed <= rangeMax);

                        // Output with vehicle name and pass/fail
                        printf("\n[ACCEL TEST] %s\n", v->config.vehicle_name[0] ? v->config.vehicle_name : "Vehicle");
                        if (speedScale < 1.0f) {
                            printf("  0-%.0f: %.2fs (bucket: %s, range: %.1f-%.1fs) %s\n",
                                   targetMph, v->accel_test_elapsed, bucketName, rangeMin, rangeMax, passed ? "PASS" : "FAIL");
                            printf("  (scaled from 0-60 by %.0f%%)\n", speedScale * 100.0f);
                        } else {
                            printf("  0-60: %.2fs (bucket: %s, range: %.1f-%.1fs) %s\n",
                                   v->accel_test_elapsed, bucketName, rangeMin, rangeMax, passed ? "PASS" : "FAIL");
                        }
                        printf("  Avg Accel: %.2f m/s² (%.0f%% of target %.2f m/s²)\n",
                               avgAccel, accelPercent, targetAccel);
                        printf("  Final: %.0f mph\n", speedMph);
                        if (collision) printf("  (ended early - collision detected)\n");
                        if (timeout) printf("  (timeout - did not reach target speed)\n");
                        fflush(stdout);

                        v->accel_test_active = false;
                        v->accel_test_timing_started = false;
                    }

                    v->accel_test_last_speed = speed;
                }
            }
            // ========== END ACCELERATION TEST ==========

            // Keep vehicle awake when there's input
            if (forward != 0 || v->steering != 0 || v->brake != 0)
            {
                impl->physicsSystem->GetBodyInterface().ActivateBody(vimpl->bodyId);
            }
        }

        // Step physics
        impl->physicsSystem->Update(pw->step_size, 1, impl->tempAllocator, impl->jobSystem);

        // Update wheel states after stepping
        for (int i = 0; i < MAX_PHYSICS_VEHICLES; i++)
        {
            PhysicsVehicle* v = &pw->vehicles[i];
            if (!v->active || !v->impl) continue;

            auto* vimpl = v->impl;
            const Wheels& wheels = vimpl->constraint->GetWheels();

            for (size_t w = 0; w < wheels.size() && w < 4; w++)
            {
                const Wheel* wheel = wheels[w];
                RMat44 wheelTransform = vimpl->constraint->GetWheelWorldTransform(
                    (uint)w, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX());

                RVec3 pos = wheelTransform.GetTranslation();
                v->wheel_states[w].position.x = (float)pos.GetX();
                v->wheel_states[w].position.y = (float)pos.GetY();
                v->wheel_states[w].position.z = (float)pos.GetZ();

                v->wheel_states[w].rotation = wheel->GetRotationAngle();
                v->wheel_states[w].steer_angle = wheel->GetSteerAngle();
                v->wheel_states[w].suspension_compression = wheel->GetSuspensionLength() /
                    wheel->GetSettings()->mSuspensionMaxLength;

                // Extract rotation matrix
                Mat44 rot = wheelTransform.GetRotation();
                v->wheel_states[w].rot_matrix[0] = rot(0, 0);
                v->wheel_states[w].rot_matrix[1] = rot(1, 0);
                v->wheel_states[w].rot_matrix[2] = rot(2, 0);
                v->wheel_states[w].rot_matrix[3] = rot(0, 1);
                v->wheel_states[w].rot_matrix[4] = rot(1, 1);
                v->wheel_states[w].rot_matrix[5] = rot(2, 1);
                v->wheel_states[w].rot_matrix[6] = rot(0, 2);
                v->wheel_states[w].rot_matrix[7] = rot(1, 2);
                v->wheel_states[w].rot_matrix[8] = rot(2, 2);
                v->wheel_states[w].rot_matrix[9] = (float)pos.GetX();
                v->wheel_states[w].rot_matrix[10] = (float)pos.GetY();
                v->wheel_states[w].rot_matrix[11] = (float)pos.GetZ();
            }
        }

        pw->accumulator -= pw->step_size;
    }
}

void physics_set_ground(PhysicsWorld* pw, float y_level)
{
    if (!pw || !pw->impl) return;

    auto* impl = pw->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    // Remove old ground if exists
    if (!impl->groundBodyId.IsInvalid())
    {
        bodyInterface.RemoveBody(impl->groundBodyId);
        bodyInterface.DestroyBody(impl->groundBodyId);
    }

    impl->groundLevel = y_level;

    // Create ground plane as large box
    BoxShapeSettings groundShapeSettings(JPH::Vec3(500.0f, 1.0f, 500.0f));
    groundShapeSettings.SetEmbedded();
    ShapeSettings::ShapeResult groundShapeResult = groundShapeSettings.Create();
    ShapeRefC groundShape = groundShapeResult.Get();

    BodyCreationSettings groundSettings(groundShape, RVec3(0, y_level - 1.0f, 0),
        Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);

    Body* ground = bodyInterface.CreateBody(groundSettings);
    impl->groundBodyId = ground->GetID();
    bodyInterface.AddBody(impl->groundBodyId, EActivation::DontActivate);
}

void physics_add_box_obstacle(PhysicsWorld* pw, ::Vec3 pos, ::Vec3 size)
{
    if (!pw || !pw->impl) return;

    auto* impl = pw->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    BoxShapeSettings boxShapeSettings(JPH::Vec3(size.x * 0.5f, size.y * 0.5f, size.z * 0.5f));
    boxShapeSettings.SetEmbedded();
    ShapeSettings::ShapeResult boxShapeResult = boxShapeSettings.Create();
    ShapeRefC boxShape = boxShapeResult.Get();

    BodyCreationSettings boxSettings(boxShape, RVec3(pos.x, pos.y, pos.z),
        Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);

    Body* box = bodyInterface.CreateBody(boxSettings);
    bodyInterface.AddBody(box->GetID(), EActivation::DontActivate);
}

void physics_add_ramp_obstacle(PhysicsWorld* pw, ::Vec3 pos, ::Vec3 size, float rotation_y)
{
    if (!pw || !pw->impl) return;

    auto* impl = pw->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    // Create a wedge/ramp shape using convex hull
    // size.x = width, size.y = height (at high end), size.z = length
    // Low end at -Z, high end at +Z (before rotation)
    // Position is at the center-bottom of the ramp
    float hw = size.x * 0.5f;  // half width
    float h = size.y;          // full height
    float hl = size.z * 0.5f;  // half length

    // 6 vertices of a triangular prism (wedge)
    // Bottom face is rectangular, top face is a line at the high end
    Array<JPH::Vec3> vertices;
    vertices.reserve(6);
    // Bottom face (4 corners at y=0)
    vertices.push_back(JPH::Vec3(-hw, 0, -hl));  // Back-left bottom
    vertices.push_back(JPH::Vec3( hw, 0, -hl));  // Back-right bottom
    vertices.push_back(JPH::Vec3(-hw, 0,  hl));  // Front-left bottom
    vertices.push_back(JPH::Vec3( hw, 0,  hl));  // Front-right bottom
    // Top edge (2 corners at y=h, at front/high end)
    vertices.push_back(JPH::Vec3(-hw, h,  hl));  // Front-left top
    vertices.push_back(JPH::Vec3( hw, h,  hl));  // Front-right top

    ConvexHullShapeSettings rampShapeSettings(vertices.data(), (int)vertices.size());
    rampShapeSettings.SetEmbedded();
    ShapeSettings::ShapeResult rampShapeResult = rampShapeSettings.Create();
    if (!rampShapeResult.IsValid()) {
        std::cerr << "[Jolt] Failed to create ramp shape: " << rampShapeResult.GetError() << std::endl;
        return;
    }
    ShapeRefC rampShape = rampShapeResult.Get();

    // Create rotation quaternion from Y angle
    Quat rotation = Quat::sRotation(JPH::Vec3::sAxisY(), rotation_y);

    BodyCreationSettings rampSettings(rampShape, RVec3(pos.x, pos.y, pos.z),
        rotation, EMotionType::Static, Layers::NON_MOVING);

    Body* ramp = bodyInterface.CreateBody(rampSettings);
    bodyInterface.AddBody(ramp->GetID(), EActivation::DontActivate);
}

void physics_add_arena_walls(PhysicsWorld* pw, float arena_size, float wall_height, float wall_thickness)
{
    float half = arena_size * 0.5f;
    float ht = wall_thickness * 0.5f;
    float hh = wall_height * 0.5f;

    // North wall
    physics_add_box_obstacle(pw, (::Vec3){0, hh, half + ht}, (::Vec3){arena_size, wall_height, wall_thickness});
    // South wall
    physics_add_box_obstacle(pw, (::Vec3){0, hh, -half - ht}, (::Vec3){arena_size, wall_height, wall_thickness});
    // East wall
    physics_add_box_obstacle(pw, (::Vec3){half + ht, hh, 0}, (::Vec3){wall_thickness, wall_height, arena_size});
    // West wall
    physics_add_box_obstacle(pw, (::Vec3){-half - ht, hh, 0}, (::Vec3){wall_thickness, wall_height, arena_size});
}

int physics_create_vehicle(PhysicsWorld* pw, ::Vec3 position, float rotation_y, const VehicleConfig* config)
{
    if (!pw || !pw->impl) return -1;

    // Find free slot
    int slot = -1;
    for (int i = 0; i < MAX_PHYSICS_VEHICLES; i++)
    {
        if (!pw->vehicles[i].active)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0) return -1;

    auto* impl = pw->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    PhysicsVehicle* v = &pw->vehicles[slot];
    v->active = true;
    v->impl = new PhysicsVehicleImpl();
    v->config = *config;
    v->spawn_position = position;
    v->spawn_rotation = rotation_y;
    v->steering = 0;
    v->throttle = 0;
    v->reverse = 0;
    v->brake = 0;
    v->engine_rpm = 0;
    v->reverse_rpm = 0;

    // Initialize acceleration test state
    v->accel_test_active = false;
    v->accel_test_timing_started = false;
    v->accel_test_elapsed = 0.0f;
    v->accel_test_print_timer = 0.0f;
    v->accel_test_start_pos = (::Vec3){0, 0, 0};
    v->accel_test_last_speed = 0.0f;

    auto* vimpl = v->impl;

    // Vehicle dimensions
    float halfLength = config->chassis_length * 0.5f;
    float halfWidth = config->chassis_width * 0.5f;
    float halfHeight = config->chassis_height * 0.5f;

    // Create car body shape with center of mass from config
    // CoM offset is normalized (-1 to 1 range per axis), multiply by half-dimensions
    // Lower Y = more stable, less likely to flip during aggressive turns
    JPH::Vec3 comOffset(
        config->center_of_mass.x * halfWidth,
        config->center_of_mass.y * halfHeight,
        config->center_of_mass.z * halfLength
    );
    RefConst<Shape> carShape = OffsetCenterOfMassShapeSettings(
        comOffset,
        new BoxShape(JPH::Vec3(halfWidth, halfHeight, halfLength))
    ).Create().Get();

    // Create rotation quaternion from Y angle
    Quat rotation = Quat::sRotation(JPH::Vec3::sAxisY(), rotation_y);

    BodyCreationSettings carBodySettings(carShape,
        RVec3(position.x, position.y, position.z), rotation,
        EMotionType::Dynamic, Layers::MOVING);
    carBodySettings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    carBodySettings.mMassPropertiesOverride.mMass = config->chassis_mass;

    Body* carBody = bodyInterface.CreateBody(carBodySettings);
    vimpl->bodyId = carBody->GetID();
    bodyInterface.AddBody(vimpl->bodyId, EActivation::Activate);

    // Create vehicle constraint
    VehicleConstraintSettings vehicleSettings;
    vehicleSettings.mUp = JPH::Vec3::sAxisY();
    vehicleSettings.mForward = JPH::Vec3::sAxisZ();

    // Create wheel settings
    for (int i = 0; i < 4; i++)
    {
        WheelSettingsWV* ws = new WheelSettingsWV();

        // Use per-wheel positions or compute from legacy values
        if (config->use_per_wheel_config)
        {
            ws->mPosition = JPH::Vec3(config->wheel_positions[i].x,
                                 config->wheel_positions[i].y,
                                 config->wheel_positions[i].z);
            ws->mRadius = config->wheel_radii[i];
            ws->mWidth = config->wheel_widths[i];
        }
        else
        {
            // Legacy position calculation
            float xSign = (i == WHEEL_FL || i == WHEEL_RL) ? -1.0f : 1.0f;
            float zSign = (i == WHEEL_FL || i == WHEEL_FR) ? 1.0f : -1.0f;
            ws->mPosition = JPH::Vec3(
                xSign * config->track_width * 0.5f,
                -config->wheel_mount_height,
                zSign * config->wheelbase * 0.5f
            );
            ws->mRadius = config->wheel_radius;
            ws->mWidth = config->wheel_width;
        }

        ws->mSuspensionDirection = JPH::Vec3(0, -1, 0);
        ws->mSteeringAxis = JPH::Vec3(0, 1, 0);
        ws->mWheelUp = JPH::Vec3(0, 1, 0);
        ws->mWheelForward = JPH::Vec3(0, 0, 1);

        // Suspension settings
        float frequency = config->use_per_wheel_config ?
            config->wheel_suspension_frequency[i] : config->suspension_frequency;
        float damping = config->use_per_wheel_config ?
            config->wheel_suspension_damping[i] : config->suspension_damping;
        float travel = config->use_per_wheel_config ?
            config->wheel_suspension_travel[i] : config->suspension_travel;

        // Default values if not specified
        if (frequency <= 0) frequency = 1.5f;
        if (damping <= 0) damping = 0.5f;
        if (travel <= 0) travel = 0.3f;

        ws->mSuspensionMinLength = 0.05f;
        ws->mSuspensionMaxLength = travel;
        ws->mSuspensionSpring.mFrequency = frequency;
        ws->mSuspensionSpring.mDamping = damping;

        // Steering
        bool canSteer = config->use_per_wheel_config ?
            config->wheel_steering[i] : (i == WHEEL_FL || i == WHEEL_FR);
        float maxSteer = config->use_per_wheel_config ?
            config->wheel_steer_angles[i] : config->max_steer_angle;
        ws->mMaxSteerAngle = canSteer ? maxSteer : 0.0f;

        // Brakes on rear wheels
        ws->mMaxHandBrakeTorque = (i == WHEEL_RL || i == WHEEL_RR) ? 4000.0f : 0.0f;

        // Wheel inertia - affects how easily wheels spin
        // I = 0.5 * m * r^2 for a solid cylinder
        float wheelMass = config->use_per_wheel_config ?
            config->wheel_masses[i] : config->wheel_mass;
        if (wheelMass <= 0) wheelMass = 15.0f;  // Default 15kg
        ws->mInertia = 0.5f * wheelMass * ws->mRadius * ws->mRadius;

        // Tire friction - scale friction curves by mu value
        float tireMu = config->tire_friction;
        if (tireMu <= 0) tireMu = 1.0f;  // Default friction coefficient
        // Jolt's friction curves: slip ratio/angle -> friction multiplier
        // Scale by our mu to get actual friction
        // IMPORTANT: Curve must extend to 1.0 (100% slip) for wheelspin scenarios
        ws->mLongitudinalFriction.mPoints.clear();
        ws->mLongitudinalFriction.mPoints.push_back({0.0f, 0.0f});
        ws->mLongitudinalFriction.mPoints.push_back({0.06f, tireMu * 1.2f});  // Peak at 6% slip
        ws->mLongitudinalFriction.mPoints.push_back({0.2f, tireMu * 1.0f});   // Transition
        ws->mLongitudinalFriction.mPoints.push_back({1.0f, tireMu * 0.8f});   // Full wheelspin (80% of mu)

        // RAILS MODE: High lateral friction - car grips through turns without flipping
        // This is the "default" behavior; disasters will temporarily reduce this
        const float railsMultiplier = 3.0f;  // 3x normal grip (5x caused rollovers)
        ws->mLateralFriction.mPoints.clear();
        ws->mLateralFriction.mPoints.push_back({0.0f, 0.0f});
        ws->mLateralFriction.mPoints.push_back({3.0f, tireMu * railsMultiplier});   // Peak grip
        ws->mLateralFriction.mPoints.push_back({20.0f, tireMu * railsMultiplier});  // Still gripping
        ws->mLateralFriction.mPoints.push_back({90.0f, tireMu * railsMultiplier * 0.9f});  // Even sideways, high grip

        vehicleSettings.mWheels.push_back(ws);
    }

    // Controller settings - minimal setup since wheels are unpowered
    // We still use WheeledVehicleController for wheel physics (steering, suspension)
    // but acceleration comes from direct body force, not engine
    WheeledVehicleControllerSettings* controllerSettings = new WheeledVehicleControllerSettings();

    // Minimal engine - just enough for Jolt to not complain
    // The engine isn't used for propulsion - that's done via direct body force
    controllerSettings->mEngine.mMaxTorque = 1.0f;  // Minimal - not used
    controllerSettings->mEngine.mMinRPM = 0.0f;
    controllerSettings->mEngine.mMaxRPM = 1.0f;

    // Single gear, not used for propulsion
    controllerSettings->mTransmission.mMode = ETransmissionMode::Auto;
    controllerSettings->mTransmission.mGearRatios = { 1.0f };
    controllerSettings->mTransmission.mReverseGearRatios = { -1.0f };

    // No differentials needed - wheels are unpowered
    // But Jolt requires at least one, so add a dummy
    controllerSettings->mDifferentials.resize(1);
    controllerSettings->mDifferentials[0].mLeftWheel = WHEEL_RL;
    controllerSettings->mDifferentials[0].mRightWheel = WHEEL_RR;
    controllerSettings->mDifferentials[0].mDifferentialRatio = 1.0f;

    vehicleSettings.mController = controllerSettings;

    // Log matchbox car physics info
    float tireMu = config->tire_friction > 0 ? config->tire_friction : 1.0f;
    printf("Matchbox physics: mass=%.0fkg, accel_force=%.0fN, brake=%.0fN, mu=%.1f\n",
           config->chassis_mass, config->accel_force, config->brake_force, tireMu);
    printf("  Target: %.1fs 0-60 (%.2f m/s²), top speed=%.0f mph\n",
           config->target_0_60_seconds, config->target_accel_ms2,
           config->top_speed_ms * 2.237f);

    // Create constraint
    vimpl->constraint = new VehicleConstraint(*carBody, vehicleSettings);

    // Collision tester
    RefConst<VehicleCollisionTester> collisionTester = new VehicleCollisionTesterCastCylinder(
        Layers::MOVING, 0.05f);
    vimpl->constraint->SetVehicleCollisionTester(collisionTester);

    impl->physicsSystem->AddConstraint(vimpl->constraint);
    impl->physicsSystem->AddStepListener(vimpl->constraint);

    pw->vehicle_count++;

    std::cout << "[Jolt] Created vehicle " << slot << " at ("
              << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;

    return slot;
}

void physics_destroy_vehicle(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;

    if (vimpl && vimpl->constraint)
    {
        impl->physicsSystem->RemoveStepListener(vimpl->constraint);
        impl->physicsSystem->RemoveConstraint(vimpl->constraint);
    }

    if (vimpl && !vimpl->bodyId.IsInvalid())
    {
        BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();
        bodyInterface.RemoveBody(vimpl->bodyId);
        bodyInterface.DestroyBody(vimpl->bodyId);
    }

    delete vimpl;
    v->impl = nullptr;
    v->active = false;
    pw->vehicle_count--;
}

void physics_vehicle_set_steering(PhysicsWorld* pw, int vehicle_id, float steering)
{
    if (!pw || vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->steering = steering;
}

void physics_vehicle_set_throttle(PhysicsWorld* pw, int vehicle_id, float throttle)
{
    if (!pw || vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->throttle = throttle;
}

void physics_vehicle_set_reverse(PhysicsWorld* pw, int vehicle_id, float reverse)
{
    if (!pw || vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->reverse = reverse;
}

void physics_vehicle_set_brake(PhysicsWorld* pw, int vehicle_id, float brake)
{
    if (!pw || vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;
    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;
    v->brake = brake;
}

void physics_vehicle_respawn(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    Quat rotation = Quat::sRotation(JPH::Vec3::sAxisY(), v->spawn_rotation);
    bodyInterface.SetPositionAndRotation(vimpl->bodyId,
        RVec3(v->spawn_position.x, v->spawn_position.y, v->spawn_position.z),
        rotation, EActivation::Activate);
    bodyInterface.SetLinearVelocity(vimpl->bodyId, JPH::Vec3::sZero());
    bodyInterface.SetAngularVelocity(vimpl->bodyId, JPH::Vec3::sZero());

    v->steering = 0;
    v->throttle = 0;
    v->reverse = 0;
    v->brake = 0;
    v->engine_rpm = 0;
    v->reverse_rpm = 0;

    // Reset acceleration test for new run
    v->accel_test_active = false;
    v->accel_test_timing_started = false;
    v->accel_test_elapsed = 0.0f;
    v->accel_test_print_timer = 0.0f;
}

void physics_vehicle_start_accel_test(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    // Get current position and velocity
    RVec3 pos = bodyInterface.GetPosition(vimpl->bodyId);
    JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
    float speed = vel.Length();

    // If already running, stop and report
    if (v->accel_test_active) {
        printf("[ACCEL TEST] Test cancelled\n");
        v->accel_test_active = false;
        return;
    }

    // Start the test
    v->accel_test_active = true;
    v->accel_test_timing_started = false;  // Wait for throttle before timing
    v->accel_test_elapsed = 0.0f;
    v->accel_test_print_timer = 0.0f;
    v->accel_test_start_pos = (::Vec3){(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ()};
    v->accel_test_last_speed = speed;

    // Set target to min(60 mph, top_speed) - some vehicles can't reach 60
    const float SIXTY_MPH_MS = 26.82f;
    float top_speed = v->config.top_speed_ms;
    if (top_speed > 0.0f && top_speed < SIXTY_MPH_MS) {
        v->accel_test_target_ms = top_speed;
        printf("[ACCEL TEST] Started - floor it! (target: %.0f mph / %.2f m/s - top speed limited)\n",
               top_speed * 2.237f, top_speed);
    } else {
        v->accel_test_target_ms = SIXTY_MPH_MS;
        printf("[ACCEL TEST] Started - floor it! (target: 60 mph / 26.82 m/s)\n");
    }
    fflush(stdout);
}

void physics_vehicle_get_position(PhysicsWorld* pw, int vehicle_id, ::Vec3* pos)
{
    if (!pw || !pw->impl || !pos) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    RVec3 p = bodyInterface.GetPosition(vimpl->bodyId);
    pos->x = (float)p.GetX();
    pos->y = (float)p.GetY();
    pos->z = (float)p.GetZ();
}

void physics_vehicle_get_rotation(PhysicsWorld* pw, int vehicle_id, float* rotation_y)
{
    if (!pw || !pw->impl || !rotation_y) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    Quat q = bodyInterface.GetRotation(vimpl->bodyId);

    // Extract Y rotation (yaw) from quaternion
    // Using atan2 for proper angle extraction
    float siny_cosp = 2.0f * (q.GetW() * q.GetY() + q.GetX() * q.GetZ());
    float cosy_cosp = 1.0f - 2.0f * (q.GetY() * q.GetY() + q.GetZ() * q.GetZ());
    *rotation_y = atan2f(siny_cosp, cosy_cosp);
}

void physics_vehicle_get_rotation_matrix(PhysicsWorld* pw, int vehicle_id, float* rot_matrix)
{
    if (!pw || !pw->impl || !rot_matrix) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    RMat44 transform = bodyInterface.GetWorldTransform(vimpl->bodyId);
    Mat44 rot = transform.GetRotation();

    // Return as 3x3 row-major
    rot_matrix[0] = rot(0, 0); rot_matrix[1] = rot(0, 1); rot_matrix[2] = rot(0, 2);
    rot_matrix[3] = rot(1, 0); rot_matrix[4] = rot(1, 1); rot_matrix[5] = rot(1, 2);
    rot_matrix[6] = rot(2, 0); rot_matrix[7] = rot(2, 1); rot_matrix[8] = rot(2, 2);
}

void physics_vehicle_get_velocity(PhysicsWorld* pw, int vehicle_id, float* speed_ms)
{
    if (!pw || !pw->impl || !speed_ms) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
    *speed_ms = vel.Length();
}

void physics_vehicle_get_lateral_velocity(PhysicsWorld* pw, int vehicle_id, float* lateral_ms)
{
    if (!pw || !pw->impl || !lateral_ms) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    // Get velocity vector
    JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);

    // Get car's rotation quaternion
    Quat q = bodyInterface.GetRotation(vimpl->bodyId);

    // Get the car's right direction (local +X is right)
    JPH::Vec3 localRight(1, 0, 0);
    JPH::Vec3 worldRight = q * localRight;

    // Lateral velocity = velocity dot right vector (horizontal components only)
    // This gives how fast we're moving sideways relative to the car's facing
    float lateral = vel.GetX() * worldRight.GetX() + vel.GetZ() * worldRight.GetZ();

    // Return absolute lateral velocity in m/s
    *lateral_ms = fabsf(lateral);
}

void physics_vehicle_get_wheel_states(PhysicsWorld* pw, int vehicle_id, WheelState* wheels)
{
    if (!pw || !wheels) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    memcpy(wheels, v->wheel_states, sizeof(WheelState) * 4);
}

void physics_debug_draw(PhysicsWorld* pw, struct LineRenderer* lr)
{
    if (!pw || !pw->impl || !lr) return;

    // Draw ground grid
    float gridSize = 50.0f;
    float gridStep = 5.0f;
    float y = pw->impl->groundLevel + 0.01f;

    for (float x = -gridSize; x <= gridSize; x += gridStep)
    {
        line_renderer_draw_line(lr,
            (::Vec3){x, y, -gridSize}, (::Vec3){x, y, gridSize},
            (::Vec3){0.3f, 0.5f, 0.3f}, 1.0f);
        line_renderer_draw_line(lr,
            (::Vec3){-gridSize, y, x}, (::Vec3){gridSize, y, x},
            (::Vec3){0.3f, 0.5f, 0.3f}, 1.0f);
    }

    // Draw vehicles
    for (int i = 0; i < MAX_PHYSICS_VEHICLES; i++)
    {
        PhysicsVehicle* v = &pw->vehicles[i];
        if (!v->active || !v->impl) continue;

        auto* vimpl = v->impl;
        auto* impl = pw->impl;
        BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

        RMat44 transform = bodyInterface.GetWorldTransform(vimpl->bodyId);
        RVec3 pos = transform.GetTranslation();

        // Draw chassis box outline
        float hw = v->config.chassis_width * 0.5f;
        float hh = v->config.chassis_height * 0.5f;
        float hl = v->config.chassis_length * 0.5f;

        ::Vec3 corners[8];
        ::Vec3 localCorners[8] = {
            {-hw, -hh, -hl}, {hw, -hh, -hl}, {hw, hh, -hl}, {-hw, hh, -hl},
            {-hw, -hh, hl}, {hw, -hh, hl}, {hw, hh, hl}, {-hw, hh, hl}
        };

        for (int c = 0; c < 8; c++)
        {
            ::Vec3 local = localCorners[c];
            RVec3 world = transform * JPH::Vec3(local.x, local.y, local.z);
            corners[c] = (::Vec3){(float)world.GetX(), (float)world.GetY(), (float)world.GetZ()};
        }

        ::Vec3 red = {0.8f, 0.2f, 0.2f};
        // Bottom
        line_renderer_draw_line(lr, corners[0], corners[1], red, 1.0f);
        line_renderer_draw_line(lr, corners[1], corners[2], red, 1.0f);
        line_renderer_draw_line(lr, corners[2], corners[3], red, 1.0f);
        line_renderer_draw_line(lr, corners[3], corners[0], red, 1.0f);
        // Top
        line_renderer_draw_line(lr, corners[4], corners[5], red, 1.0f);
        line_renderer_draw_line(lr, corners[5], corners[6], red, 1.0f);
        line_renderer_draw_line(lr, corners[6], corners[7], red, 1.0f);
        line_renderer_draw_line(lr, corners[7], corners[4], red, 1.0f);
        // Verticals
        line_renderer_draw_line(lr, corners[0], corners[4], red, 1.0f);
        line_renderer_draw_line(lr, corners[1], corners[5], red, 1.0f);
        line_renderer_draw_line(lr, corners[2], corners[6], red, 1.0f);
        line_renderer_draw_line(lr, corners[3], corners[7], red, 1.0f);

        // Draw wheels - use stored wheel rotation matrix from Jolt
        // POC draws cylinder in local X-Z plane with Y as axle direction
        ::Vec3 blue = {0.2f, 0.2f, 0.8f};
        for (int w = 0; w < 4; w++)
        {
            float radius = v->config.use_per_wheel_config ?
                v->config.wheel_radii[w] : v->config.wheel_radius;

            // Get wheel world position
            ::Vec3 wpos = v->wheel_states[w].position;

            // Get wheel orientation from stored rotation matrix
            // rot_matrix layout: [0-2]=col0(X/right), [3-5]=col1(Y/axle), [6-8]=col2(Z/forward)
            float* rm = v->wheel_states[w].rot_matrix;

            // Wheel circle is in local X-Z plane (perpendicular to Y axle)
            // X column = horizontal extent of circle
            ::Vec3 wheelX = {rm[0], rm[1], rm[2]};
            // Z column = vertical extent of circle
            ::Vec3 wheelZ = {rm[6], rm[7], rm[8]};

            // Draw circle: local p = (cos*r, 0, sin*r) transformed to world
            // world p = pos + cos*r*X_col + sin*r*Z_col
            const int segments = 16;
            for (int s = 0; s < segments; s++)
            {
                float a1 = (float)s / segments * 2.0f * 3.14159f;
                float a2 = (float)(s + 1) / segments * 2.0f * 3.14159f;

                ::Vec3 p1 = {
                    wpos.x + cosf(a1) * radius * wheelX.x + sinf(a1) * radius * wheelZ.x,
                    wpos.y + cosf(a1) * radius * wheelX.y + sinf(a1) * radius * wheelZ.y,
                    wpos.z + cosf(a1) * radius * wheelX.z + sinf(a1) * radius * wheelZ.z
                };
                ::Vec3 p2 = {
                    wpos.x + cosf(a2) * radius * wheelX.x + sinf(a2) * radius * wheelZ.x,
                    wpos.y + cosf(a2) * radius * wheelX.y + sinf(a2) * radius * wheelZ.y,
                    wpos.z + cosf(a2) * radius * wheelX.z + sinf(a2) * radius * wheelZ.z
                };

                line_renderer_draw_line(lr, p1, p2, blue, 1.0f);
            }
        }
    }
}

// ========== CRUISE CONTROL ==========

// Helper: convert m/s to mph
static float ms_to_mph(float ms) {
    return ms * 2.23694f;
}

// Helper: convert mph to m/s
static float mph_to_ms(float mph) {
    return mph / 2.23694f;
}

void physics_vehicle_cruise_hold(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    // Get current speed
    JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
    float speed = vel.Length();

    v->cruise_enabled = true;
    v->cruise_target_ms = speed;

    printf("Cruise: HOLD at %.0f mph (%.1f m/s)\n", ms_to_mph(speed), speed);
    fflush(stdout);
}

void physics_vehicle_cruise_set(PhysicsWorld* pw, int vehicle_id, float target_ms)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    // Clamp to top speed
    float clamped = target_ms;
    if (v->config.top_speed_ms > 0 && clamped > v->config.top_speed_ms) {
        clamped = v->config.top_speed_ms;
    }

    v->cruise_enabled = true;
    v->cruise_target_ms = clamped;

    printf("Cruise: SET to %.0f mph (%.1f m/s)\n", ms_to_mph(clamped), clamped);
    fflush(stdout);
}

void physics_vehicle_cruise_snap_up(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    // Use target speed if cruise already active, otherwise use current speed
    float base_mph;
    if (v->cruise_enabled && v->cruise_target_ms > 0) {
        base_mph = ms_to_mph(v->cruise_target_ms);
    } else {
        JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
        base_mph = ms_to_mph(vel.Length());
    }

    // Snap UP to next 10 mph increment (33 -> 40, 40 -> 50)
    float next_mph = ceilf((base_mph + 0.5f) / 10.0f) * 10.0f;

    // Clamp to top speed
    float top_mph = ms_to_mph(v->config.top_speed_ms);
    if (next_mph > top_mph && top_mph > 0) {
        next_mph = top_mph;
    }

    v->cruise_enabled = true;
    v->cruise_target_ms = mph_to_ms(next_mph);

    printf("Cruise: UP to %.0f mph (was %.0f mph)\n", next_mph, base_mph);
    fflush(stdout);
}

void physics_vehicle_cruise_snap_down(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active || !v->impl) return;

    auto* impl = pw->impl;
    auto* vimpl = v->impl;
    BodyInterface& bodyInterface = impl->physicsSystem->GetBodyInterface();

    // Use target speed if cruise already active, otherwise use current speed
    float base_mph;
    if (v->cruise_enabled && v->cruise_target_ms > 0) {
        base_mph = ms_to_mph(v->cruise_target_ms);
    } else {
        JPH::Vec3 vel = bodyInterface.GetLinearVelocity(vimpl->bodyId);
        base_mph = ms_to_mph(vel.Length());
    }

    // Snap DOWN to previous 10 mph increment (33 -> 30, 30 -> 20)
    float prev_mph = floorf((base_mph - 0.5f) / 10.0f) * 10.0f;
    if (prev_mph < 0) prev_mph = 0;

    v->cruise_enabled = true;
    v->cruise_target_ms = mph_to_ms(prev_mph);

    printf("Cruise: DOWN to %.0f mph (was %.0f mph)\n", prev_mph, base_mph);
    fflush(stdout);
}

void physics_vehicle_cruise_cancel(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    if (v->cruise_enabled) {
        v->cruise_enabled = false;
        v->cruise_target_ms = 0;
        printf("Cruise: OFF\n");
        fflush(stdout);
    }
}

bool physics_vehicle_cruise_active(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return false;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return false;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return false;

    return v->cruise_enabled;
}

float physics_vehicle_cruise_target(PhysicsWorld* pw, int vehicle_id)
{
    if (!pw || !pw->impl) return 0;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return 0;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return 0;

    return v->cruise_target_ms;
}

void physics_vehicle_get_traction_info(PhysicsWorld* pw, int vehicle_id, float* force_n, float* traction)
{
    if (!pw || !pw->impl) return;
    if (vehicle_id < 0 || vehicle_id >= MAX_PHYSICS_VEHICLES) return;

    PhysicsVehicle* v = &pw->vehicles[vehicle_id];
    if (!v->active) return;

    if (force_n) *force_n = v->last_applied_force;
    if (traction) *traction = v->last_traction;
}

} // extern "C"
