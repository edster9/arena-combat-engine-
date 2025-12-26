#include "entity.h"
#include <string.h>
#include <math.h>
#include <float.h>

// Team colors
static Vec3 team_colors[TEAM_COUNT] = {
    {0.8f, 0.2f, 0.15f},   // Red
    {0.15f, 0.3f, 0.8f},   // Blue
    {0.9f, 0.75f, 0.1f},   // Yellow
    {0.2f, 0.7f, 0.3f}     // Green
};

static Vec3 team_highlight_colors[TEAM_COUNT] = {
    {1.0f, 0.4f, 0.35f},   // Red highlight
    {0.4f, 0.55f, 1.0f},   // Blue highlight
    {1.0f, 0.9f, 0.4f},    // Yellow highlight
    {0.45f, 0.95f, 0.55f}  // Green highlight
};

Vec3 team_get_color(Team team) {
    if (team >= 0 && team < TEAM_COUNT) {
        return team_colors[team];
    }
    return vec3(0.5f, 0.5f, 0.5f);  // Grey fallback
}

Vec3 team_get_highlight_color(Team team) {
    if (team >= 0 && team < TEAM_COUNT) {
        return team_highlight_colors[team];
    }
    return vec3(0.8f, 0.8f, 0.8f);  // Light grey fallback
}

void entity_manager_init(EntityManager* em) {
    memset(em, 0, sizeof(EntityManager));
    em->selected_id = -1;
    em->next_id = 1;  // Start IDs at 1 (0 reserved for "none")
}

Entity* entity_manager_create(EntityManager* em, EntityType type, Team team) {
    if (em->count >= MAX_ENTITIES) {
        return NULL;
    }

    Entity* e = &em->entities[em->count++];
    memset(e, 0, sizeof(Entity));

    e->id = em->next_id++;
    e->type = type;
    e->team = team;
    e->active = true;
    e->selected = false;
    e->scale = 1.0f;
    e->position = vec3_zero();
    e->rotation_y = 0.0f;

    // Default bounds for vehicles (will be set properly when mesh is known)
    e->bounds_half = vec3(2.25f, 0.7f, 1.0f);  // Half of car dimensions

    return e;
}

Entity* entity_manager_get_by_id(EntityManager* em, int id) {
    for (int i = 0; i < em->count; i++) {
        if (em->entities[i].id == id && em->entities[i].active) {
            return &em->entities[i];
        }
    }
    return NULL;
}

void entity_manager_remove(EntityManager* em, int id) {
    for (int i = 0; i < em->count; i++) {
        if (em->entities[i].id == id) {
            em->entities[i].active = false;
            if (em->selected_id == id) {
                em->selected_id = -1;
            }
            return;
        }
    }
}

void entity_manager_select(EntityManager* em, int id) {
    // Deselect previous
    if (em->selected_id >= 0) {
        Entity* prev = entity_manager_get_by_id(em, em->selected_id);
        if (prev) prev->selected = false;
    }

    // Select new
    em->selected_id = id;
    if (id >= 0) {
        Entity* e = entity_manager_get_by_id(em, id);
        if (e) e->selected = true;
    }
}

void entity_manager_deselect_all(EntityManager* em) {
    entity_manager_select(em, -1);
}

Entity* entity_manager_get_selected(EntityManager* em) {
    if (em->selected_id < 0) return NULL;
    return entity_manager_get_by_id(em, em->selected_id);
}

// Ray-AABB intersection test
// Returns distance along ray, or -1 if no hit
static float ray_aabb_intersect(Vec3 ray_origin, Vec3 ray_dir, Vec3 box_min, Vec3 box_max) {
    float tmin = -FLT_MAX;
    float tmax = FLT_MAX;

    // X slab
    if (fabsf(ray_dir.x) > 0.0001f) {
        float t1 = (box_min.x - ray_origin.x) / ray_dir.x;
        float t2 = (box_max.x - ray_origin.x) / ray_dir.x;
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
        tmin = fmaxf(tmin, t1);
        tmax = fminf(tmax, t2);
    } else if (ray_origin.x < box_min.x || ray_origin.x > box_max.x) {
        return -1.0f;
    }

    // Y slab
    if (fabsf(ray_dir.y) > 0.0001f) {
        float t1 = (box_min.y - ray_origin.y) / ray_dir.y;
        float t2 = (box_max.y - ray_origin.y) / ray_dir.y;
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
        tmin = fmaxf(tmin, t1);
        tmax = fminf(tmax, t2);
    } else if (ray_origin.y < box_min.y || ray_origin.y > box_max.y) {
        return -1.0f;
    }

    // Z slab
    if (fabsf(ray_dir.z) > 0.0001f) {
        float t1 = (box_min.z - ray_origin.z) / ray_dir.z;
        float t2 = (box_max.z - ray_origin.z) / ray_dir.z;
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
        tmin = fmaxf(tmin, t1);
        tmax = fminf(tmax, t2);
    } else if (ray_origin.z < box_min.z || ray_origin.z > box_max.z) {
        return -1.0f;
    }

    if (tmax < tmin || tmax < 0.0f) {
        return -1.0f;
    }

    return tmin > 0.0f ? tmin : tmax;
}

int entity_manager_pick(EntityManager* em, Vec3 ray_origin, Vec3 ray_dir) {
    int closest_id = -1;
    float closest_dist = FLT_MAX;

    for (int i = 0; i < em->count; i++) {
        Entity* e = &em->entities[i];
        if (!e->active) continue;

        // Build AABB from entity position and bounds
        Vec3 box_min = vec3(
            e->position.x - e->bounds_half.x * e->scale,
            e->position.y,
            e->position.z - e->bounds_half.z * e->scale
        );
        Vec3 box_max = vec3(
            e->position.x + e->bounds_half.x * e->scale,
            e->position.y + e->bounds_half.y * 2.0f * e->scale,
            e->position.z + e->bounds_half.z * e->scale
        );

        float dist = ray_aabb_intersect(ray_origin, ray_dir, box_min, box_max);
        if (dist >= 0.0f && dist < closest_dist) {
            closest_dist = dist;
            closest_id = e->id;
        }
    }

    return closest_id;
}
