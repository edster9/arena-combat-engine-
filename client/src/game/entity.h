#ifndef ENTITY_H
#define ENTITY_H

#include "../math/vec3.h"
#include "../render/obj_loader.h"
#include <stdbool.h>

// Team identifiers
typedef enum {
    TEAM_RED = 0,
    TEAM_BLUE,
    TEAM_YELLOW,
    TEAM_GREEN,
    TEAM_COUNT
} Team;

// Entity types
typedef enum {
    ENTITY_VEHICLE = 0,
    ENTITY_OBSTACLE,
    ENTITY_TYPE_COUNT
} EntityType;

// Entity structure
typedef struct {
    // Transform
    Vec3 position;
    float rotation_y;   // Radians
    float scale;

    // Identity
    EntityType type;
    Team team;
    int id;             // Unique identifier

    // State
    bool active;
    bool selected;

    // Bounding (for picking) - axis-aligned after scale, centered at position
    Vec3 bounds_half;   // Half-extents of bounding box
} Entity;

// Entity manager
#define MAX_ENTITIES 64

typedef struct {
    Entity entities[MAX_ENTITIES];
    int count;
    int next_id;
    int selected_id;    // -1 if none selected
} EntityManager;

// Team colors
Vec3 team_get_color(Team team);
Vec3 team_get_highlight_color(Team team);  // Brighter version for selection

// Entity manager functions
void entity_manager_init(EntityManager* em);
Entity* entity_manager_create(EntityManager* em, EntityType type, Team team);
Entity* entity_manager_get_by_id(EntityManager* em, int id);
void entity_manager_remove(EntityManager* em, int id);

// Selection
void entity_manager_select(EntityManager* em, int id);
void entity_manager_deselect_all(EntityManager* em);
Entity* entity_manager_get_selected(EntityManager* em);

// Ray picking - returns entity ID or -1 if nothing hit
int entity_manager_pick(EntityManager* em, Vec3 ray_origin, Vec3 ray_dir);

#endif // ENTITY_H
