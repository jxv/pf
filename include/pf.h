#ifndef PF_H
#define PF_H

#include <stdlib.h>
#include <stdbool.h>
#include <ml.h>

typedef struct pf_aabb {
    v2f min;
    v2f max;
} pf_aabb_t;

typedef enum pf_shape_tag {
    PF_SHAPE_RECT,
    PF_SHAPE_CIRCLE,
} pf_shape_tag_t;

typedef struct pf_shape {
    pf_shape_tag_t tag;
    union {
        v2f radii;
        float radius;
    };
} pf_shape_t;

typedef enum pf_mode {
    PF_MODE_STATIC,
    PF_MODE_DYNAMIC,
} pf_mode_t;

typedef enum pf_group {
    PF_GROUP_PLATFORM,
    PF_GROUP_CHARACTER,
    PF_GROUP_OBJECT,
} pf_group_t;

typedef enum pf_platform {
    PF_PLATFORM_NO_WAY,     // Solid platform
    PF_PLATFORM_ONE_WAY,    // Jump-up-onto only platform
    PF_PLATFORM_TWO_WAY,    // Jump-up and jump-down platform
} pf_platform_t;

typedef enum pf_dir {
    PF_DIR_U,
    PF_DIR_D,
    PF_DIR_L,
    PF_DIR_R,
} pf_dir_t;

typedef struct pf_force {
    v2f impulse;    // Strength of the force
    v2f decay;      // Percentage to retain each iteration
    v2f cap;        // Max/min speed capacity
} pf_force_t;

typedef struct pf_gravity {
    pf_dir_t dir;
    float accel;
    float cap;
    float vel;
} pf_gravity_t;

typedef struct pf_body {
    pf_mode_t mode;
    pf_group_t group;
    pf_shape_t shape;
    v2f pos;
    struct pf_body *parent;
    v2f dpos;               // Change of position
    pf_force_t in;          // Internal/automonous
    pf_force_t ex;          // External
    pf_gravity_t gravity;
    float mass;
    float inverse_mass;
    float static_friction;
    float dynamic_friction;
    float restitution;
} pf_body_t;

typedef struct pf_manifold {
    v2f normal;
    float penetration;
    float mixed_restitution;
    float dynamic_friction;
    float static_friction;
} pf_manifold_t;

bool pf_intersect(const pf_aabb_t *a, const pf_aabb_t *b);
bool pf_inside(const v2f *a, const pf_aabb_t *b);
v2f pf_aabb_pos(const pf_aabb_t *a);
pf_aabb_t pf_body_to_aabb(const pf_body_t *a);
bool pf_test_body(const pf_aabb_t *a, const pf_body_t *b);
bool pf_body_to_body(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration);
bool pf_solve_collision(const pf_body_t *a, const pf_body_t *b, pf_manifold_t *m);
v2f pf_gravity_v2f(pf_dir_t dir, float vel);

void pf_step_forces(float dt, pf_body_t *a);
void pf_apply_manifold(const pf_manifold_t *m, pf_body_t *a, pf_body_t *b);
void pf_update_dpos(float dt, pf_body_t *a);
void pf_apply_dpos(pf_body_t *a);
void pf_pos_correction(const pf_manifold_t *m, pf_body_t *a, pf_body_t *b);

void pf_body_set_mass(float mass, pf_body_t *a);
void pf_body_esque(float density, float restitution, pf_body_t *a);
void pf_rock_esque(pf_body_t *a);
void pf_wood_esque(pf_body_t *a);
void pf_metal_esque(pf_body_t *a);
void pf_bouncy_ball_esque(pf_body_t *a);
void pf_super_ball_esque(pf_body_t *a);
void pf_pillow_esque(pf_body_t *a);
void pf_static_esque(pf_body_t *a);

pf_body_t _pf_body();
pf_shape_t pf_circle(float radius);
pf_shape_t pf_box(float side);
pf_shape_t pf_rect(float w, float h);

#endif
