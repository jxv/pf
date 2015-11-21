#ifndef PF_H
#define PF_H

#include <stdlib.h>
#include <stdbool.h>
#include <ml.h>

typedef struct {
    v2f min;
    v2f max;
} pf_aabb;

typedef enum {
    PF_SHAPE_RECT,
    PF_SHAPE_CIRCLE,
    PF_SHAPE_TRI,
} pf_shape_tag;

typedef enum {
    PF_CORNER_UL,   // up-left
    PF_CORNER_UR,   // up-right
    PF_CORNER_DL,   // down-left
    PF_CORNER_DR,   // down-right
} pf_corner;

typedef struct {
    v2f radii;
    pf_corner hypotenuse;
    float slope;
} pf_tri;

typedef struct {
    pf_shape_tag tag;
    union {
        v2f radii;
        float radius;
        pf_tri tri;
    };
} pf_shape;

typedef enum {
    PF_MODE_STATIC,
    PF_MODE_DYNAMIC,
} pf_mode;

typedef enum {
    PF_GROUP_PLATFORM,
    PF_GROUP_OBJECT,
} pf_group_tag;

typedef enum {
    PF_DIR_U,
    PF_DIR_D,
    PF_DIR_L,
    PF_DIR_R,
} pf_dir;

typedef struct {
    v2f impulse;    // Strength of the force
    v2f decay;      // Percentage to retain each iteration
    v2f cap;        // Max/min speed capacity
} pf_force;

typedef struct {
    pf_dir dir;
    float accel;
    float cap;
    float vel;
} pf_gravity;

struct pf_body;

typedef enum {
    PF_PLATFORM_NO_WAY,     // Solid platform
    PF_PLATFORM_ONE_WAY,    // Jump-up-onto only platform
    PF_PLATFORM_TWO_WAY,    // Jump-up and jump-down platform
} pf_platform_tag;

typedef struct {
    pf_platform_tag tag;
    v2f convey;
} pf_platform;

typedef enum {
    PF_OBJECT_CHARACTER,
    PF_OBJECT_ITEM,
} pf_object_tag;

typedef struct {
    pf_object_tag tag;
    struct pf_body const *parent;
} pf_object;

typedef struct {
    pf_group_tag tag;
    union {
        pf_platform platform;
        pf_object object;
    };
} pf_group;

typedef struct pf_body {
    pf_mode mode;
    pf_group group;
    pf_shape shape;
    v2f pos;
    v2f dpos;               // Change of position
    pf_force in;            // Internal/automonous
    pf_force ex;            // External
    pf_gravity gravity;
    float mass;
    float inverse_mass;
    float static_friction;
    float dynamic_friction;
    float restitution;
} pf_body;

typedef struct {
    v2f normal;
    float penetration;
    float mixed_restitution;
    float dynamic_friction;
    float static_friction;
} pf_manifold;

bool pf_intersect(const pf_aabb *a, const pf_aabb *b);
bool pf_inside(const v2f *a, const pf_aabb *b);
v2f pf_aabb_pos(const pf_aabb *a);
pf_aabb pf_body_to_aabb(const pf_body *a);
bool pf_test_body(const pf_aabb *a, const pf_body *b);
bool pf_body_to_body(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);
bool pf_solve_collision(const pf_body *a, const pf_body *b, pf_manifold *m);
v2f pf_gravity_v2f(pf_dir dir, float vel);

void pf_step_forces(float dt, pf_body *a);
void pf_apply_manifold(const pf_manifold *m, pf_body *a, pf_body *b);
void pf_update_dpos(float dt, pf_body *a);
void pf_apply_dpos(pf_body *a);
void pf_pos_correction(const pf_manifold *m, pf_body *a, pf_body *b);

void pf_body_set_mass(float mass, pf_body *a);
void pf_body_esque(float density, float restitution, pf_body *a);
void pf_rock_esque(pf_body *a);
void pf_wood_esque(pf_body *a);
void pf_metal_esque(pf_body *a);
void pf_bouncy_ball_esque(pf_body *a);
void pf_super_ball_esque(pf_body *a);
void pf_pillow_esque(pf_body *a);
void pf_static_esque(pf_body *a);

pf_tri _pf_tri(v2f radii, pf_corner hypotenuse); 
pf_body _pf_body();
pf_shape pf_circle(float radius);
pf_shape pf_box(float side);
pf_shape pf_rect(float w, float h);

bool pf_test_tri(const pf_aabb *a, const v2f *pos, const pf_tri *t);
bool pf_rect_to_tri(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);

#endif
