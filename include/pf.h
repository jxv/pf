#ifndef PF_H
#define PF_H

#include <stdlib.h>
#include <stdbool.h>
#include <ml.h>

typedef struct {
    v2f min;
    v2f max;
} PfAabb;

typedef enum {
    PF_SHAPE_RECT,
    PF_SHAPE_CIRCLE,
    PF_SHAPE_TRI,
} PfShapeTag;

typedef enum {
    PF_CORNER_UL,   // up-left
    PF_CORNER_UR,   // up-right
    PF_CORNER_DL,   // down-left
    PF_CORNER_DR,   // down-right
} PfCorner;

typedef struct {
    v2f radii;
    bool line;  // use slope only
    PfCorner hypotenuse;
    float radians;
    float m;    // slope
    v2f proj;   // projection vector
    v2f normal; // normal
    float sin;  // calculated with radians
    float cos;  // calculated with radias
} PfTri;

typedef struct {
    PfShapeTag tag;
    union {
        v2f radii;
        float radius;
        PfTri tri;
    };
} PfShape;

typedef enum {
    PF_MODE_STATIC,
    PF_MODE_DYNAMIC,
} PfMode;

typedef enum {
    PF_GROUP_PLATFORM,
    PF_GROUP_OBJECT,
} PfGroupTag;

typedef enum {
    PF_DIR_U = 1,
    PF_DIR_D = 2,
    PF_DIR_L = 4,
    PF_DIR_R = 8,
} PfDir;

typedef struct {
    v2f impulse;    // Strength of the force
    v2f decay;      // Percentage to retain each iteration
    v2f cap;        // Max/min speed capacity
} PfForce;

typedef struct {
    PfDir dir;
    float accel;
    float cap;
    float vel;
} PfGravity;

struct PfBody;

/*
typedef enum {
    PF_PLATFORM_NO_WAY,     // Solid platform
    PF_PLATFORM_ONE_WAY,    // Jump-up-onto only platform
    PF_PLATFORM_TWO_WAY,    // Jump-up and jump-down platform
} PfPlatformTag;
*/

typedef struct {
    //PfPlatform_tag tag;
    PfDir allow;
    v2f convey;
    struct PfBody *left;
    struct PfBody *right;
} PfPlatform;

typedef enum {
    PF_OBJECT_CHARACTER,
    PF_OBJECT_ITEM,
} PfObjectTag;

typedef struct PfObject {
    PfObjectTag tag;
    struct PfBody const *parent;
    bool check_parent;
} PfObject;

typedef struct {
    PfGroupTag tag;
    union {
        PfPlatform platform;
        PfObject object;
    };
} PfGroup;

typedef struct  PfBody {
    PfMode mode;
    PfGroup group;
    PfShape shape;
    v2f pos;
    v2f dpos;               // Change of position
    PfForce in;            // Internal/automonous
    PfForce ex;            // External
    PfGravity gravity;
    float mass;
    float inverse_mass;
    float static_friction;
    float dynamic_friction;
    float restitution;
} PfBody;

typedef struct {
    v2f normal;
    float penetration;
    float mixed_restitution;
    float dynamic_friction;
    float static_friction;
} PfManifold;

bool pf_intersect(const PfAabb *a, const PfAabb *b);
bool pf_inside(const v2f *a, const PfAabb *b);
v2f pf_aabb_pos(const PfAabb *a);
PfAabb pf_body_to_aabb(const PfBody *a);
bool pf_test_body(const PfAabb *a, const PfBody *b);
bool pf_body_to_body(const PfBody *a, const PfBody *b, v2f *normal, float *penetration);
bool pf_solve_collision(const PfBody *a, const PfBody *b, PfManifold *m);
v2f pf_gravity_v2f(PfDir dir, float vel);

void pf_step_forces(float dt, PfBody *a);
void pf_apply_manifold(const PfManifold *m, PfBody *a, PfBody *b);
void pf_update_dpos(float dt, PfBody *a);
void pf_apply_dpos(PfBody *a);
void pf_pos_correction(const PfManifold *m, PfBody *a, PfBody *b);

void pf_body_set_mass(float mass, PfBody *a);
void pf_body_esque(float density, float restitution, PfBody *a);
void pf_rock_esque(PfBody *a);
void pf_wood_esque(PfBody *a);
void pf_metal_esque(PfBody *a);
void pf_bouncy_ball_esque(PfBody *a);
void pf_super_ball_esque(PfBody *a);
void pf_pillow_esque(PfBody *a);
void pf_static_esque(PfBody *a);

PfTri _pf_tri(v2f radii, bool line, PfCorner hypotenuse); 
PfGroup _pf_platform();
PfBody _pf_body();
PfShape pf_circle(float radius);
PfShape pf_box(float side);
PfShape pf_rect(float w, float h);

bool pf_test_tri(const PfAabb *a, const v2f *pos, const PfTri *t);
bool pf_rect_to_tri(const PfBody *a, const PfBody *b, v2f *normal, float *penetration);
float pf_line_point_dist(float p_m, float p_b, float q_x, float q_y);
PfAabb _to_aabb(const PfBody *a);
void pf_transform_move_on_slope(PfBody *a, float dt);

v2f pf_move_left_on_slope_transform(const PfTri *t);
v2f pf_move_right_on_slope_transform(const PfTri *t);

#endif
