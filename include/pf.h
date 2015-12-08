#ifndef PF_H
#define PF_H

#include <ml.h>

typedef struct {
    v2f min;
    v2f max;
} pf_aabb;

typedef struct {
    v2f center;
    float radius;
} pf_circle;

typedef struct {
    float angle;
    float sin;
    float cos;
    float len;
    v2f normal;
} pf_face;

typedef struct {
    v2f point;
    pf_face face;
} pf_polypair;

typedef struct {
    pf_polypair *pairs;
    int count;
} pf_polygon;

// Overlapping borders should not register as a collision.
// This is because of the difficulty of finding normals between exacts points on tagnents.

bool pf_test_aabb_vs_aabb(const pf_aabb *a, const pf_aabb *b);
bool pf_test_point_vs_aabb(const v2f *a, const pf_aabb *b);
bool pf_test_circle_vs_circle(const v2f *a_pos, const pf_circle *a, const v2f *b_pos, const pf_circle *b);

void pf_compute_face(pf_face *f, const v2f *a, const v2f *b);

#endif
