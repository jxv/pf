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

bool pf_test_aabb_vs_aabb(const pf_aabb *a, const pf_aabb *b);
bool pf_test_point_vs_aabb(const v2f *a, const pf_aabb *b);
bool pf_test_circle_vs_circle(const v2f *a_pos, const pf_circle *a, const v2f *b_pos, const pf_circle *b);

#endif
