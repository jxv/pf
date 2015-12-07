#include <math.h>
#include "pf.h"

bool pf_test_aabb_vs_aabb(const pf_aabb *a, const pf_aabb *b) {
    return
        a->min.x < b->max.x &&
        b->min.x < a->max.x &&
        a->min.y < b->max.y &&
        b->min.y < a->max.y;
}

bool pf_test_point_vs_aabb(const v2f *a, const pf_aabb *b) {
    return
        a->x < b->max.x &&
        a->x > b->min.x &&
        a->y < b->max.y &&
        a->y > b->min.y;
}

bool pf_test_circle_vs_circle(const v2f *a_pos, const pf_circle *a, const v2f *b_pos, const pf_circle *b) {
    const v2f a_center = addv2f(*a_pos, a->center);
    const v2f b_center = addv2f(*b_pos, b->center);
    const v2f n = subv2f(b_center, a_center);
    float dist_sq = sqlenv2f(n);
    float radius = a->radius + b->radius;
    return dist_sq < radius * radius;
}
