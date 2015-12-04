#include "pf.h"

bool pf_test_aabb_vs_aabb(const pf_aabb *a, const pf_aabb *b) {
    return
        a->min.x <= b->max.x &&
        b->min.x <= a->max.x &&
        a->min.y <= b->max.y &&
        b->min.y <= a->max.y;
}

bool pf_test_circle_vs_circle(const v2f *a_pos, const pf_circle *a, const v2f *b_pos, const pf_circle *b) {
    return false;
}
