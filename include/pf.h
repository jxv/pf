#ifndef PF_H
#define PF_H

#include <ml.h>

typedef struct {
    v2f min;
    v2f max;
} pf_aabb;

bool pf_test_aabb_vs_aabb(const pf_aabb *a, const pf_aabb *b);

#endif
