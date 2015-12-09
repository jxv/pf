#include <math.h>
#include <stdio.h>
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

void pf_compute_face(pf_face *f, const v2f *a, const v2f *b) {
    v2f c = subv2f(*b, *a);
    f->angle = atan2f(c.y, c.x);
    while (f->angle < 0) {
        f->angle += M_PI * 2;
    }
    f->sin = sinf(f->angle);
    f->cos = cosf(f->angle);
    f->len = lenv2f(c);
    f->normal = _v2f(-f->sin, f->cos);
}

pf_polypair _pf_polypair(v2f point) {
    return (pf_polypair) {
        .point = point,
        .face = {
            .angle = 0,
            .sin = 0,
            .cos = 0,
            .len = 0,
            .normal = _v2f(0,0),
        },
    };
}

pf_platform_bind _pf_platform_bind_ab(
    int a_polygon_index, int a_face_index, pf_face_point a_point,
    int b_polygon_index, int b_face_index, pf_face_point b_point) {
    return (pf_platform_bind) {
        .a = {
            .is = true,
            .polygon_index = a_polygon_index,
            .face_index = a_face_index,
            .point = a_point,
        },
        .b = {
            .is = true,
            .polygon_index = b_polygon_index,
            .face_index = b_face_index,
            .point = b_point,
        },
    };
}

pf_platform_bind _pf_platform_bind_a(int a_polygon_index, int a_face_index, pf_face_point a_point) {
    return (pf_platform_bind) {
        .a = {
            .is = true,
            .polygon_index = a_polygon_index,
            .face_index = a_face_index,
            .point = a_point,
        },
        .b = {
            .is = false,
            .polygon_index = 0,
            .face_index = 0,
            .point = 0,
        },
    };
}
