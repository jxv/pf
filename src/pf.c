#include "pf.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>

typedef enum {
    PF_TRI_REGION_AB,
    PF_TRI_REGION_AC,
    PF_TRI_REGION_BC,
    PF_TRI_REGION_A,
    PF_TRI_REGION_B,
    PF_TRI_REGION_C,
} pf_tri_region;

typedef enum {
    PF_RECT_REGION_U,
    PF_RECT_REGION_D,
    PF_RECT_REGION_L,
    PF_RECT_REGION_R,
    PF_RECT_REGION_UL,
    PF_RECT_REGION_UR,
    PF_RECT_REGION_DL,
    PF_RECT_REGION_DR,
} pf_rect_region;

typedef struct {
    struct {
        float min;
        float max;
    } a;
    struct {
        float min;
        float max;
    } b;
} pf_project;

bool pf_intersect(const pf_aabb *a, const pf_aabb *b) {
    return
        a->max.x >= b->min.x && a->min.x <= b->max.x &&
        a->max.y >= b->min.y && a->min.y <= b->max.y;
}

bool pf_inside(const v2f *a, const pf_aabb *b) {
    return
        a->x >= b->min.x && a->x <= b->max.x &&
        a->y >= b->min.y && a->y <= b->max.y;
}

float pf_line_point_dist(float p_m, float p_b, float q_x, float q_y) {
    const float q_m = -1.0 / p_m;
    const float q_b = q_y - (q_m * q_x);
    const float r_x = (p_b - q_b) / (q_m - p_m);
    const float r_y = p_m * r_x + p_b;
    return lenv2f(subv2f(_v2f(r_x, r_y), _v2f(q_x, q_y)));
}

float pf_perp_slope(float m) {
    return -1.f / m;
}

float pf_slope_offset(float m, float x, float y) {
    return y - (m * x);
}

v2f pf_normalize_segment(const v2f *a, const v2f *b) {
    return normv2f(subv2f(*a, *b));
}

float pf_slope_from_points(const v2f *b, const v2f *a) {
    return (a->y - b->y) / (a->x - b->x);
}

float tri_angle(const v2f *radii, pf_corner hypotenuse) {
    switch (hypotenuse) {
    case PF_CORNER_UL:
    case PF_CORNER_DR:
        return atan2(-radii->x, radii->y);
    case PF_CORNER_UR:
    case PF_CORNER_DL:
        return atan2(radii->x, radii->y);
    default:
        assert(false);
    }
}

m2f rotation_matrix(float radians) {
    const float s = sinf(radians);
    const float c = cosf(radians);
    return _m2f(c, -s, s, c);
}

v2f projection_vector(float radians) {
    return _v2f(cosf(radians), -sinf(radians));
}

float project_overlap(const pf_project *proj) {
    if ((proj->a.max < proj->b.min) || (proj->b.max < proj->a.min)) {
        return 0;
    }
    return fminf(proj->a.max, proj->b.max) - fmaxf(proj->a.min, proj->b.min);
}

pf_aabb pf_rect_to_aabb(const v2f *pos, const v2f *radii) {
    return (pf_aabb) {
        .min = subv2f(*pos, *radii),
        .max = addv2f(*pos, *radii)
    };
}

pf_aabb pf_circle_to_aabb(const v2f *pos, float radius) {
    return (pf_aabb) {
        .min = subv2nf(*pos, radius),
        .max = divv2nf(*pos, radius)
    };
}

pf_aabb pf_tri_to_aabb(const v2f *pos, const pf_tri *tri) {
    return (pf_aabb) {
        .min = subv2f(*pos, tri->radii),
        .max = addv2f(*pos, tri->radii)
    };
}

pf_aabb pf_shape_to_aabb(const v2f *pos, const pf_shape *sh) {
    switch (sh->tag) {
    case PF_SHAPE_RECT:
        return pf_rect_to_aabb(pos, &sh->radii);
    case PF_SHAPE_CIRCLE:
        return pf_circle_to_aabb(pos, sh->radius);
    case PF_SHAPE_TRI:
        return pf_tri_to_aabb(pos, &sh->tri);
    default:
        assert(false);
    }
}

pf_aabb pf_body_to_aabb(const pf_body *a) {
    return pf_shape_to_aabb(&a->pos, &a->shape);
}


void pf_closest_point_to_line(const v2f *a, const v2f *b, const v2f *c, v2f *d) {
    const v2f ab = subv2f(*b, *a);
    const float t = dotv2f(subv2f(*c, *a), ab) / dotv2f(ab, ab);
    *d = addv2f(*a, mulv2nf(ab, t));
}

bool pf_closest_point_to_segment(const v2f *a, const v2f *b, const v2f *c, v2f *d) {
    const v2f ab = subv2f(*b, *a);
    const float t = dotv2f(subv2f(*c, *a), ab) / dotv2f(ab, ab);
    if (t >= 0.0f && t <= 1.0f) {
        *d = addv2f(*a, mulv2nf(ab, t));
        return true;
    }
    return false;
}

bool pf_point_in_triangle(const v2f *a, const v2f *b, const v2f *c, const v2f *p) {
    // out of a
    const v2f ab = subv2f(*b, *a);
    const v2f ac = subv2f(*c, *a);
    const v2f ap = subv2f(*p, *a);
    const float d1 = dotv2f(ab, ap);
    const float d2 = dotv2f(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) {
        return false;
    }

    // out of b
    const v2f bp = subv2f(*p, *b);
    const float d3 = dotv2f(ab, bp);
    const float d4 = dotv2f(ac, bp);
    if (d3 >= 0.f && d4 <= d3) {
        return false;
    }

    // out of edge a-b
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        return false;
    }

    // out of c
    const v2f cp = subv2f(*p, *c);
    const float d5 = dotv2f(ab, cp);
    const float d6 = dotv2f(ac, cp);
    if (d6 >= 0.f && d5 <= d6) {
        return false;
    }

    // out of edge a-c
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        return false;
    }

    // out of edge b-c
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        return false;
    }

    // inside
    return true;
}

// closest point on rect perimeter
void pf_closest_point_rect(const v2f *pos, const v2f *radii, const v2f *p, v2f *q) {
    const pf_aabb b = pf_rect_to_aabb(pos, radii);
    if (pf_inside(p, &b)) {
        const v2f xclamp = _v2f(clampf(b.min.x, b.max.x, p->x), p->y);
        const v2f yclamp = _v2f(p->x, clampf(b.min.y, b.max.y, p->y));
        *q = sqlenv2f(subv2f(*p, xclamp)) < sqlenv2f(subv2f(*p, yclamp))
            ? xclamp
            : yclamp;
    } else {
        *q = clampv2f(subv2f(*pos, *radii), addv2f(*pos, *radii), *p);
    }
}

// q is closest point, returns point's region
void pf_closest_point_triangle_no_region(const v2f *a, const v2f *b, const v2f *c, const v2f *p, v2f *q) {
    // out of a
    const v2f ab = subv2f(*b, *a);
    const v2f ac = subv2f(*c, *a);
    const v2f ap = subv2f(*p, *a);
    const float d1 = dotv2f(ab, ap);
    const float d2 = dotv2f(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) {
        *q = *a;
        return;
    }

    // out of b
    const v2f bp = subv2f(*p, *b);
    const float d3 = dotv2f(ab, bp);
    const float d4 = dotv2f(ac, bp);
    if (d3 >= 0.f && d4 <= d3) {
        *q = *b;
        return;
    }

    // out of edge a-b
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        const float v = d1 / (d1 - d3);
        *q = addv2f(*a, mulv2nf(ab, v));
        return;
    }

    // out of c
    const v2f cp = subv2f(*p, *c);
    const float d5 = dotv2f(ab, cp);
    const float d6 = dotv2f(ac, cp);
    if (d6 >= 0.f && d5 <= d6) {
        *q = *c;
        return;
    }

    // out of edge a-c
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        const float w = d2 / (d2 - d6);
        *q = addv2f(*a, mulv2nf(ac, w));
        return;
    }

    // out of edge b-c
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        *q = addv2f(*b, mulv2nf(subv2f(*c, *b), w));
        return;
    }

    // inside (optimize here at some point)
    const v2f center = mulv2nf(_v2f(a->x + b->x + c->x, a->y + b->y + c->y), 1.f / 3.f);
    if (pf_point_in_triangle(&center, a, b, p)) {
        pf_closest_point_to_line(a, b, p, q);
    } else if (pf_point_in_triangle(&center, a, c, p)) {
        pf_closest_point_to_line(a, c, p, q);
    } else {
        assert(pf_point_in_triangle(&center, b, c, p));
        pf_closest_point_to_line(b, c, p, q);
    }
}


// q is closest point, returns point's region
pf_tri_region pf_closest_point_triangle(const v2f *a, const v2f *b, const v2f *c, const v2f *p, v2f *q, bool *inside) {
    *inside = false;
    // out of a
    const v2f ab = subv2f(*b, *a);
    const v2f ac = subv2f(*c, *a);
    const v2f ap = subv2f(*p, *a);
    const float d1 = dotv2f(ab, ap);
    const float d2 = dotv2f(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) {
        *q = *a;
        return PF_TRI_REGION_A;
    }

    // out of b
    const v2f bp = subv2f(*p, *b);
    const float d3 = dotv2f(ab, bp);
    const float d4 = dotv2f(ac, bp);
    if (d3 >= 0.f && d4 <= d3) {
        *q = *b;
        return PF_TRI_REGION_B;
    }

    // out of edge a-b
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        const float v = d1 / (d1 - d3);
        *q = addv2f(*a, mulv2nf(ab, v));
        return PF_TRI_REGION_AB;
    }

    // out of c
    const v2f cp = subv2f(*p, *c);
    const float d5 = dotv2f(ab, cp);
    const float d6 = dotv2f(ac, cp);
    if (d6 >= 0.f && d5 <= d6) {
        *q = *c;
        return PF_TRI_REGION_C;
    }

    // out of edge a-c
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        const float w = d2 / (d2 - d6);
        *q = addv2f(*a, mulv2nf(ac, w));
        return PF_TRI_REGION_AC;
    }

    // out of edge b-c
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        *q = addv2f(*b, mulv2nf(subv2f(*c, *b), w));
        return PF_TRI_REGION_BC;
    }

    // inside (optimize here at some point)
    *inside = true;
    const v2f center = mulv2nf(_v2f(a->x + b->x + c->x, a->y + b->y + c->y), 1.f / 3.f);
    if (pf_point_in_triangle(&center, a, b, p)) {
        pf_closest_point_to_line(a, b, p, q);
        return PF_TRI_REGION_AB;
    } else if (pf_point_in_triangle(&center, a, c, p)) {
        pf_closest_point_to_line(a, c, p, q);
        return PF_TRI_REGION_AC;
    } else {
        assert(pf_point_in_triangle(&center, b, c, p));
        pf_closest_point_to_line(b, c, p, q);
        return PF_TRI_REGION_BC;
    }
}

void pf_flip(v2f *normal, float *penetration) {
    *normal = negv2f(*normal);
    *penetration = - *penetration;
}

void pf_point_to_point(const v2f *a, const v2f *b, v2f *normal, float *penetration) {
    const v2f seg = subv2f(*a, *b);
    *penetration = lenv2f(seg);
    *normal = nearzerof(*penetration) ? _v2f(0, 1) : normv2f(seg);
}

// Penetration is negative if c.x is less than the closest contact's x
bool pf_point_to_face(const v2f *a, const v2f *b, const v2f *c, v2f *normal, float *penetration, v2f *closest_point) {
    if (pf_closest_point_to_segment(a, b, c, closest_point)) {
        pf_point_to_point(c, closest_point, normal, penetration);
        return true;
    }
    return false;
}

bool pf_circle_to_circle_(const v2f *a_pos, float a_radius, const v2f *b_pos, float b_radius, v2f *normal, float *penetration) {
    const v2f n = subv2f(*b_pos, *a_pos);
    float dist_sq = sqlenv2f(n);
    float dist = sqrtf(dist_sq);
    float radius = a_radius + b_radius;
    if (dist_sq >= radius * radius) {
        return false;
    } else if (dist == 0) {
        *penetration = a_radius;
        *normal = _v2f(1,0);
    } else {
        *penetration = radius - dist;
        *normal = divv2nf(n, dist);
    }
    return true;
}

bool pf_circle_to_point(const v2f *a_pos, float a_radius, const v2f *b_pos, v2f *normal, float *penetration) {
    return pf_circle_to_circle_(a_pos, a_radius, b_pos, 0, normal, penetration);
}

bool pf_point_to_circle(const v2f *a_pos, const v2f *b_pos, float b_radius, v2f *normal, float *penetration) {
    return pf_circle_to_circle_(a_pos, 0, b_pos, b_radius, normal, penetration);
}

bool pf_test_rect(const pf_aabb *a, const pf_body *b) {
    assert(b->shape.tag == PF_SHAPE_RECT);
    const pf_aabb rect = pf_body_to_aabb(b);
    return pf_intersect(a, &rect);
}

bool pf_test_circle(const pf_aabb *a, const pf_body *b) {
    assert(b->shape.tag == PF_SHAPE_CIRCLE);
    const bool inside = pf_inside(&b->pos, a);
    const v2f closest = clampv2f(a->min, a->max, b->pos);
    const v2f pos_diff = subv2f(b->pos, pf_aabb_pos(a));
    float radius = b->shape.radius;
    const v2f normal = subv2f(pos_diff, closest);
    float dist_sq = sqlenv2f(normal);
    return inside || dist_sq <= (radius *radius);
}

inline
bool pf_point_in_tri_ul(float x, float y, const v2f *dr, const v2f *ofs, float slope) {
    return
        x <= dr->x &&
        y <= dr->y &&
        (y - ofs->y) >= (x - ofs->x) * slope;
}
 
bool pf_test_tri_ul(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const v2f ofs = addv2f(*pos, _v2f(-radii->x,  radii->y));
    const v2f ur = addv2f(*pos, _v2f( radii->x, -radii->y));
    const v2f dl = addv2f(*pos, _v2f(-radii->x,  radii->y));
    const v2f dr = addv2f(*pos, _v2f( radii->x,  radii->y));
    return
        pf_inside(&ur, a) ||
        pf_inside(&dl, a) ||
        pf_inside(&dr, a) ||
        pf_point_in_tri_ul(a->min.x, a->min.y, &dr, &ofs, slope) ||
        pf_point_in_tri_ul(a->max.x, a->min.y, &dr, &ofs, slope) ||
        pf_point_in_tri_ul(a->min.x, a->max.y, &dr, &ofs, slope) ||
        pf_point_in_tri_ul(a->max.x, a->max.y, &dr, &ofs, slope);
}

inline
bool pf_point_in_tri_ur(float x, float y, const v2f *dl, const v2f *ofs, float slope) {
    return
        x >= dl->x &&
        y <= dl->y &&
        (y - ofs->y) >= (x - ofs->x) * slope;
}
 
bool pf_test_tri_ur(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const v2f ofs = subv2f(*pos, *radii);
    const v2f ul = addv2f(*pos, _v2f(-radii->x, -radii->y));
    const v2f dl = addv2f(*pos, _v2f(-radii->x,  radii->y));
    const v2f dr = addv2f(*pos, _v2f( radii->x,  radii->y));
   return
        pf_inside(&ul, a) ||
        pf_inside(&dl, a) ||
        pf_inside(&dr, a) ||
        pf_point_in_tri_ur(a->min.x, a->min.y, &dl, &ofs, slope) ||
        pf_point_in_tri_ur(a->max.x, a->min.y, &dl, &ofs, slope) ||
        pf_point_in_tri_ur(a->min.x, a->max.y, &dl, &ofs, slope) ||
        pf_point_in_tri_ur(a->max.x, a->max.y, &dl, &ofs, slope);
}

inline
bool pf_point_in_tri_dl(float x, float y, const v2f *ur, const v2f *ofs, float slope) {
    return
        x <= ur->x &&
        y >= ur->y &&
        (y - ofs->y) <= (x - ofs->x) * slope;
}
 
bool pf_test_tri_dl(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const v2f ofs = subv2f(*pos, *radii);
    const v2f ul = addv2f(*pos, _v2f(-radii->x, -radii->y));
    const v2f ur = addv2f(*pos, _v2f( radii->x, -radii->y));
    const v2f dr = addv2f(*pos, _v2f( radii->x,  radii->y));
   return
        pf_inside(&ul, a) ||
        pf_inside(&ur, a) ||
        pf_inside(&dr, a) ||
        pf_point_in_tri_dl(a->min.x, a->min.y, &ur, &ofs, slope) ||
        pf_point_in_tri_dl(a->max.x, a->min.y, &ur, &ofs, slope) ||
        pf_point_in_tri_dl(a->min.x, a->max.y, &ur, &ofs, slope) ||
        pf_point_in_tri_dl(a->max.x, a->max.y, &ur, &ofs, slope);
}

inline
bool pf_point_in_tri_dr(float x, float y, const v2f *ul, const v2f *ofs, float slope) {
    return
        x >= ul->x &&
        y >= ul->y &&
        (y - ofs->y) <= (x - ofs->x) * slope;
}
 
bool pf_test_tri_dr(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const v2f ofs = addv2f(*pos, _v2f(-radii->x,  radii->y));
    const v2f ul = addv2f(*pos, _v2f(-radii->x, -radii->y));
    const v2f ur = addv2f(*pos, _v2f( radii->x, -radii->y));
    const v2f dl = addv2f(*pos, _v2f(-radii->x,  radii->y));
   return
        pf_inside(&ul, a) ||
        pf_inside(&ur, a) ||
        pf_inside(&dl, a) ||
        pf_point_in_tri_dr(a->min.x, a->min.y, &ul, &ofs, slope) ||
        pf_point_in_tri_dr(a->max.x, a->min.y, &ul, &ofs, slope) ||
        pf_point_in_tri_dr(a->min.x, a->max.y, &ul, &ofs, slope) ||
        pf_point_in_tri_dr(a->max.x, a->max.y, &ul, &ofs, slope);
}

bool pf_test_tri(const pf_aabb *a, const v2f *pos, const pf_tri *t) {
    switch (t->hypotenuse) {
    case PF_CORNER_UL:
        return pf_test_tri_ul(a, pos, &t->radii, t->m);
    case PF_CORNER_UR:
        return pf_test_tri_ur(a, pos, &t->radii, t->m);
    case PF_CORNER_DL:
        return pf_test_tri_dl(a, pos, &t->radii, t->m);
    case PF_CORNER_DR:
        return pf_test_tri_dr(a, pos, &t->radii, t->m);
    default:
        assert(false);
    }
}

bool pf_test_body(const pf_aabb *a, const pf_body *b) {
    switch (b->shape.tag) {
    case PF_SHAPE_RECT:
        return pf_test_rect(a, b);
    case PF_SHAPE_CIRCLE:
        return pf_test_circle(a, b);
    case PF_SHAPE_TRI:
        return pf_test_tri(a, &b->pos, &b->shape.tri);
    default:
        assert(false);
    }
}

bool pf_body_to_body_swap(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);
bool pf_rect_to_rect(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);
bool pf_rect_to_circle(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);
bool pf_rect_to_tri(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);
bool pf_circle_to_circle(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);
bool pf_circle_to_tri(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);

bool pf_body_to_body(const pf_body *a, const pf_body *b,
             v2f *normal, float *penetration) {
    switch (a->shape.tag) {
    case PF_SHAPE_RECT:
        switch (b->shape.tag) {
        case PF_SHAPE_RECT:
            return pf_rect_to_rect(a, b, normal, penetration);
        case PF_SHAPE_CIRCLE:
            return pf_rect_to_circle(a, b, normal, penetration);
        case PF_SHAPE_TRI:
            return pf_rect_to_tri(a, b, normal, penetration);
        default:
            assert(false);
        }
    case PF_SHAPE_CIRCLE:
        switch (b->shape.tag) {
        case PF_SHAPE_RECT:
            return pf_body_to_body_swap(a, b, normal, penetration);
        case PF_SHAPE_CIRCLE:
            return pf_circle_to_circle(a, b, normal, penetration);
        case PF_SHAPE_TRI:
            return pf_circle_to_tri(a, b, normal, penetration);
        default:
            assert(false);
        }
    case PF_SHAPE_TRI:
        switch (b->shape.tag) {
        case PF_SHAPE_RECT:
        case PF_SHAPE_CIRCLE:
            return pf_body_to_body_swap(a, b, normal, penetration);
        case PF_SHAPE_TRI:
        default:
            return false;
            //assert(false);
        }
    default:
        assert(false);
    }
}

bool pf_body_to_body_swap(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const bool test = pf_body_to_body(b, a, normal, penetration);
    if (test) {
        *normal = negv2f(*normal);
    }
    return test;
}

bool pf_aabb_to_aabb(const pf_aabb *a, const pf_aabb *b, v2f *normal, float *penetration) {
    const v2f a_pos = mulv2nf(addv2f(a->min, a->max), 0.5);
    const v2f b_pos = mulv2nf(addv2f(b->min, b->max), 0.5);
    const v2f n = subv2f(b_pos, a_pos);
    const v2f a_radii = mulv2nf(subv2f(a->max, a->min), 0.5);
    const v2f b_radii = mulv2nf(subv2f(b->max, b->min), 0.5);
    const v2f overlap = subv2f(addv2f(a_radii, b_radii), absv2f(n));
    if (!pf_intersect(a, b)) {
        return false;
    } else if (fabsf(overlap.x) < fabsf(overlap.y)) {
        *penetration = overlap.x;
        *normal = _v2f(n.x < 0 ? -1 : 1, 0);
        return true;
    } else {
        *penetration = overlap.y;
        *normal = _v2f(0, n.y < 0 ? -1 : 1);
        return true;
    }
}

bool pf_rect_to_rect(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const v2f n = subv2f(b->pos, a->pos);
    const v2f overlap = subv2f(addv2f(a->shape.radii, b->shape.radii), absv2f(n));
    const pf_aabb a_shape = pf_body_to_aabb(a);
    const pf_aabb b_shape = pf_body_to_aabb(b);
    if (!pf_intersect(&a_shape, &b_shape)) {
        return false;
    } else if (fabsf(overlap.x) < fabsf(overlap.y)) {
        *penetration = overlap.x;
        *normal = _v2f(n.x < 0 ? -1 : 1, 0);
        return true;
    } else {
        *penetration = overlap.y;
        *normal = _v2f(0, n.y < 0 ? -1 : 1);
        return true;
    }
}

bool pf_rect_to_circle(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    // Is position outside of a direct vertical/horizontal face
    const bool out_lf = b->pos.x < a->pos.x - a->shape.radii.x;
    const bool out_rt = b->pos.x > a->pos.x + a->shape.radii.x;
    const bool out_up = b->pos.y < a->pos.y - a->shape.radii.y;
    const bool out_dn = b->pos.y > a->pos.y + a->shape.radii.y;
    if ((out_lf || out_rt) && (out_up || out_dn)) {
        /* Treat as (circle/corner_point)_to_circle collision */
        pf_body a_ = *a;
        a_.shape.tag = PF_SHAPE_CIRCLE;
        a_.shape.radius = 0;
        a_.pos.x += out_lf ? -a->shape.radii.x : a->shape.radii.x;
        a_.pos.y += out_up ? -a->shape.radii.y : a->shape.radii.y;
        return pf_circle_to_circle(&a_, b, normal, penetration);
    } else {
        /* Treat as pf_rect_to_rect collision */
        pf_body b_ = *b;
        b_.shape.tag = PF_SHAPE_RECT;
        b_.shape.radii = _v2f(b->shape.radius, b->shape.radius);
        return pf_rect_to_rect(a, &b_, normal, penetration);
    }
}

bool pf_circle_to_circle(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const v2f n = subv2f(b->pos, a->pos);
    float dist_sq = sqlenv2f(n);
    float dist = sqrtf(dist_sq);
    float radius = a->shape.radius + b->shape.radius;
    if (dist_sq >= radius *radius) {
        return false;
    } else if (dist == 0) {
        *penetration = a->shape.radius;
        *normal = _v2f(1,0);
    } else {
        *penetration = radius - dist;
        *normal = divv2nf(n, dist);
    }
    return true;
}

bool pf_circle_to_tri_ul(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_tri *t = &b->shape.tri;
    assert(t->hypotenuse == PF_CORNER_UL);
    const v2f ur = addv2f(b->pos, _v2f( t->radii.x, -t->radii.y));
    const v2f dl = addv2f(b->pos, _v2f(-t->radii.x,  t->radii.y));
    const v2f dr = addv2f(b->pos, _v2f( t->radii.x,  t->radii.y));
    v2f p;
    bool inside;
    const pf_tri_region region = pf_closest_point_triangle(&ur, &dl, &dr, &a->pos, &p, &inside);

    const v2f diff = subv2f(p, a->pos);
    *penetration = a->shape.radius + (inside ? lenv2f(diff) : -lenv2f(diff));

    if (*penetration <= 0) {
        return false;
    }

    if (!eqv2f(a->pos, p)) {
        *normal = inside ? negv2f(normv2f(diff)) : normv2f(diff);
    } else {
        switch (region) {
        case PF_TRI_REGION_AB:
            *normal = _v2f(1, pf_perp_slope(t->m));
            break;
        case PF_TRI_REGION_AC:
            *normal = _v2f(-1, 0);
            break;
        case PF_TRI_REGION_BC:
            *normal = _v2f(0, -1);
            break;
        case PF_TRI_REGION_A:
            *normal = _v2f(-1, 1);
            break;
        case PF_TRI_REGION_B:
            *normal = _v2f(1, -1);
            break;
        case PF_TRI_REGION_C:
            *normal = _v2f(-1, -1);
            break;
        default:
            assert(false);
        }
        *normal = normv2f(*normal);
    }
    return true;
}

bool pf_circle_to_tri_ur(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_tri *t = &b->shape.tri;
    assert(t->hypotenuse == PF_CORNER_UR);
    const v2f ul = addv2f(b->pos, _v2f(-t->radii.x, -t->radii.y));
    const v2f dl = addv2f(b->pos, _v2f(-t->radii.x,  t->radii.y));
    const v2f dr = addv2f(b->pos, _v2f( t->radii.x,  t->radii.y));
    v2f p;
    bool inside;
    const pf_tri_region region = pf_closest_point_triangle(&ul, &dl, &dr, &a->pos, &p, &inside);
    
    const v2f diff = subv2f(p, a->pos);
    *penetration = a->shape.radius + (inside ? lenv2f(diff) : -lenv2f(diff));

    if (*penetration <= 0) {
        return false;
    }

    if (!eqv2f(a->pos, p)) {
        *normal = inside ? negv2f(normv2f(diff)) : normv2f(diff);
    } else {
        switch (region) {
        case PF_TRI_REGION_AB:
            *normal = _v2f(1, 0);
            break;
        case PF_TRI_REGION_AC:
            *normal = _v2f(-1, -pf_perp_slope(t->m));
            break;
        case PF_TRI_REGION_BC:
            *normal = _v2f(0, -1);
            break;
        case PF_TRI_REGION_A:
            *normal = _v2f(1, 1);
            break;
        case PF_TRI_REGION_B:
            *normal = _v2f(1, -1);
            break;
        case PF_TRI_REGION_C:
            *normal = _v2f(-1, -1);
            break;
        default:
            assert(false);
        }
        *normal = normv2f(*normal);
    }
    return true;
}

bool pf_circle_to_tri_dl(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_tri *t = &b->shape.tri;
    assert(t->hypotenuse == PF_CORNER_DL);
    const v2f ul = addv2f(b->pos, _v2f(-t->radii.x, -t->radii.y));
    const v2f ur = addv2f(b->pos, _v2f( t->radii.x, -t->radii.y));
    const v2f dr = addv2f(b->pos, _v2f( t->radii.x,  t->radii.y));
    v2f p;
    bool inside;
    const pf_tri_region region = pf_closest_point_triangle(&ul, &ur, &dr, &a->pos, &p, &inside);

    const v2f diff = subv2f(p, a->pos);
    *penetration = a->shape.radius + (inside ? lenv2f(diff) : -lenv2f(diff));

    if (*penetration <= 0) {
        return false;
    }

    if (!eqv2f(a->pos, p)) {
        *normal = inside ? negv2f(normv2f(diff)) : normv2f(diff);
    } else {
        switch (region) {
        case PF_TRI_REGION_AB:
            *normal = _v2f(0, 1);
            break;
        case PF_TRI_REGION_AC:
            *normal = _v2f(1, pf_perp_slope(t->m));
            break;
        case PF_TRI_REGION_BC:
            *normal = _v2f(-1, 0);
            break;
        case PF_TRI_REGION_A:
            *normal = _v2f(1, 1);
            break;
        case PF_TRI_REGION_B:
            *normal = _v2f(-1, 1);
            break;
        case PF_TRI_REGION_C:
            *normal = _v2f(-1, -1);
            break;
        default:
            break;
        }
        *normal = normv2f(*normal);
    }
    return true;
}

bool pf_circle_to_tri_dr(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_tri *t = &b->shape.tri;
    assert(t->hypotenuse == PF_CORNER_DR);
    const v2f ul = addv2f(b->pos, _v2f(-t->radii.x, -t->radii.y));
    const v2f ur = addv2f(b->pos, _v2f( t->radii.x, -t->radii.y));
    const v2f dl = addv2f(b->pos, _v2f(-t->radii.x,  t->radii.y));
    v2f p;
    bool inside;
    const pf_tri_region region = pf_closest_point_triangle(&ul, &ur, &dl, &a->pos, &p, &inside);

    const v2f diff = subv2f(p, a->pos);
    *penetration = a->shape.radius + (inside ? lenv2f(diff) : -lenv2f(diff));

    if (*penetration <= 0) {
        return false;
    }

    if (!eqv2f(a->pos, p)) {
        *normal = inside ? negv2f(normv2f(diff)) : normv2f(diff);
    } else {
        switch (region) {
        case PF_TRI_REGION_AB:
            *normal = _v2f(0, 1);
            break;
        case PF_TRI_REGION_AC:
            *normal = _v2f(1, 0);
            break;
        case PF_TRI_REGION_BC:
            *normal = _v2f(-1, -pf_perp_slope(t->m));
            break;
        case PF_TRI_REGION_A:
            *normal = _v2f(1, 1);
            break;
        case PF_TRI_REGION_B:
            *normal = _v2f(-1, 1);
            break;
        case PF_TRI_REGION_C:
            *normal = _v2f(1, -1);
            break;
        default:
            assert(false);
        }
        *normal = normv2f(*normal);
    }
    return true;
}

bool pf_circle_to_tri(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    switch (b->shape.tri.hypotenuse) {
    case PF_CORNER_UL:
        return pf_circle_to_tri_ul(a, b, normal, penetration);
    case PF_CORNER_UR:
        return pf_circle_to_tri_ur(a, b, normal, penetration);
    case PF_CORNER_DL:
        return pf_circle_to_tri_dl(a, b, normal, penetration);
    case PF_CORNER_DR:
        return pf_circle_to_tri_dr(a, b, normal, penetration);
    default:
        assert(false);
    }
}

bool pf_rect_to_tri_ul(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_aabb r_box = pf_rect_to_aabb(&a->pos, &a->shape.radii);
    const pf_tri *t = &b->shape.tri;
    const pf_aabb t_box = pf_tri_to_aabb(&b->pos, t);

    // Collisions for sides
    if (!pf_aabb_to_aabb(&r_box, &t_box, normal, penetration)) {
        return false;
    }

    {   // Collision against slope
        // rect projection
        const float a_max = dotv2f(t->proj, _v2f(r_box.max.x, r_box.max.y));    // dr
        // tri projection
        const float b_min = dotv2f(t->proj, _v2f(t_box.min.x, t_box.max.y));   // dl (ur works too)
        const float overlap = a_max - b_min; // slope penetration (rect's dr into slope)
        if (overlap <= 0) { // No overlap means no collision
            return false;
        }
        if (overlap < *penetration) {
            *penetration = overlap;
            *normal = t->normal;
        }
    }

    return *penetration > 0;
}

bool pf_rect_to_tri_ur(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_aabb r_box = pf_rect_to_aabb(&a->pos, &a->shape.radii);
    const pf_tri *t = &b->shape.tri;
    const pf_aabb t_box = pf_tri_to_aabb(&b->pos, t);

    // Collisions for sides
    if (!pf_aabb_to_aabb(&r_box, &t_box, normal, penetration)) {
        return false;
    }

    {   // Collision against slope
        // rect projection
        const float a_min = dotv2f(t->proj, _v2f(r_box.min.x, r_box.max.y));   // dl
        // tri projection
        const float b_max = dotv2f(t->proj, _v2f(t_box.max.x, t_box.max.y));   // dr (ul works too)
        const float overlap = b_max - a_min; // slope penetration (rect's dl into slope)
        if (overlap <= 0) { // No overlap means no collision
            return false;
        }
        if (overlap < *penetration) {
            *penetration = overlap;
            *normal = t->normal;
        }
    }

    return *penetration > 0;
}

bool pf_rect_to_tri_dl(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_aabb r_box = pf_rect_to_aabb(&a->pos, &a->shape.radii);
    const pf_tri *t = &b->shape.tri;
    const pf_aabb t_box = pf_tri_to_aabb(&b->pos, t);

    // Collisions for sides
    if (!pf_aabb_to_aabb(&r_box, &t_box, normal, penetration)) {
        return false;
    }

    {   // Collision against slope
        // rect projection
        const float a_max = dotv2f(t->proj, _v2f(r_box.max.x, r_box.min.y));   // ur
        // tri projection
        const float b_min = dotv2f(t->proj, _v2f(t_box.max.x, t_box.max.y));   // dr (ul works too)
        const float overlap = a_max - b_min; // slope penetration (rect's ur into slope)
        if (overlap <= 0) { // No overlap means no collision
            return false;
        }
        if (overlap < *penetration) {
            *penetration = overlap;
            *normal = t->normal;
        }
    }

    return *penetration > 0;
}

bool pf_rect_to_tri_dr(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    const pf_aabb r_box = pf_rect_to_aabb(&a->pos, &a->shape.radii);
    const pf_tri *t = &b->shape.tri;
    const pf_aabb t_box = pf_tri_to_aabb(&b->pos, t);

    // Collisions for sides
    if (!pf_aabb_to_aabb(&r_box, &t_box, normal, penetration)) {
        return false;
    }

    {   // Collision against slope
        // rect projection
        const float a_min = dotv2f(t->proj, _v2f(r_box.min.x, r_box.min.y));    // ul
        // tri projection
        const float b_max = dotv2f(t->proj, _v2f(t_box.min.x, t_box.max.y));   // dl (ur works too)
        const float overlap = b_max - a_min; // slope penetration (rect's ul into slope)
        if (overlap <= 0) { // No overlap means no collision
            return false;
        }
        if (overlap < *penetration) {
            *penetration = overlap;
            *normal = t->normal;
        }
    }
    return *penetration > 0;
}

bool pf_rect_to_tri(const pf_body *a, const pf_body *b, v2f *normal, float *penetration) {
    switch (b->shape.tri.hypotenuse) {
    case PF_CORNER_UL:
        return pf_rect_to_tri_ul(a, b, normal, penetration);
    case PF_CORNER_UR:
        return pf_rect_to_tri_ur(a, b, normal, penetration);
    case PF_CORNER_DL:
        return pf_rect_to_tri_dl(a, b, normal, penetration);
    case PF_CORNER_DR:
        return pf_rect_to_tri_dr(a, b, normal, penetration);
    default:
        assert(false);
    }
}

inline v2f pf_aabb_pos(const pf_aabb *a) {
    return divv2nf(addv2f(a->min, a->max), 2);
}

bool pf_solve_collision(const pf_body *a, const pf_body *b, pf_manifold *m) {
    if (pf_body_to_body(a, b, &m->normal, &m->penetration)) {
        if (fabsf(m->penetration) < 0.0001) {
            return false;
        }
        m->mixed_restitution = a->restitution * b->restitution;
        m->dynamic_friction = a->dynamic_friction * b->dynamic_friction;
        m->static_friction = a->static_friction * b->static_friction;
        return true;
    }
    return false;
}

v2f pf_gravity_v2f(pf_dir dir, float vel) {
    switch (dir) {
    case PF_DIR_U:
        return _v2f(0, -vel);
    case PF_DIR_D:
        return _v2f(0, vel);
    case PF_DIR_L:
        return _v2f(-vel, 0);
    case PF_DIR_R:
        return _v2f(vel, 0);
    default:
        assert(false);
    }
}

void pf_step_forces(float dt, pf_body *a) {
    if (!nearzerof(a->inverse_mass)) {
        a->in.impulse = mulv2f(a->in.impulse, a->in.decay);
        a->ex.impulse = mulv2f(a->ex.impulse, a->ex.decay);
        if (!a->group.object.parent) {
            a->gravity.vel = a->gravity.vel + (a->gravity.accel * dt / 2);
        } else {
            a->gravity.vel = 0;
        }
    } else {
        a->in.impulse = mulv2f(a->in.impulse, a->in.decay);
        a->ex.impulse = _v2f(0,0);
        a->gravity.vel = 0;
    }
}

void pf_update_dpos(float dt, pf_body *a) {
    if (!nearzerof(a->inverse_mass)) {
        a->in.impulse = clampv2f(sigv2f(a->in.cap), absv2f(a->in.cap), a->in.impulse);
        a->ex.impulse = clampv2f(sigv2f(a->ex.cap), absv2f(a->ex.cap), a->ex.impulse);
        a->gravity.vel = clampf(-a->gravity.cap, a->gravity.cap, a->gravity.vel);

        a->dpos = mulv2nf(a->in.impulse, dt);
        a->dpos = addv2f(a->dpos, mulv2nf(a->ex.impulse, dt));
        if (!a->group.object.parent) {
            a->dpos = addv2f(a->dpos, pf_gravity_v2f(a->gravity.dir, a->gravity.vel));
        }
    } else {
        a->in.impulse = clampv2f(sigv2f(a->in.cap), absv2f(a->in.cap), a->in.impulse);
        a->dpos = mulv2nf(a->in.impulse, dt);
    }
}

void pf_apply_dpos(pf_body *a) {
    a->pos = addv2f(a->pos, a->dpos);
}

void pf_pos_correction(const pf_manifold *m, pf_body *a,  pf_body *b) {
    float percent = 0.2;
    float slop = 0.01;
    float adjust = (m->penetration - slop) / (a->inverse_mass + b->inverse_mass);
    const v2f correction = mulv2nf(m->normal, fmaxf(0, adjust) * percent);
    a->pos = subv2f(a->pos, mulv2nf(correction, a->inverse_mass));
    b->pos = addv2f(b->pos, mulv2nf(correction, b->inverse_mass));
}

void pf_apply_manifold(const pf_manifold *m, pf_body *a, pf_body *b) {
    const float inverse_mass_sum = a->inverse_mass + b->inverse_mass;
    if (nearzerof(inverse_mass_sum)) {
        // TODO: verify this below
        a->ex.impulse = _v2f(0,0);
        b->ex.impulse = _v2f(0,0);
        return;
    }
    v2f rv = subv2f(b->ex.impulse, a->ex.impulse);
    const float contact_velocity = dotv2f(rv, m->normal);
    if (contact_velocity > 0) {
        return;
    }
    const float e = fminf(a->restitution, b->restitution);
    const float j = (-(1 + e) * contact_velocity) / inverse_mass_sum;
    const v2f impulse = mulv2nf(m->normal, j);
    a->ex.impulse = subv2f(a->ex.impulse, mulv2nf(impulse, a->inverse_mass));
    b->ex.impulse = addv2f(b->ex.impulse, mulv2nf(impulse, b->inverse_mass));
    rv = subv2f(b->ex.impulse, a->ex.impulse);
    const v2f t = normv2f(subv2f(rv, mulv2nf(m->normal, dotv2f(rv, m->normal))));
    const float jt = -dotv2f(rv,t) / inverse_mass_sum;
    if (nearzerof(jt)) {
        return;
    }
    float k = fabsf(jt) < (j *m->static_friction)
        ? jt
        : (-j *m->dynamic_friction);
    const v2f tagent_impulse = mulv2nf(t, k);
    a->ex.impulse = subv2f(a->ex.impulse, mulv2nf(tagent_impulse, a->inverse_mass));
    b->ex.impulse = addv2f(b->ex.impulse, mulv2nf(tagent_impulse, b->inverse_mass));
}

void pf_body_set_mass(float mass, pf_body *a) {
    a->mass = mass;
    a->inverse_mass = recipinff(mass);
}

float pf_mass_from_density(float density, const pf_shape shape) {
    switch (shape.tag) {
    case PF_SHAPE_RECT: return density * shape.radii.x * shape.radii.y;
    case PF_SHAPE_CIRCLE: return density * M_PI *shape.radius *shape.radius;
    default: assert(false);
    }
}

void pf_body_esque(float density, float restitution, pf_body *a) {
    pf_body_set_mass(pf_mass_from_density(density, a->shape), a);
    a->restitution = restitution;
}

void pf_rock_esque(pf_body *a) {
    pf_body_esque(0.6, 0.1, a);
}

void pf_wood_esque(pf_body *a) {
    pf_body_esque(0.3, 0.2, a);
}

void pf_metal_esque(pf_body *a) {
    pf_body_esque(1.2, 0.05, a);
}

void pf_bouncy_ball_esque(pf_body *a) {
    pf_body_esque(0.3, 0.8, a);
}

void pf_super_ball_esque(pf_body *a) {
    pf_body_esque(0.3, 0.95, a);
}

void pf_pillow_esque(pf_body *a) {
    pf_body_esque(0.1, 0.2, a);
}

void pf_static_esque(pf_body *a) {
    pf_body_esque(0, 0.4, a);
}

float pf_tri_slope(const v2f *radii, pf_corner hypotenuse) {
    switch (hypotenuse) {
    case PF_CORNER_UL:
    case PF_CORNER_DR:
        return -radii->y / radii->x;
    case PF_CORNER_UR:
    case PF_CORNER_DL:
        return radii->y / radii->x;
    default:
        assert(false);
    }   
}

// Returns a normal facing inwards
v2f tri_normal(const v2f *radii, pf_corner hypotenuse) {
    switch (hypotenuse) {
    case PF_CORNER_UL:
        return normv2f(_v2f(radii->y, radii->x));
    case PF_CORNER_UR:
        return normv2f(_v2f(-radii->y, radii->x));
    case PF_CORNER_DL:
        return normv2f(_v2f(radii->y, -radii->x));
    case PF_CORNER_DR:
        return normv2f(_v2f(-radii->y, -radii->x));
    default:
        assert(false);
    }
}

pf_tri _pf_tri(v2f radii, pf_corner hypotenuse) {
    return (pf_tri) {
        .radii = radii,
        .hypotenuse = hypotenuse,
        .radians = tri_angle(&radii, hypotenuse),
        .m = pf_tri_slope(&radii, hypotenuse),
        .proj = projection_vector(tri_angle(&radii, hypotenuse)),
        .normal = tri_normal(&radii, hypotenuse),
    };
}

pf_body _pf_body() {
    return (pf_body) {
        .mode = PF_MODE_DYNAMIC,
        .shape = (pf_shape) {
                .tag = PF_SHAPE_CIRCLE,
                .radius = 0
            },
        .pos = _v2f(0,0),
        .group.object.parent = NULL,
        .dpos = _v2f(0,0),
        .in = { 
            .impulse = fillv2f(0),
            .decay = fillv2f(0.5),
            .cap = fillv2f(1000),
        },
        .ex = {
            .impulse = fillv2f(0),
            .decay = fillv2f(0.3),
            .cap = fillv2f(1000),
        },
        .gravity = {
            .dir = PF_DIR_D,
            .vel = 0,
            .accel = 9.8,
            .cap = 10,
        },
        .mass = 1,
        .inverse_mass = 1,
        .static_friction = 0.9,
        .dynamic_friction = 0.7,
        .restitution = 0.5,
    };
}

pf_shape pf_circle(float radius) {
    return (pf_shape) {
        .tag = PF_SHAPE_CIRCLE,
        .radius = radius,
    };
}

pf_shape pf_box(float side) {
    return (pf_shape) {
        .tag = PF_SHAPE_RECT,
        .radii = _v2f(side, side),
    };
}

pf_shape pf_rect(float w, float h) {
    return (pf_shape) {
        .tag = PF_SHAPE_RECT,
        .radii = _v2f(w, h),
    };
}
