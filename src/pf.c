#include "pf.h"
#include <math.h>
#include <assert.h>

bool pf_intersect(const pf_aabb_t *a, const pf_aabb_t *b) {
    return
        a->max.x >= b->min.x && a->min.x <= b->max.x &&
        a->max.y >= b->min.y && a->min.y <= b->max.y;
}

bool pf_inside(const v2f *a, const pf_aabb_t *b) {
    return
        a->x >= b->min.x && a->x <= b->max.x &&
        a->y >= b->min.y && a->y <= b->max.y;
}

pf_aabb_t pf_rect_to_aabb(const pf_body_t *a);
pf_aabb_t pf_circle_to_aabb(const pf_body_t *a);

pf_aabb_t pf_body_to_aabb(const pf_body_t *a) {
    switch (a->shape.tag) {
    case PF_SH_RECT: return pf_rect_to_aabb(a);
    case PF_SH_CIRCLE: return pf_circle_to_aabb(a);
    default: assert(false);
    }
}

pf_aabb_t pf_rect_to_aabb(const pf_body_t *a) {
    return (pf_aabb_t) {
        .min = subv2f(a->position, a->shape.radii),
        .max = addv2f(a->position, a->shape.radii)
    };
}

pf_aabb_t pf_circle_to_aabb(const pf_body_t *a) {
    return (pf_aabb_t) {
        .min = subv2nf(a->position, a->shape.radius),
        .max = divv2nf(a->position, a->shape.radius)
    };
}

bool pf_test_rect(const pf_aabb_t *a, const pf_body_t *b);
bool pf_test_circle(const pf_aabb_t *a, const pf_body_t *b);

bool pf_test_body(const pf_aabb_t *a, const pf_body_t *b) {
    switch (b->shape.tag) {
    case PF_SH_RECT: return pf_test_rect(a, b);
    case PF_SH_CIRCLE: return pf_test_circle(a, b);
    default: assert(false);
    }
}

bool pf_test_rect(const pf_aabb_t *a, const pf_body_t *b) {
    const pf_aabb_t rect = pf_rect_to_aabb(b);
    return pf_intersect(a, &rect);
}

bool pf_test_circle(const pf_aabb_t *a, const pf_body_t *b) {
    const bool inside = pf_inside(&b->position, a);
    const v2f closest = clampv2f(a->min, a->max, b->position);
    const v2f pos_diff = subv2f(b->position, pf_aabb_position(a));
    float radius = b->shape.radius;
    const v2f normal = subv2f(pos_diff, closest);
    float dist_sq = sqlenv2f(normal);
    return inside || dist_sq <= (radius *radius);
}

bool pf_body_to_body_swap(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration);
bool pf_rect_to_rect(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration);
bool pf_rect_to_circle(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration);
bool pf_circle_to_circle(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration);

bool pf_body_to_body(const pf_body_t *a, const pf_body_t *b,
             v2f *normal, float *penetration) {
    switch (a->shape.tag) {
    case PF_SH_RECT:
        switch (b->shape.tag) {
        case PF_SH_RECT:
            return pf_rect_to_rect(a, b, normal, penetration);
        case PF_SH_CIRCLE:
            return pf_rect_to_circle(a, b, normal, penetration);
        default: assert(false);
        }
    case PF_SH_CIRCLE:
        switch (b->shape.tag) {
        case PF_SH_RECT:
            return pf_body_to_body_swap(a, b, normal, penetration);
        case PF_SH_CIRCLE:
            return pf_circle_to_circle(a, b, normal, penetration);
        default: assert(false);
        }
    default: assert(false);
    }
}

bool pf_body_to_body_swap(const pf_body_t *a, const pf_body_t *b,
              v2f *normal, float *penetration) {
    const bool test = pf_body_to_body(b, a, normal, penetration);
    if (test) {
        *normal = negv2f(*normal);
    }
    return test;
}

bool pf_rect_to_rect(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration) {
    const v2f n = subv2f(b->position, a->position);
    const v2f overlap = subv2f(addv2f(a->shape.radii, b->shape.radii), absv2f(n));
    const pf_aabb_t a_shape = pf_rect_to_aabb(a);
    const pf_aabb_t b_shape = pf_rect_to_aabb(b);
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

bool pf_rect_to_circle(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration) {
    const bool out_lf = b->position.x < a->position.x - a->shape.radii.x;
    const bool out_rt = b->position.x > a->position.x + a->shape.radii.x;
    const bool out_up = b->position.y < a->position.y - a->shape.radii.y;
    const bool out_dn = b->position.y > a->position.y + a->shape.radii.y;
    if ((out_lf || out_rt) && (out_up || out_dn)) {
        /* Treat as (circle/corner_point)_to_circle collision */
        pf_body_t a_ = *a;
        a_.shape.tag = PF_SH_CIRCLE;
        a_.shape.radius = 0;
        a_.position.x += out_lf ? -a->shape.radii.x : a->shape.radii.x;
        a_.position.y += out_up ? -a->shape.radii.y : a->shape.radii.y;
        return pf_circle_to_circle(&a_, b, normal, penetration);
    } else {
        /* Treat as pf_rect_to_rect collision */
        pf_body_t b_ = *b;
        b_.shape.tag = PF_SH_RECT;
        b_.shape.radii = _v2f(b->shape.radius, b->shape.radius);
        return pf_rect_to_rect(a, &b_, normal, penetration);
    }
}

bool pf_circle_to_circle(const pf_body_t *a, const pf_body_t *b, v2f *normal, float *penetration) {
    const v2f n = subv2f(b->position, a->position);
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

inline v2f pf_aabb_position(const pf_aabb_t *a) {
    return divv2nf(addv2f(a->min, a->max), 2);
}

bool pf_solve_collision(const pf_body_t *a, const pf_body_t *b, pf_manifold_t *m) {
    if (pf_body_to_body(a, b, &m->normal, &m->penetration)) {
        // /*
        if (fabsf(m->penetration) < 0.0001) {
            return false;
        }
        // */
        m->mixed_restitution = a->restitution * b->restitution;
        m->dynamic_friction = a->dynamic_friction * b->dynamic_friction;
        m->static_friction = a->static_friction * b->static_friction;
        return true;
    }
    return false;
}

void pf_integrate_force(float dt, pf_body_t *a) {
    if (!nearzerof(a->inverse_mass)) {
        const v2f velocity = mulv2nf(
            addv2f(mulv2nf(a->force, a->inverse_mass),
                   a->enable_gravity ? a->gravity : _v2f(0,0)),
            dt / 2
        );
        a->velocity = addv2f(a->velocity, velocity);
    } else {
        // Allows moving static bodies
        a->velocity = mulv2nf(a->force, dt);
        a->position = addv2f(a->velocity, a->position);
    }
}

void pf_integrate_velocity(float dt, pf_body_t *a) {
    if (!nearzerof(a->inverse_mass)) {
        const v2f position = mulv2nf(a->velocity, dt);
        a->position = addv2f(a->position, position);
        //a->position = addv2f(a->position, mulv2nf(a->parent_velocity, dt));
        pf_integrate_force(dt, a);
    }
}

void pf_manifold_initialize(const pf_body_t *a, const pf_body_t *b, pf_manifold_t *m) {
    m->mixed_restitution = a->restitution *b->restitution;
    m->static_friction = a->static_friction *b->static_friction;
    m->dynamic_friction = a->dynamic_friction *b->dynamic_friction;
}

void pf_positional_correction(const pf_manifold_t *m, pf_body_t *a,  pf_body_t *b) {
    float percent = 0.4;
    float slop = 0.05;
    float adjust = (m->penetration - slop) /
                 (a->inverse_mass + b->inverse_mass);
    const v2f correction = mulv2nf(m->normal, fmaxf(0, adjust) * percent);
    a->position = subv2f(a->position, mulv2nf(correction, a->inverse_mass));
    b->position = addv2f(b->position, mulv2nf(correction, b->inverse_mass));
}

void pf_manifold_apply_impulse(const pf_manifold_t *m, pf_body_t *a, pf_body_t *b) {
    float inverse_massSum = a->inverse_mass + b->inverse_mass;
    if (nearzerof(inverse_massSum)) {
        a->velocity = _v2f(0,0);
        b->velocity = _v2f(0,0);
        return;
    }

    v2f rv = subv2f(b->velocity, a->velocity);
    float contactVelocity = dotv2f(rv, m->normal);
    if (contactVelocity > 0)
        return;

    float e = fminf(a->restitution, b->restitution);
    float j = (-(1 + e) *contactVelocity) / inverse_massSum;
    const v2f impulse = mulv2nf(m->normal, j);
    a->velocity = subv2f(a->velocity, mulv2nf(impulse, a->inverse_mass));
    b->velocity = addv2f(b->velocity, mulv2nf(impulse, b->inverse_mass));
    rv = subv2f(b->velocity, a->velocity);
    const v2f t = normv2f(subv2f(rv, mulv2nf(m->normal,
                         dotv2f(rv, m->normal))));
    float jt = -dotv2f(rv,t) / inverse_massSum;
    if (nearzerof(jt))
        return;

    float k = fabsf(jt) < (j *m->static_friction)
        ? jt
        : (-j *m->dynamic_friction);
    const v2f tagentImpulse = mulv2nf(t, k);
    a->velocity = subv2f(a->velocity,
                 mulv2nf(tagentImpulse, a->inverse_mass));
    b->velocity = addv2f(b->velocity,
                 mulv2nf(tagentImpulse, b->inverse_mass));
}

void pf_body_set_mass(float mass, pf_body_t *a) {
    a->mass = mass;
    a->inverse_mass = recipinff(mass);
}

float pf_mass_from_density(float density, const pf_shape_t shape) {
    switch (shape.tag) {
    case PF_SH_RECT: return density * shape.radii.x * shape.radii.y;
    case PF_SH_CIRCLE: return density * M_PI *shape.radius *shape.radius;
    default: assert(false);
    }
}

void pf_body_esque(float density, float restitution, pf_body_t *a) {
    pf_body_set_mass(pf_mass_from_density(density, a->shape), a);
    a->restitution = restitution;
}

void pf_rock_esque(pf_body_t *a) {
    pf_body_esque(0.6, 0.1, a);
}

void pf_wood_esque(pf_body_t *a) {
    pf_body_esque(0.3, 0.2, a);
}

void pf_metal_esque(pf_body_t *a) {
    pf_body_esque(1.2, 0.05, a);
}

void pf_bouncy_ball_esque(pf_body_t *a) {
    pf_body_esque(0.3, 0.8, a);
}

void pf_super_ball_esque(pf_body_t *a) {
    pf_body_esque(0.3, 0.95, a);
}

void pf_pillow_esque(pf_body_t *a) {
    pf_body_esque(0.1, 0.2, a);
}

void pf_static_esque(pf_body_t *a) {
    pf_body_esque(0, 0.4, a);
}

pf_body_t _pf_body() {
    return (pf_body_t) {
        .mode = PF_BM_DYNAMIC,
        .shape = (pf_shape_t) {
                .tag = PF_SH_CIRCLE,
                .radius = 0
            },
        .position = _v2f(0,0),
        .velocity = _v2f(0,0),
         //.parent_velocity = _v2f(0,0),
        .parent = NULL,
        .force = _v2f(0,0),
        .gravity = _v2f(0,9.8),
        .enable_gravity = true,
        .mass = 1,
        .inverse_mass = 1,
        .static_friction = 0.9,
        .dynamic_friction = 0.7,
        .restitution = 0.5,
    };
}

pf_shape_t pf_circle(float radius) {
    return (pf_shape_t) {
        .tag = PF_SH_CIRCLE,
        .radius = radius,
    };
}

pf_shape_t pf_box(float side) {
    return (pf_shape_t) {
        .tag = PF_SH_RECT,
        .radii = _v2f(side, side),
    };
}

pf_shape_t pf_rect(float w, float h) {
    return (pf_shape_t) {
        .tag = PF_SH_RECT,
        .radii = _v2f(w, h),
    };
}
