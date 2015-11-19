#include "pf.h"
#include <math.h>
#include <assert.h>

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
        .max = divv2f(*pos, tri->radii)
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
bool pf_point_in_tri_ul(float x, float y, const v2f *dr, float offset, float slope) {
    return
        x <= dr->x &&
        y <= dr->y &&
        (x - offset) * slope <= (y - dr->y);
}
 
bool pf_test_tri_ul(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const float offset = pos->x - radii->x;
    const v2f ur = addv2f(*pos, _v2f( radii->x, -radii->y));
    const v2f dl = addv2f(*pos, _v2f(-radii->x,  radii->y));
    const v2f dr = addv2f(*pos, _v2f( radii->x,  radii->y));
   return
        pf_inside(&ur, a) ||
        pf_inside(&dl, a) ||
        pf_inside(&dr, a) ||
        pf_point_in_tri_ul(a->min.x, a->min.y, &dr, offset, slope) ||
        pf_point_in_tri_ul(a->max.x, a->min.y, &dr, offset, slope) ||
        pf_point_in_tri_ul(a->min.x, a->max.y, &dr, offset, slope) ||
        pf_point_in_tri_ul(a->max.x, a->max.y, &dr, offset, slope);
}

inline
bool pf_point_in_tri_ur(float x, float y, const v2f *dl, float offset, float slope) {
    return
        x >= dl->x &&
        y <= dl->y &&
        (x - offset) * slope <= (y - dl->y);
}
 
bool pf_test_tri_ur(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const float offset = pos->x - radii->x;
    const v2f ul = addv2f(*pos, _v2f(-radii->x, -radii->y));
    const v2f dl = addv2f(*pos, _v2f(-radii->x,  radii->y));
    const v2f dr = addv2f(*pos, _v2f( radii->x,  radii->y));
   return
        pf_inside(&ul, a) ||
        pf_inside(&dl, a) ||
        pf_inside(&dr, a) ||
        pf_point_in_tri_ur(a->min.x, a->min.y, &dl, offset, slope) ||
        pf_point_in_tri_ur(a->max.x, a->min.y, &dl, offset, slope) ||
        pf_point_in_tri_ur(a->min.x, a->max.y, &dl, offset, slope) ||
        pf_point_in_tri_ur(a->max.x, a->max.y, &dl, offset, slope);
}

inline
bool pf_point_in_tri_dl(float x, float y, const v2f *ur, float offset, float slope) {
    return
        x <= ur->x &&
        y >= ur->y &&
        (x - offset) * slope >= (y - ur->y);
}
 
bool pf_test_tri_dl(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const float offset = pos->x - radii->x;
    const v2f ul = addv2f(*pos, _v2f(-radii->x, -radii->y));
    const v2f ur = addv2f(*pos, _v2f( radii->x, -radii->y));
    const v2f dl = addv2f(*pos, _v2f(-radii->x,  radii->y));
    const v2f dr = addv2f(*pos, _v2f( radii->x,  radii->y));
   return
        pf_inside(&ul, a) ||
        pf_inside(&dl, a) ||
        pf_inside(&dr, a) ||
        pf_point_in_tri_dl(a->min.x, a->min.y, &ur, offset, slope) ||
        pf_point_in_tri_dl(a->max.x, a->min.y, &ur, offset, slope) ||
        pf_point_in_tri_dl(a->min.x, a->max.y, &ur, offset, slope) ||
        pf_point_in_tri_dl(a->max.x, a->max.y, &ur, offset, slope);
}

inline
bool pf_point_in_tri_dr(float x, float y, const v2f *ul, float offset, float slope) {
    return
        x >= ul->x &&
        y >= ul->y &&
        (x - offset) * slope >= (y - ul->y);
}
 
bool pf_test_tri_dr(const pf_aabb *a, const v2f *pos, const v2f *radii, float slope) {
    const float offset = pos->x - radii->x;
    const v2f ul = addv2f(*pos, _v2f(-radii->x, -radii->y));
    const v2f ur = addv2f(*pos, _v2f( radii->x, -radii->y));
    const v2f dl = addv2f(*pos, _v2f(-radii->x,  radii->y));
   return
        pf_inside(&ul, a) ||
        pf_inside(&ur, a) ||
        pf_inside(&dl, a) ||
        pf_point_in_tri_dr(a->min.x, a->min.y, &ul, offset, slope) ||
        pf_point_in_tri_dr(a->max.x, a->min.y, &ul, offset, slope) ||
        pf_point_in_tri_dr(a->min.x, a->max.y, &ul, offset, slope) ||
        pf_point_in_tri_dr(a->max.x, a->max.y, &ul, offset, slope);
}

bool pf_test_tri(const pf_aabb *a, const v2f *pos, const pf_tri *t) {
    switch (t->hypotenuse) {
    case PF_CORNER_UL:
        return pf_test_tri_ul(a, pos, &t->radii, t->slope);
    case PF_CORNER_UR:
        return pf_test_tri_ur(a, pos, &t->radii, t->slope);
    case PF_CORNER_DL:
        return pf_test_tri_dl(a, pos, &t->radii, t->slope);
    case PF_CORNER_DR:
        return pf_test_tri_dr(a, pos, &t->radii, t->slope);
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
bool pf_circle_to_circle(const pf_body *a, const pf_body *b, v2f *normal, float *penetration);

bool pf_body_to_body(const pf_body *a, const pf_body *b,
             v2f *normal, float *penetration) {
    switch (a->shape.tag) {
    case PF_SHAPE_RECT:
        switch (b->shape.tag) {
        case PF_SHAPE_RECT:
            return pf_rect_to_rect(a, b, normal, penetration);
        case PF_SHAPE_CIRCLE:
            return pf_rect_to_circle(a, b, normal, penetration);
        default:
            assert(false);
        }
    case PF_SHAPE_CIRCLE:
        switch (b->shape.tag) {
        case PF_SHAPE_RECT:
            return pf_body_to_body_swap(a, b, normal, penetration);
        case PF_SHAPE_CIRCLE:
            return pf_circle_to_circle(a, b, normal, penetration);
        default:
            assert(false);
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

inline v2f pf_aabb_pos(const pf_aabb *a) {
    return divv2nf(addv2f(a->min, a->max), 2);
}

bool pf_solve_collision(const pf_body *a, const pf_body *b, pf_manifold *m) {
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

/*
void pf_integrate_force(float dt, pf_body *a) {
    if (!nearzerof(a->inverse_mass)) {
        const v2f velocity = mulv2nf(
            addv2f(mulv2nf(a->force, a->inverse_mass),
                   a->gravity.rate : _v2f(0,0)),
            dt / 2
        );
        a->velocity = a->parent ? velocity : addv2f(a->velocity, velocity);
    } else {
        // Allows moving static bodies
        a->velocity = mulv2nf(a->force, dt);
        a->pos = addv2f(a->velocity, a->pos);
    }
}
*/

/*
void pf_integrate_velocity(float dt, pf_body *a) {
    if (!nearzerof(a->inverse_mass)) {
        const v2f pos = mulv2nf(a->velocity, dt);
        a->pos = addv2f(a->pos, pos);
        pf_integrate_force(dt, a);
    }
}
*/

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

/*
void pf_manifold_apply_impulse(const pf_manifold *m, pf_body *a, pf_body *b) {
    const float inverse_massSum = a->inverse_mass + b->inverse_mass;
    if (nearzerof(inverse_massSum)) {
        a->velocity = _v2f(0,0);
        b->velocity = _v2f(0,0);
        return;
    }
    v2f rv = subv2f(b->velocity, a->velocity);
    const float contactVelocity = dotv2f(rv, m->normal);
    if (contactVelocity > 0) {
        return;
    }
    float e = fminf(a->restitution, b->restitution);
    float j = (-(1 + e) *contactVelocity) / inverse_massSum;
    const v2f impulse = mulv2nf(m->normal, j);
    a->velocity = subv2f(a->velocity, mulv2nf(impulse, a->inverse_mass));
    b->velocity = addv2f(b->velocity, mulv2nf(impulse, b->inverse_mass));
    rv = subv2f(b->velocity, a->velocity);
    const v2f t = normv2f(subv2f(rv, mulv2nf(m->normal, dotv2f(rv, m->normal))));
    const float jt = -dotv2f(rv,t) / inverse_massSum;
    if (nearzerof(jt)) {
        return;
    }
    float k = fabsf(jt) < (j *m->static_friction)
        ? jt
        : (-j *m->dynamic_friction);
    const v2f tagentImpulse = mulv2nf(t, k);
    a->velocity = subv2f(a->velocity, mulv2nf(tagentImpulse, a->inverse_mass));
    b->velocity = addv2f(b->velocity, mulv2nf(tagentImpulse, b->inverse_mass));
}
*/

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
        return radii->y / radii->x;
    case PF_CORNER_UR:
    case PF_CORNER_DL:
        return -radii->y / radii->x;
    default:
        assert(false);
    }   
}

pf_tri _pf_tri(v2f radii, pf_corner hypotenuse) {
    return (pf_tri) {
        .radii = radii,
        .hypotenuse = hypotenuse,
        .slope = pf_tri_slope(&radii, hypotenuse),
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
