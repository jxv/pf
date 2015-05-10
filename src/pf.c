#include "pf.h"
#include <math.h>
#include <assert.h>

bool pf_intersect(const struct pf_aabb a, const struct pf_aabb b) {
	return
		a.max.x >= b.min.x && a.min.x <= b.max.x &&
		a.max.y >= b.min.y && a.min.y <= b.max.y;
}

bool pf_inside(const v2f a, const struct pf_aabb b) {
	return
		a.x >= b.min.x && a.x <= b.max.x &&
		a.y >= b.min.y && a.y <= b.max.y;
}

struct pf_aabb pf_rect_to_aabb(const struct pf_body *a);
struct pf_aabb pf_circle_to_aabb(const struct pf_body *a);

struct pf_aabb pf_body_to_aabb(const struct pf_body *a) {
	switch (a->shape.tag) {
	case PF_SH_RECT: return pf_rect_to_aabb(a);
	case PF_SH_CIRCLE: return pf_circle_to_aabb(a);
	default: assert(false);
	}
}

struct pf_aabb pf_rect_to_aabb(const struct pf_body *a) {
	return (struct pf_aabb) {
		.min = subv2f(a->position, a->shape.radii),
		.max = addv2f(a->position, a->shape.radii)
	};
}

struct pf_aabb pf_circle_to_aabb(const struct pf_body *a) {
	return (struct pf_aabb) {
		.min = subv2fs(a->position, a->shape.radius),
		.max = divv2fs(a->position, a->shape.radius)
	};
}

bool pf_test_rect(const struct pf_aabb a, const struct pf_body *b);
bool pf_test_circle(const struct pf_aabb a, const struct pf_body *b);

bool pf_test_body(const struct pf_aabb a, const struct pf_body *b) {
	switch (b->shape.tag) {
	case PF_SH_RECT: return pf_test_rect(a, b);
	case PF_SH_CIRCLE: return pf_test_circle(a, b);
	default: assert(false);
	}
}

bool pf_test_rect(const struct pf_aabb a, const struct pf_body *b) {
	return pf_intersect(a, pf_rect_to_aabb(b));
}

bool pf_test_circle(const struct pf_aabb a, const struct pf_body *b) {
	const bool inside = pf_inside(b->position, a);
	const v2f closest = clampv2f(a.min, a.max, b->position);
	const v2f pos_diff = subv2f(b->position, pf_aabb_position(a));
	const float radius = b->shape.radius;
	const v2f normal = subv2f(pos_diff, closest);
	const float dist_sq = sqlenv2f(normal);
	return inside || dist_sq <= (radius *radius);
}

bool pf_body_to_body_swap(const struct pf_body *a, const struct pf_body *b,
			  v2f *normal, float *penetration);
bool pf_rect_to_rect(const struct pf_body *a, const struct pf_body *b,
		     v2f *normal, float *penetration);
bool pf_rect_to_circle(const struct pf_body *a, const struct pf_body *b,
		       v2f *normal, float *penetration);
bool pf_circle_to_circle(const struct pf_body *a, const struct pf_body *b,
			 v2f *normal, float *penetration);

bool pf_body_to_body(const struct pf_body *a, const struct pf_body *b,
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

bool pf_body_to_body_swap(const struct pf_body *a, const struct pf_body *b,
			  v2f *normal, float *penetration) {
	bool test = pf_body_to_body(b, a, normal, penetration);
	if (test)
		*normal = negv2f(*normal);
	return test;
}

bool pf_rect_to_rect(const struct pf_body *a, const struct pf_body *b,
		     v2f *normal, float *penetration) {
	const v2f n = subv2f(b->position, a->position);
	const v2f overlap = subv2f(addv2f(a->shape.radii, b->shape.radii),
				   absv2f(n));
	if (!pf_intersect(pf_rect_to_aabb(a), pf_rect_to_aabb(b))) {
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

bool pf_rect_to_circle(const struct pf_body *a, const struct pf_body *b,
		       v2f *normal, float *penetration) {
	const bool out_lf = b->position.x < a->position.x - a->shape.radii.x;
	const bool out_rt = b->position.x > a->position.x + a->shape.radii.x;
	const bool out_up = b->position.y < a->position.y - a->shape.radii.y;
	const bool out_dn = b->position.y > a->position.y + a->shape.radii.y;
	if ((out_lf || out_rt) && (out_up || out_dn)) {
		/* Treat as (circle/corner_point)_to_circle collision */
		struct pf_body a_ = *a;
		a_.shape.tag = PF_SH_CIRCLE;
		a_.shape.radius = 0;
		a_.position.x += out_lf ? -a->shape.radii.x : a->shape.radii.x;
		a_.position.y += out_up ? -a->shape.radii.y : a->shape.radii.y;
		return pf_circle_to_circle(&a_, b, normal, penetration);
	} else {
		/* Treat as pf_rect_to_rect collision */
		struct pf_body b_ = *b;
		b_.shape.tag = PF_SH_RECT;
		b_.shape.radii = _v2f(b->shape.radius, b->shape.radius);
		return pf_rect_to_rect(a, &b_, normal, penetration);
	}
}

bool pf_circle_to_circle(const struct pf_body *a, const struct pf_body *b,
			 v2f *normal, float *penetration) {
	const v2f n = subv2f(b->position, a->position);
	const float dist_sq = sqlenv2f(n);
	const float dist = sqrtf(dist_sq);
	const float radius = a->shape.radius + b->shape.radius;
	if (dist_sq >= radius *radius) {
		return false;
	} else if (dist == 0) {
		*penetration = a->shape.radius;
		*normal = _v2f(1,0);
	} else {
		*penetration = radius - dist;
		*normal = divv2fs(n, dist);
	}
	return true;
}

inline v2f pf_aabb_position(const struct pf_aabb a) {
	return divv2fs(addv2f(a.min, a.max), 2);
}

bool pf_solve_collision(const struct pf_body *a, const struct pf_body *b,
			struct pf_manifold *m) {
	if (pf_body_to_body(a, b, &m->normal, &m->penetration)) {
		m->mixed_restitution = a->restitution *b->restitution;
		m->dynamic_friction = a->dynamic_friction * b->dynamic_friction;
		m->static_friction = a->static_friction * b->static_friction;
		return true;
	}
	return false;
}

void pf_integrate_force(const float dt, struct pf_body *a) {
	if (!nearzerof(a->inverse_mass)) {
		const v2f velocity = mulv2fs(
			addv2f(mulv2fs(a->force, a->inverse_mass), a->gravity),
			dt / 2
		);
		a->velocity = addv2f(a->velocity, velocity);
	} else {
		// Allows moving static bodies
		a->velocity = mulv2fs(a->force, dt);
		a->position = addv2f(a->velocity, a->position);
	}
}

void pf_integrate_velocity(const float dt, struct pf_body *a) {
	if (!nearzerof(a->inverse_mass)) {
		const v2f position = mulv2fs(a->velocity, dt);
		a->position = addv2f(a->position, position);
		a->position = addv2f(a->position, mulv2fs(a->parent_velocity, dt));
		pf_integrate_force(dt, a);
	}
}

void pf_manifold_initialize(const struct pf_body *a, const struct pf_body *b,
			    struct pf_manifold *m) {
	m->mixed_restitution = a->restitution *b->restitution;
	m->static_friction = a->static_friction *b->static_friction;
	m->dynamic_friction = a->dynamic_friction *b->dynamic_friction;
}

void pf_positional_correction(const struct pf_manifold *m, struct pf_body *a,
			      struct pf_body *b) {
	const float percent = 0.4;
	const float slop = 0.05;
	const float adjust = (m->penetration - slop) /
			     (a->inverse_mass + b->inverse_mass);
	const v2f correction = mulv2fs(m->normal, fmaxf(0, adjust) * percent);
	a->position = subv2f(a->position, mulv2fs(correction, a->inverse_mass));
	b->position = addv2f(b->position, mulv2fs(correction, b->inverse_mass));
}

void pf_manifold_apply_impulse(const struct pf_manifold *m, struct pf_body *a,
			       struct pf_body *b) {
	const float inverse_massSum = a->inverse_mass + b->inverse_mass;
	if (nearzerof(inverse_massSum)) {
		a->velocity = _v2f(0,0);
		b->velocity = _v2f(0,0);
		return;
	}

	v2f rv = subv2f(b->velocity, a->velocity);
	const float contactVelocity = dotv2f(rv, m->normal);
	if (contactVelocity > 0)
		return;

	const float e = fminf(a->restitution, b->restitution);
	const float j = (-(1 + e) *contactVelocity) / inverse_massSum;
	const v2f impulse = mulv2fs(m->normal, j);
	a->velocity = subv2f(a->velocity, mulv2fs(impulse, a->inverse_mass));
	b->velocity = addv2f(b->velocity, mulv2fs(impulse, b->inverse_mass));
	rv = subv2f(b->velocity, a->velocity);
	const v2f t = normv2f(subv2f(rv, mulv2fs(m->normal,
						 dotv2f(rv, m->normal))));
	const float jt = -dotv2f(rv,t) / inverse_massSum;
	if (nearzerof(jt))
		return;

	const float k = fabsf(jt) < (j *m->static_friction)
		? jt
		: (-j *m->dynamic_friction);
	const v2f tagentImpulse = mulv2fs(t, k);
	a->velocity = subv2f(a->velocity,
			     mulv2fs(tagentImpulse, a->inverse_mass));
	b->velocity = addv2f(b->velocity,
			     mulv2fs(tagentImpulse, b->inverse_mass));
}

void pf_body_set_mass(const float mass, struct pf_body *a) {
	a->mass = mass;
	a->inverse_mass = recipinff(mass);
}

float pf_mass_from_density(const float density, const struct pf_shape shape) {
	switch (shape.tag) {
	case PF_SH_RECT: return density *shape.radii.x *shape.radii.y;
	case PF_SH_CIRCLE: return density *M_PI *shape.radius *shape.radius;
	default: assert(false);
	}
}

void pf_body_esque(const float density, const float restitution,
		   struct pf_body *a) {
	pf_body_set_mass(pf_mass_from_density(density, a->shape), a);
	a->restitution = restitution;
}

void pf_rock_esque(struct pf_body *a) {
	pf_body_esque(0.6, 0.1, a);
}

void pf_wood_esque(struct pf_body *a) {
	pf_body_esque(0.3, 0.2, a);
}

void pf_metal_esque(struct pf_body *a) {
	pf_body_esque(1.2, 0.05, a);
}

void pf_bouncy_ball_esque(struct pf_body *a) {
	pf_body_esque(0.3, 0.8, a);
}

void pf_super_ball_esque(struct pf_body *a) {
	pf_body_esque(0.3, 0.95, a);
}

void pf_pillow_esque(struct pf_body *a) {
	pf_body_esque(0.1, 0.2, a);
}

void pf_static_esque(struct pf_body *a) {
	pf_body_esque(0, 0.4, a);
}

struct pf_body _pf_body() {
	return (struct pf_body) {
		.mode = PF_BM_DYNAMIC,
		.shape = (struct pf_shape) {
				.tag = PF_SH_CIRCLE,
				.radius = 0
			},
		.position = _v2f(0,0),
		.velocity = _v2f(0,0),
		.parent_velocity = _v2f(0,0),
		.force = _v2f(0,0),
		.gravity = _v2f(0,9.8),
		.mass = 1,
		.inverse_mass = 1,
		.static_friction = 0.5,
		.dynamic_friction = 0.5,
		.restitution = 0.5,
	};
}

struct pf_shape pf_circle(const float radius) {
	return (struct pf_shape) {
		.tag = PF_SH_CIRCLE,
		.radius = radius,
	};
}

struct pf_shape pf_box(const float side) {
	return (struct pf_shape) {
		.tag = PF_SH_RECT,
		.radii = _v2f(side, side),
	};
}

struct pf_shape pf_rect(const float w, const float h) {
	return (struct pf_shape) {
		.tag = PF_SH_RECT,
		.radii = _v2f(w, h),
	};
}
