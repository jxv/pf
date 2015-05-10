#ifndef PF_H
#define PF_H

#include <stdlib.h>
#include <stdbool.h>
#include <ml.h>

struct pf_aabb {
	v2f min;
	v2f max;
};

enum pf_shape_tag {
	PF_SH_RECT,
	PF_SH_CIRCLE,
};

struct pf_shape {
	enum pf_shape_tag tag;
	union {
		v2f radii;
		float radius;
	};
};

enum pf_body_mode {
	PF_BM_STATIC,
	PF_BM_DYNAMIC,
};

struct pf_body {
	enum pf_body_mode mode;
	struct pf_shape shape;
	v2f position;
	v2f velocity;
	v2f parent_velocity;
	v2f force;
	v2f gravity;
	float mass;
	float inverse_mass;
	float static_friction;
	float dynamic_friction;
	float restitution;
};

struct pf_manifold {
	v2f normal;
	float penetration;
	float mixed_restitution;
	float dynamic_friction;
	float static_friction;
};

bool pf_intersect(const struct pf_aabb a, const struct pf_aabb b);
bool pf_inside(v2f a, const struct pf_aabb b);
v2f pf_aabb_position(const struct pf_aabb a);
struct pf_aabb pf_body_to_aabb(const struct pf_body *a);
bool pf_test_body(const struct pf_aabb a, const struct pf_body *b);
bool pf_body_to_body(const struct pf_body *a, const struct pf_body *b,
		     v2f *normal, float *penetration);
bool pf_solve_collision(const struct pf_body *a, const struct pf_body *b,
			struct pf_manifold *m);
void pf_integrate_force(const float dt, struct pf_body *a);
void pf_integrate_velocity(const float dt, struct pf_body *a);
void pf_manifold_initialize(const struct pf_body *a, const struct pf_body *b,
			    struct pf_manifold *m);
void pf_positional_correction(const struct pf_manifold *m, struct pf_body *a,
			      struct pf_body *b);
void pf_manifold_apply_impulse(const struct pf_manifold *m, struct pf_body *a,
			       struct pf_body *b);

void pf_body_set_mass(const float mass, struct pf_body *a);
void pf_body_esque(const float density, const float restitution,
		   struct pf_body *a);
void pf_rock_esque(struct pf_body *a);
void pf_wood_esque(struct pf_body *a);
void pf_metal_esque(struct pf_body *a);
void pf_bouncy_ball_esque(struct pf_body *a);
void pf_super_ball_esque(struct pf_body *a);
void pf_pillow_esque(struct pf_body *a);
void pf_static_esque(struct pf_body *a);

struct pf_body _pf_body();
struct pf_shape pf_circle(const float radius);
struct pf_shape pf_box(const float side);
struct pf_shape pf_rect(const float w, const float h);

#endif
