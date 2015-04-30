#include "pf.h"
#include <assert.h>

bool pfIntersect(const PFAABB a, const PFAABB b) {
	return 
		a.max.x >= b.min.x && a.min.x <= b.max.x &&
		a.max.y >= b.min.y && a.min.y <= b.max.y; 
}

bool pfInside(const MLV2f a, const PFAABB b) {
	return 
		a.x >= b.min.x && a.x <= b.max.x &&
		a.y >= b.min.y && a.y <= b.max.y; 
}

PFAABB pfRectToAABB(const PFBody *a);
PFAABB pfCircleToAABB(const PFBody *a);

PFAABB pfBodyToAABB(const PFBody *a) {
	switch (a->shape.tag) {
	case PF_SH_RECT: return pfRectToAABB(a);
	case PF_SH_CIRCLE: return pfCircleToAABB(a);
	default: assert(false);
	}
}

PFAABB pfRectToAABB(const PFBody *a) {
	return (PFAABB) {
		.min = mlSubV2f(a->position, a->shape.radii),
		.max = mlAddV2f(a->position, a->shape.radii)
	};
}

PFAABB pfCircleToAABB(const PFBody *a) {
	return (PFAABB) {
		.min = mlSubV2ff(a->position, a->shape.radius),
		.max = mlDivV2ff(a->position, a->shape.radius)
	};
}

bool pfTestRect(const PFAABB a, const PFBody *b);
bool pfTestCircle(const PFAABB a, const PFBody *b);

bool pfTestBody(const PFAABB a, const PFBody *b) {
	switch (b->shape.tag) {
	case PF_SH_RECT: return pfTestRect(a, b);
	case PF_SH_CIRCLE: return pfTestCircle(a, b);
	default: assert(false);
	}
}

bool pfTestRect(const PFAABB a, const PFBody *b) {
	return pfIntersect(a, pfRectToAABB(b));
}

bool pfTestCircle(const PFAABB a, const PFBody *b) {
	const bool inside = pfInside(b->position, a);
	const MLV2f closest = mlClampV2f(a.min, a.max, b->position);
	const MLV2f pos_diff = mlSubV2f(b->position, pfAABBPosition(a));
	const float radius = b->shape.radius; 
	const MLV2f normal = mlSubV2f(pos_diff, closest);
	const float dist_sq = mlSqLenV2f(normal);
	return inside || dist_sq <= (radius * radius);
}

bool pfBodyToBodySwap(const PFBody *a, const PFBody *b, MLV2f *normal, float *penetration);
bool pfRectToRect(const PFBody *a, const PFBody *b, MLV2f *normal, float *penetration);
bool pfRectToCircle(const PFBody *a, const PFBody *b, MLV2f *normal, float *penetration);
bool pfCircleToCircle(const PFBody *a, const PFBody *b, MLV2f *normal, float *penetration);

bool pfBodyToBody(const PFBody *a, const PFBody *b, MLV2f *normal, float *penetration) {
	switch (a->shape.tag) {
	case PF_SH_RECT:
		switch (b->shape.tag) {
		case PF_SH_RECT: return pfRectToRect(a, b, normal, penetration);
		case PF_SH_CIRCLE: return pfRectToCircle(a, b, normal, penetration);
		default: assert(false);
		}
	case PF_SH_CIRCLE:
		switch (b->shape.tag) {
		case PF_SH_RECT: return pfBodyToBodySwap(a, b, normal, penetration);
		case PF_SH_CIRCLE: return pfCircleToCircle(a, b, normal, penetration);
		default: assert(false);
		}
	default: assert(false);
	}
}

bool pfBodyToBodySwap(const PFBody *a, const PFBody *b, MLV2f *normal, float *penetration) {
	bool test = pfBodyToBody(b, a, normal, penetration);
	if (test)
		*normal = mlNegateV2f(*normal);
	return test;
}

bool pfRectToRect(const PFBody *a, const PFBody *b, MLV2f * normal, float * penetration) {
	const MLV2f n = mlSubV2f(b->position, a->position);
	const MLV2f overlap = mlSubV2f(mlAddV2f(a->shape.radii, b->shape.radii), mlAbsV2f(n));
	if (!pfIntersect(pfRectToAABB(a), pfRectToAABB(b))) {
		return false;
	} else if (mlAbsf(overlap.x) < mlAbsf(overlap.y)) {
		*penetration = overlap.x;
		normal->x = n.x < 0 ? -1 : 1;
		normal->y = 0;
	} else {
		*penetration = overlap.y;
		normal->x = 0;
		normal->y = n.y < 0 ? -1 : 1;
	}
	return true;
}

bool pfRectToCircle(const PFBody *a, const PFBody *b, MLV2f * normal, float * penetration) {
	const bool out_lf = b->position.x < a->position.x - a->shape.radii.x;
	const bool out_rt = b->position.x > a->position.x + a->shape.radii.x;
	const bool out_up = b->position.y < a->position.y - a->shape.radii.y;
	const bool out_dn = b->position.y > a->position.y + a->shape.radii.y;
	if ((out_lf || out_rt) && (out_up || out_dn)) {
		/* Treat as (circle/corner_point)_to_circle collision */
		PFBody a_ = *a;
		a_.shape.tag = PF_SH_CIRCLE;
		a_.shape.radius = 0;
		a_.position.x += out_lf ? -a->shape.radii.x : a->shape.radii.x;
		a_.position.y += out_up ? -a->shape.radii.y : a->shape.radii.y;
		return  pfCircleToCircle(&a_, b, normal, penetration);
	} else {
		/* Treat as pfRectToRect collision */
		PFBody b_ = *b;
		b_.shape.tag = PF_SH_RECT;
		b_.shape.radii = mlV2f(b->shape.radius, b->shape.radius);
		return pfRectToRect(a, &b_, normal, penetration);
	}
}

bool pfCircleToCircle(const PFBody *a, const PFBody *b, MLV2f * normal, float * penetration) {
	const MLV2f n = mlSubV2f(b->position, a->position);
	const float dist_sq = mlSqLenV2f(n);
	const float dist = mlSqrtf(dist_sq);
	const float radius = a->shape.radius + a->shape.radius;
	if (dist_sq >= radius * radius) {
		return false;
	} else if (dist == 0) {
		*penetration = a->shape.radius;
		*normal = mlV2f(1,0);
	} else {
		*penetration = radius - dist;
		*normal = mlDivV2ff(n, dist);
	}
	return true;
}

inline MLV2f pfAABBPosition(const PFAABB a) {
	return mlDivV2ff(mlAddV2f(a.min, a.max), 2);
}

bool pfSolveCollision(const PFBody *a, const PFBody *b, PFManifold * m) {
	if (pfBodyToBody(a, b, &m->normal, &m->penetration)) {
		m->e = a->restitution * b->restitution;
		m->dynamicFriction = a->dynamicFriction * b->dynamicFriction;
		m->staticFriction = a->staticFriction * b->staticFriction;
		return true;
	}
	return false;
}

void pfIntegrateForce(const float dt, const MLV2f gravity, PFBody *a) {
	if (!mlNearZerof(a->inverseMass)) {
		const MLV2f velocity = mlMulV2ff(
			mlAddV2f(mlMulV2ff(a->force, a->inverseMass), gravity),
			dt / 2
		);
		a->velocity = mlAddV2f(a->velocity, velocity);
	}
}

void pfIntegrateVelocity(const float dt, const MLV2f gravity, PFBody *a) {
	if (!mlNearZerof(a->inverseMass)) {
		const MLV2f position = mlMulV2ff(a->velocity, dt);
		a->position = mlAddV2f(a->position, position);
		pfIntegrateForce(dt, gravity, a);
	}
}

void pfManifoldInitialize(const PFBody *a, const PFBody *b, PFManifold * m) {
	m->e = a->restitution * b->restitution;
	m->staticFriction = a->staticFriction * b->staticFriction;
	m->dynamicFriction = a->dynamicFriction * b->dynamicFriction;
}

void pfPositionalCorrection(const PFManifold * m, PFBody *a, PFBody *b) {
	const float percent = 0.4;
	const float slop = 0.05;
	const MLV2f correction = mlMulV2ff(
		m->normal,
		mlMaxf(0, (m->penetration - slop) / (a->inverseMass + b->inverseMass)) * percent
	);
	a->position = mlSubV2f(a->position, mlMulV2ff(correction, a->inverseMass));
	b->position = mlAddV2f(b->position, mlMulV2ff(correction, b->inverseMass));
}

void pfManifoldApplyImpulse(const PFManifold * m, PFBody *a, PFBody *b) {
	const float inverseMassSum = a->inverseMass + b->inverseMass;
	if (mlNearZerof(inverseMassSum)) {
		a->velocity = mlV2Zerof();
		b->velocity = mlV2Zerof();
		return;
	}
	
	MLV2f rv = mlSubV2f(b->velocity, a->velocity);
	const float contactVelocity = mlDotV2f(rv, m->normal);
	if (contactVelocity > 0)
		return;

	const float e = mlMinf(a->restitution, b->restitution);
	const float j = (-(1 + e) * contactVelocity) / inverseMassSum;
	const MLV2f impulse = mlMulV2ff(m->normal, j);
	a->velocity = mlSubV2f(a->velocity, mlMulV2ff(impulse, a->inverseMass));
	b->velocity = mlAddV2f(b->velocity, mlMulV2ff(impulse, b->inverseMass));
	rv = mlSubV2f(b->velocity, a->velocity);
	const MLV2f t = mlNormalizeV2f(mlSubV2f(rv, mlMulV2ff(m->normal, mlDotV2f(rv, m->normal))));
	const float jt = -mlDotV2f(rv,t) / inverseMassSum;
	if (mlNearZerof(jt)) {
		return;
	}

	const MLV2f tagentImpulse = mlMulV2ff(
		t,
		mlAbsf(jt) < (j * m->staticFriction) ? jt : (-j * m->dynamicFriction)
	);
	a->velocity = mlSubV2f(a->velocity, mlMulV2ff(tagentImpulse, a->inverseMass));
	b->velocity = mlAddV2f(b->velocity, mlMulV2ff(tagentImpulse, b->inverseMass));
}

void pfBodySetMass(const float mass, PFBody *a) {
	a->mass = mass;
	a->inverseMass = mlRecipNoInff(mass);
}

float pfMassFromDensity(const float density, const PFShape shape) {
	switch (shape.tag) {
	case PF_SH_RECT: return density * shape.radii.x * shape.radii.y;
	case PF_SH_CIRCLE: return density * mlPif() * shape.radius * shape.radius;
	default: assert(false);
	}
}

void pfBodyEsque(const float density, const float restitution, PFBody *a) {
	pfBodySetMass(pfMassFromDensity(density, a->shape), a);
	a->restitution = restitution;
}

void pfRockEsque(PFBody *a) {
	pfBodyEsque(0.6, 0.1, a);
}

void pfWoodEsque(PFBody *a) {
	pfBodyEsque(0.3, 0.2, a);
}

void pfMetalEsque(PFBody *a) {
	pfBodyEsque(1.2, 0.05, a);
}

void pfBouncyBallEsque(PFBody *a) {
	pfBodyEsque(0.3, 0.8, a);
}

void pfSuperBallEsque(PFBody *a) {
	pfBodyEsque(0.3, 0.95, a);
}

void pfPillowEsque(PFBody *a) {
	pfBodyEsque(0.1, 0.2, a);
}

void pfStaticEsque(PFBody *a) {
	pfBodyEsque(0, 0.4, a);
}

PFBody pfBody() {
	return (PFBody) {
		.mode = PF_BM_DYNAMIC,
		.shape = (PFShape) { 
				.tag = PF_SH_CIRCLE,
				.radius = 0
			},
		.position = mlV2Zerof(),
		.velocity = mlV2Zerof(),
		.force = mlV2Zerof(),
		.mass = 1,
		.inverseMass = 1,
		.staticFriction = 0.5,
		.dynamicFriction = 0.5,
		.restitution = 0.5,
	};
}

PFShape pfCircle(const float radius) {
	return (PFShape) {
		.tag = PF_SH_CIRCLE,
		.radius = radius,
	};
}

PFShape pfBox(const float side) {
	return (PFShape) {
		.tag = PF_SH_RECT,
		.radii = mlV2Fillf(side),
	};
}

PFShape pfRect(const float w, const float h) {
	return (PFShape) {
		.tag = PF_SH_RECT,
		.radii = mlV2f(w, h),
	};
}
