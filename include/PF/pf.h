#ifndef PFH
#define PFH

#include <stdlib.h>
#include <stdbool.h>
#include <ML/ml.h>

typedef struct {
	MLV2f min;
	MLV2f max;
} PFAABB;

typedef enum {
	PF_SH_RECT,
	PF_SH_CIRCLE
} PFShapeTag;

typedef struct {
	PFShapeTag tag;
	union {
		MLV2f radii;
		float radius;
	};
} PFShape;

typedef enum {
	PF_BM_STATIC,
	PF_BM_DYNAMIC
} PFBodyMode;

typedef struct {
	PFBodyMode mode;
	PFShape shape;
	MLV2f position;
	MLV2f velocity;
	MLV2f force;
	float mass;
	float inverseMass;
	float staticFriction;
	float dynamicFriction;
	float restitution;
} PFBody;

typedef struct {
	MLV2f normal;
	float penetration;
	float e;
	float dynamicFriction;
	float staticFriction;
} PFManifold;

bool pfIntersect(const PFAABB a, const PFAABB b);
bool pfInside(MLV2f a, const PFAABB b);
MLV2f pfAABBPosition(const PFAABB a);
PFAABB pfBodyToAABB(const PFBody * a);
bool pfTestBody(const PFAABB a, const PFBody * b);
bool pfBodyToBody(const PFBody * a, const PFBody * b, MLV2f * normal, float * penetration);
bool pfSolveCollision(const PFBody * a, const PFBody * b, PFManifold * m);
void pfIntegrateForce(const float dt, const MLV2f gravity, PFBody * a);
void pfIntegrateVelocity(const float dt, const MLV2f gravity, PFBody * a);
void pfManifoldInitialize(const PFBody * a, const PFBody * b, PFManifold * m);
void pfPositionalCorrection(const PFManifold * m, PFBody * a, PFBody * b);
void pfManifoldApplyImpulse(const PFManifold * m, PFBody * a, PFBody * b);

void pfBodySetMass(const float mass, PFBody *a);
void pfBodyEsque(const float density, const float restitution, PFBody * a);
void pfRockEsque(PFBody * a);
void pfWoodEsque(PFBody * a);
void pfMetalEsque(PFBody * a);
void pfBouncyBallEsque(PFBody * a);
void pfSuperBallEsque(PFBody * a);
void pfPillowEsque(PFBody * a);
void pfStaticEsque(PFBody * a);
PFBody pfBody();
PFShape pfCircle(const float radius);
PFShape pfBox(const float side);
PFShape pfRect(const float w, const float h);

#endif
