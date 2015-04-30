#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <ML/ml.h>
#include <PF/pf.h>
#include <SDL2/SDL.h>

typedef struct {
	int key;
	const PFBody *body;
} KeyBody;

typedef struct {
	int aKey;
	int bKey;
	PFManifold manifold;
} KeysManifold;

#define MAX_BODIES 4096 /* So many bodies */
#define MAX_MANIFOLDS (MAX_BODIES * 2) /* Large enough */
#define CANNON_BALL_RADIUS 1.5

typedef struct {
	PFBody bodies[MAX_BODIES];
	int bodyCount;
	KeysManifold manifolds[MAX_MANIFOLDS];
	int manifoldCount;
	float dt;
	MLV2f gravity;
	int iterations;
} World;

typedef struct {
	bool quit;
	bool left;
	bool right;
	bool up;
	bool down;
} Input;

typedef struct {
	SDL_Renderer *ren;
	World world;
	Input input;
	SDL_Texture *circle;
	SDL_Texture *box;
	SDL_Texture *cannonBall;
} Demo;

void mkDemo(Demo *d, SDL_Renderer *ren);
void brDemo(Demo *d);
void loopDemo(Demo *d);

int main() {
	if (SDL_Init(SDL_INIT_EVERYTHING) > 0) {
		return EXIT_FAILURE;
	}
	SDL_Window *win = SDL_CreateWindow("PF Demo", 0, 0, 800, 600, 0);
	SDL_Renderer *ren = SDL_CreateRenderer(win, -1, 0);
	Demo demo;
	mkDemo(&demo, ren);
	loopDemo(&demo);
	brDemo(&demo);
	SDL_DestroyRenderer(ren);
	SDL_DestroyWindow(win);
	SDL_Quit();
	return EXIT_SUCCESS;
}

void mkWorld(World *w) {
	w->bodyCount = 0;
	w->manifoldCount = 0;
	w->dt = 1.0 / 60.0;
	w->gravity = mlV2f(0,0);
	w->iterations = 10;

	/* Add bodies */

	// cannon ball
	w->bodies[0] = pfBody();
	w->bodies[0].shape = pfCircle(CANNON_BALL_RADIUS);
	w->bodies[0].position = mlV2f(-200,35);
	w->bodies[0].velocity = mlV2f(200,0);
	pfMetalEsque(w->bodies + 0);

	w->bodyCount++;

	for (int i = 0; i < 50; i++) {
		for (int j = 0; j < 50; j++) {
			PFBody *a = w->bodies + (i * 50 + j + 1);
			a->shape = (i + j) % 2 == 0 ? pfBox(1) : pfCircle(1);
			a->position = mlV2f(i * 1 + 6,j * 1 + 5);
			pfRockEsque(a);
			w->bodyCount++;
		}
	}
}

SDL_Texture *createCircleTex(const float radius, SDL_Renderer *ren) {
	const MLV2f size = mlV2Fillf(10 *radius);
	SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, size.x, size.y);
	SDL_SetRenderTarget(ren, tex);
	SDL_SetRenderDrawColor(ren, 0x00, 0x00, 0x00, 0x00);
	SDL_RenderClear(ren);
	SDL_SetRenderDrawColor(ren, 0xff, 0xff, 0xff, 0xff);

	SDL_Point points[16];
	const float rad = 5 * radius - 0.5;
	const MLV2f center = mlV2Fillf(rad);
	for (int i = 0; i < 16; i++) {
		const float theta = (float)i * mlTauf() / 15.0f;
		const MLV2f point = mlAddV2f(center, mlMulV2ff(mlV2f(mlCosf(theta), mlSinf(theta)), rad));
		points[i] = (SDL_Point) { .x = mlRoundf(point.x), .y = mlRoundf(point.y) };
	}
	SDL_RenderDrawLines(ren, points, 16);

	SDL_SetRenderTarget(ren, NULL);
	return tex;
}

SDL_Texture *createBoxTex(SDL_Renderer *ren) {
	SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 10, 10);
	SDL_SetRenderTarget(ren, tex);
	SDL_SetRenderDrawColor(ren, 0x00, 0x00, 0x00, 0x00);
	SDL_RenderClear(ren);
	SDL_SetRenderDrawColor(ren, 0xff, 0xff, 0xff, 0xff);
	const SDL_Rect rect = { .x = 0, .y = 0, .w = 10, .h = 10 };
	SDL_RenderDrawRect(ren, &rect);
	SDL_SetRenderTarget(ren, NULL);
	return tex;
}

void mkDemo(Demo *d, SDL_Renderer *ren) {
	d->ren = ren;
	mkWorld(&d->world);
	d->cannonBall = createCircleTex(CANNON_BALL_RADIUS, d->ren);
	d->circle = createCircleTex(1, d->ren);
	d->box = createBoxTex(d->ren);
}

void brDemo(Demo *d) {
	SDL_DestroyTexture(d->circle);
	SDL_DestroyTexture(d->box);
	SDL_DestroyTexture(d->cannonBall);
}

void readInput(Input *input) {
	SDL_Event event;
	while (SDL_PollEvent(&event)) {	
		if (event.type == SDL_KEYDOWN) {
			input->quit = true;
		}
	}
}

unsigned int delayTime(const unsigned int goal, 
		const unsigned int start,
		const unsigned int end) {
	const unsigned int frame = end - start;
	return frame >= goal ? 0 : (goal - frame);
}

void stepWorld(World *w);
void renderDemo(Demo *d);

void loopDemo(Demo *d) {
	d->input.quit = false;
	do {
		const unsigned int startTick = SDL_GetTicks();
		readInput(&d->input);
		stepWorld(&d->world);
		renderDemo(d);
		const unsigned int endTick = SDL_GetTicks();
		printf("FPS: %f\n", 1000.0f / (endTick - startTick));
		SDL_Delay(delayTime(16, startTick, endTick));
		printf("x:%f y:%f\n", d->world.bodies[0].position.x, d->world.bodies[0].position.y);
	} while (!d->input.quit);
}

static void generateCollisions(World *w);
static void integrateForces(World *w);
static void initializeCollisions(World *w);
static void solveCollisions(World *w);
static void integrateVelocities(World *w);
static void correctPositions(World *w);
static void resetCollisions(World *w);

void stepWorld(World *w) {
	generateCollisions(w);
	integrateForces(w);
	initializeCollisions(w);
	solveCollisions(w);
	integrateVelocities(w);
	correctPositions(w);
	resetCollisions(w);
}

static void generateCollisions(World *w) {
	for (int i = 0; i < w->bodyCount; i++) {
		const PFBody *iBody = w->bodies + i;
		const bool iNearZero = mlNearZeroV2f(iBody->velocity);
		const bool iNoMass = iBody->mass == 0;

		for (int j = i + 1; j < w->bodyCount; j++) {
			const PFBody *jBody = w->bodies + j;

			if (iNoMass && jBody->mass == 0)
				continue;
			if (iNearZero && mlNearZeroV2f(jBody->velocity))
				continue;

			KeysManifold *km = w->manifolds + w->manifoldCount;
			if (pfSolveCollision(iBody, jBody, &km->manifold)) {
				km->aKey = i;
				km->bKey = j;
				w->manifoldCount++;
				if (w->manifoldCount == MAX_MANIFOLDS)
					return;
			}
		}
	}
}

static void integrateForces(World *w) {
	for (int i = 0; i < w->bodyCount; i++)
		pfIntegrateForce(w->dt, w->gravity, w->bodies + i);
}

static void initializeCollisions(World *w) {
	for (int i = 0; i < w->manifoldCount; i++)
		pfManifoldInitialize(
			w->bodies + w->manifolds[i].aKey,
			w->bodies + w->manifolds[i].bKey,
			&w->manifolds[i].manifold
		);
}

static void solveCollisions(World * w) {
	for (int it = 0; it < w->iterations; it++)
		for (int i = 0; i < w->manifoldCount; i++) {
			const KeysManifold *km = w->manifolds + i;
			pfManifoldApplyImpulse(&km->manifold, w->bodies + km->aKey, w->bodies + km->bKey);
		}
}

static void integrateVelocities(World *w) {
	for (int i = 0; i < w->bodyCount; i++)
		pfIntegrateVelocity(w->dt, w->gravity, w->bodies + i);
}

static void correctPositions(World *w) {
	for (int i = 0; i < w->bodyCount; i++) {
		const KeysManifold *km = w->manifolds + i;
		pfPositionalCorrection(&km->manifold, w->bodies + km->aKey, w->bodies + km->bKey);
	}
}

static void resetCollisions(World *w) {
	for (int i = 0; i < w->bodyCount; i++)
		w->bodies[i].force = mlV2Zerof();
	w->manifoldCount = 0;
}

void renderDemo(Demo *d) {
	SDL_SetRenderDrawColor(d->ren, 0x00, 0x00, 0x00, 0xff);
	SDL_RenderClear(d->ren);

	for (int i = 0; i < d->world.bodyCount; i++) {
		const PFBody *a = d->world.bodies + i;
		switch (a->shape.tag) {
		case PF_SH_RECT: {
			const SDL_Rect rect = {
				.x = a->position.x * 10, .y = a->position.y * 10,
				.w = 10, .h = 10
			};
			SDL_RenderCopy(d->ren, d->box, NULL, &rect);
			break;
		}
		case PF_SH_CIRCLE:
			if (i == 0) {
				const SDL_Rect rect = {
					.x = a->position.x * 10, .y = a->position.y * 10,
					.w = 10.0 * CANNON_BALL_RADIUS, .h = 10.0 * CANNON_BALL_RADIUS
				};
				SDL_RenderCopy(d->ren, d->cannonBall, NULL, &rect);
			} else {
				const SDL_Rect rect = {
					.x = a->position.x * 10, .y = a->position.y * 10,
					.w = 10, .h = 10
				};
				SDL_RenderCopy(d->ren, d->circle, NULL, &rect);
			}
			break;
		default:
			assert(false);
		}
	}
	SDL_RenderPresent(d->ren);
}
