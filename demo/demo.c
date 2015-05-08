#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <ML/ml.h>
#include <PF/pf.h>
#include <SDL2/SDL.h>
#include <math.h>

struct key_body {
	int key;
	const PFBody *body;
};

struct keys_manifold {
	int a_key;
	int b_key;
	PFManifold manifold;
};

#define MAX_BODIES 4096 /* So many bodies */
#define MAX_MANIFOLDS (MAX_BODIES * 2) /* Large enough */
#define CANNON_BALL_RADIUS 1.5

struct world {
	PFBody bodies[MAX_BODIES];
	int body_count;
	struct keys_manifold manifolds[MAX_MANIFOLDS];
	int manifoldCount;
	float dt;
	MLV2f gravity;
	int iterations;
};

struct input {
	bool quit;
	bool left;
	bool right;
	bool up;
	bool down;
};

struct demo {
	SDL_Renderer *renderer;
	struct world world;
	struct input input;
};

void make_demo(struct demo *d, SDL_Renderer *renderer);
void loop_demo(struct demo *d);

int main() {
	if (SDL_Init(SDL_INIT_EVERYTHING) > 0) {
		return EXIT_FAILURE;
	}
	SDL_Window *win = SDL_CreateWindow("PF Demo", 0, 0, 320, 240, 0);
	SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_PRESENTVSYNC);
	struct demo demo;
	make_demo(&demo, renderer);
	loop_demo(&demo);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(win);
	SDL_Quit();
	return EXIT_SUCCESS;
}

void make_world_0(struct world *w) {
	w->body_count = 0;
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

	w->body_count++;

	for (int i = 0; i < 50; i++) {
		for (int j = 0; j < 50; j++) {
			PFBody *a = w->bodies + (i * 50 + j + 1);
			a->shape = (i + j) % 2 == 0 ? pfBox(1) : pfCircle(1);
			a->position = mlV2f(i * 1 + 6,j * 1 + 5);
			pfRockEsque(a);
			w->body_count++;
		}
	}
	
	// walls
	PFBody *lf, *rt, *up, *dn;
	lf = w->bodies + w->body_count;
//	w->body_count++;
	rt = w->bodies + w->body_count;
//	w->body_count++;
	up = w->bodies + w->body_count;
//	w->body_count++;
	dn = w->bodies + w->body_count;
//	w->body_count++;
	
	lf->position = mlV2f(-40, -20);
	lf->shape = pfRect(40, 80);
	pfBodySetMass(0, lf);
	
	rt->position = mlV2f(100, -20);
	rt->shape = pfRect(40, 80);
	pfBodySetMass(0, rt);

	up->position = mlV2f(40, -20);
	up->shape = pfRect(100, 40);
	pfBodySetMass(0, up);
	
	dn->position = mlV2f(40, 80);
	dn->shape = pfRect(100, 40);
	pfBodySetMass(0, dn);
}

void make_world_1(struct world *w) {
	w->body_count = 0;
	w->manifoldCount = 0;
	w->dt = 1.0 / 60.0;
	w->gravity = mlV2f(0.5,9.8);
	w->iterations = 10;
	
	{
		PFBody *a = &w->bodies[w->body_count];
		w->body_count++;
		pfBodySetMass(0, a);
		a->shape = pfCircle(50); 
		a->position = mlV2f(40,-50);
		pfBouncyBallEsque(a);
	}

	{
		PFBody *a = &w->bodies[w->body_count];
		w->body_count++;
		pfBodySetMass(0, a);
		a->shape = pfCircle(10); 
		a->position = mlV2f(10,30);
		a->velocity = mlV2f(30,0);
		pfBouncyBallEsque(a);
	}
	
	{
		PFBody *a = &w->bodies[w->body_count];
		w->body_count++;
		pfBodySetMass(0, a);
		a->shape = pfRect(20,60);
		a->position = mlV2f(140,30);
		pfMetalEsque(a);
	}
	
	{
		PFBody *a = &w->bodies[w->body_count];
		w->body_count++;
		pfBodySetMass(0, a);
		a->shape = pfRect(100,10);
		a->position = mlV2f(100,200);
	}
}


SDL_Texture *createCircleTex(const float radius, SDL_Renderer *renderer) {
	const MLV2f size = mlV2Fillf(10 * radius);
	SDL_Texture *tex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, size.x, size.y);
	SDL_SetRenderTarget(renderer, tex);
	SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0x00);
	SDL_RenderClear(renderer);
	SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);

	SDL_Point points[16];
	const float rad = 5 * radius - 0.5;
	const MLV2f center = mlV2Fillf(rad);
	for (int i = 0; i < 16; i++) {
		const float theta = (float)i * mlTauf() / 15.0f;
		const MLV2f point = mlAddV2f(center, mlMulV2ff(mlV2f(mlCosf(theta), mlSinf(theta)), rad));
		points[i] = (SDL_Point) { .x = mlRoundf(point.x), .y = mlRoundf(point.y) };
	}
	SDL_RenderDrawLines(renderer, points, 16);

	SDL_SetRenderTarget(renderer, NULL);
	return tex;
}

SDL_Texture *createBoxTex(SDL_Renderer *renderer) {
	SDL_Texture *tex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 10, 10);
	SDL_SetRenderTarget(renderer, tex);
	SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0x00);
	SDL_RenderClear(renderer);
	SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
	const SDL_Rect rect = { .x = 0, .y = 0, .w = 10, .h = 10 };
	SDL_RenderDrawRect(renderer, &rect);
	SDL_SetRenderTarget(renderer, NULL);
	return tex;
}

void make_demo(struct demo *d, SDL_Renderer *renderer) {
	d->renderer = renderer;
	make_world_1(&d->world);
}

void read_input(struct input *input) {
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

void step_world(struct world *w);
void render_demo(struct demo *d);

void loop_demo(struct demo *d) {
	d->input.quit = false;
	do {
		const unsigned int start_tick = SDL_GetTicks();
		read_input(&d->input);
		step_world(&d->world);
		render_demo(d);
		const unsigned int end_tick = SDL_GetTicks();
		printf("FPS: %f\n", 1000.0f / (end_tick - start_tick));
		SDL_Delay(delayTime(16, start_tick, end_tick));
		printf("x:%f y:%f\n", d->world.bodies[0].position.x, d->world.bodies[0].position.y);
	} while (!d->input.quit);
}

void generate_collisions(struct world *w);
void integrate_forces(struct world *w);
void initialize_collisions(struct world *w);
void solve_collisions(struct world *w);
void integrate_velocities(struct world *w);
void correct_positions(struct world *w);
void reset_collisions(struct world *w);

void step_world(struct world *w) {
	generate_collisions(w);
	integrate_forces(w);
	initialize_collisions(w);
	solve_collisions(w);
	integrate_velocities(w);
	correct_positions(w);
	reset_collisions(w);
}

void generate_collisions(struct world *w) {
	for (int i = 0; i < w->body_count; i++) {
		const PFBody *i_body = &w->bodies[i];
		const bool iNearZero = mlNearZeroV2f(i_body->velocity);
		const bool iNoMass = i_body->mass == 0;

		for (int j = i + 1; j < w->body_count; j++) {
			const PFBody *j_body = &w->bodies[j];

			if (iNoMass && j_body->mass == 0)
				continue;
			if (iNearZero && mlNearZeroV2f(j_body->velocity))
				continue;

			struct keys_manifold *km = &w->manifolds[w->manifoldCount];
			if (pfSolveCollision(i_body, j_body, &km->manifold)) {
				km->a_key = i;
				km->b_key = j;
				w->manifoldCount++;
				if (w->manifoldCount == MAX_MANIFOLDS)
					return;
			}
		}
	}
}

void integrate_forces(struct world *w) {
	for (int i = 0; i < w->body_count; i++)
		pfIntegrateForce(w->dt, w->gravity, w->bodies + i);
}

void initialize_collisions(struct world *w) {
	for (int i = 0; i < w->manifoldCount; i++)
		pfManifoldInitialize(
			w->bodies + w->manifolds[i].a_key,
			w->bodies + w->manifolds[i].b_key,
			&w->manifolds[i].manifold
		);
}

void solve_collisions(struct world *w) {
	for (int it = 0; it < w->iterations; it++)
		for (int i = 0; i < w->manifoldCount; i++) {
			const struct keys_manifold *km = &w->manifolds[i];
			pfManifoldApplyImpulse(&km->manifold, &w->bodies[km->a_key], &w->bodies[km->b_key]);
		}
}

void integrate_velocities(struct world *w) {
	for (int i = 0; i < w->body_count; i++)
		pfIntegrateVelocity(w->dt, w->gravity, &w->bodies[i]);
}

void correct_positions(struct world *w) {
	for (int i = 0; i < w->body_count; i++) {
		const struct keys_manifold *km = &w->manifolds[i];
		pfPositionalCorrection(&km->manifold, &w->bodies[km->a_key], &w->bodies[km->b_key]);
	}
}

void reset_collisions(struct world *w) {
	for (int i = 0; i < w->body_count; i++)
		w->bodies[i].force = mlV2Zerof();
	w->manifoldCount = 0;
}

void render_demo(struct demo *d) {
	SDL_SetRenderDrawColor(d->renderer, 0x00, 0x00, 0x00, 0xff);
	SDL_RenderClear(d->renderer);

	SDL_SetRenderDrawColor(d->renderer, 0xff, 0xff, 0xff, 0xff);
	for (int i = 0; i < d->world.body_count; i++) {
		const PFBody *a = &d->world.bodies[i];
		switch (a->shape.tag) {
		case PF_SH_RECT: {
			const SDL_Rect rect = {
				.x = a->position.x - a->shape.radii.x, .y = a->position.y - a->shape.radii.y,
				.w = a->shape.radii.x * 2, .h = a->shape.radii.y * 2 
			};
			SDL_RenderDrawRect(d->renderer, &rect);
			break;
		}
		case PF_SH_CIRCLE: {
			const int len = 17;
			SDL_Point lines[len];
			for (int i = 0; i < len; i++) {
				const float theta = (float)i * 2.0f * M_PI / (float)(len - 1);
				lines[i].x = (float)a->position.x + (float)a->shape.radius * cosf(theta);
				lines[i].y = (float)a->position.y + (float)a->shape.radius * sinf(theta);
			}
			lines[len - 1] = lines[0];
			SDL_RenderDrawLines(d->renderer, lines, len);
			break;
		}
		default: assert(false);
		}
	}
	SDL_RenderPresent(d->renderer);
}
