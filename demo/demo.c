#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <ml.h>
#include <pf.h>
#include <SDL2/SDL.h>
#include <math.h>

struct key_body {
	int key;
	const struct pf_body *body;
};

struct keys_manifold {
	int a_key;
	int b_key;
	struct pf_manifold manifold;
};

#define MAX_BODIES 4096 /* So many bodies */
#define MAX_MANIFOLDS (MAX_BODIES * 2) /* Large enough */
#define CANNON_BALL_RADIUS 1.5

struct world {
	struct pf_body bodies[MAX_BODIES];
	int body_count;
	struct keys_manifold manifolds[MAX_MANIFOLDS];
	int manifoldCount;
	float dt;
	v2f gravity;
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
	w->gravity = mkv2f(0,0);
	w->iterations = 10;

	/* Add bodies */

	// cannon ball
	w->bodies[0] = pf_make_body();
	w->bodies[0].shape = pf_circle(CANNON_BALL_RADIUS);
	w->bodies[0].position = mkv2f(-200,35);
	w->bodies[0].velocity = mkv2f(200,0);
	pf_metal_esque(w->bodies + 0);

	w->body_count++;

	for (int i = 0; i < 50; i++) {
		for (int j = 0; j < 50; j++) {
			struct pf_body *a = w->bodies + (i * 50 + j + 1);
			a->shape = (i + j) % 2 == 0 ? pf_box(1) : pf_circle(1);
			a->position = mkv2f(i * 1 + 6,j * 1 + 5);
			pf_rock_esque(a);
			w->body_count++;
		}
	}
	
	// walls
	struct pf_body *lf, *rt, *up, *dn;
	lf = w->bodies + w->body_count;
//	w->body_count++;
	rt = w->bodies + w->body_count;
//	w->body_count++;
	up = w->bodies + w->body_count;
//	w->body_count++;
	dn = w->bodies + w->body_count;
//	w->body_count++;
	
	lf->position = mkv2f(-40, -20);
	lf->shape = pf_rect(40, 80);
	pf_body_set_mass(0, lf);
	
	rt->position = mkv2f(100, -20);
	rt->shape = pf_rect(40, 80);
	pf_body_set_mass(0, rt);

	up->position = mkv2f(40, -20);
	up->shape = pf_rect(100, 40);
	pf_body_set_mass(0, up);
	
	dn->position = mkv2f(40, 80);
	dn->shape = pf_rect(100, 40);
	pf_body_set_mass(0, dn);
}

void make_world_1(struct world *w) {
	w->body_count = 0;
	w->manifoldCount = 0;
	w->dt = 1.0 / 60.0;
	w->gravity = mkv2f(0.5,9.8);
	w->iterations = 10;
	
	{
		struct pf_body *a = &w->bodies[w->body_count];
		w->body_count++;
		pf_body_set_mass(0, a);
		a->shape = pf_circle(5); 
		a->position = mkv2f(4,-5);
		pf_bouncy_ball_esque(a);
	}

	{
		struct pf_body *a = &w->bodies[w->body_count];
		w->body_count++;
		pf_body_set_mass(0, a);
		a->shape = pf_circle(1); 
		a->position = mkv2f(1,3);
		a->velocity = mkv2f(3,0);
		pf_bouncy_ball_esque(a);
	}
	
	{
		struct pf_body *a = &w->bodies[w->body_count];
		w->body_count++;
		pf_body_set_mass(0, a);
		a->shape = pf_rect(2,6);
		a->position = mkv2f(14,3);
		pf_metal_esque(a);
	}
	
	{
		struct pf_body *a = &w->bodies[w->body_count];
		w->body_count++;
		pf_body_set_mass(0, a);
		a->shape = pf_rect(10,1);
		a->position = mkv2f(10,20);
	}
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
		const struct pf_body *i_body = &w->bodies[i];
		const bool iNearZero = nearzerov2f(i_body->velocity);
		const bool iNoMass = i_body->mass == 0;

		for (int j = i + 1; j < w->body_count; j++) {
			const struct pf_body *j_body = &w->bodies[j];

			if (iNoMass && j_body->mass == 0)
				continue;
			if (iNearZero && nearzerov2f(j_body->velocity))
				continue;

			struct keys_manifold *km = &w->manifolds[w->manifoldCount];
			if (pf_solve_collision(i_body, j_body, &km->manifold)) {
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
		pf_integrate_force(w->dt, w->gravity, w->bodies + i);
}

void initialize_collisions(struct world *w) {
	for (int i = 0; i < w->manifoldCount; i++)
		pf_manifold_initialize(
			w->bodies + w->manifolds[i].a_key,
			w->bodies + w->manifolds[i].b_key,
			&w->manifolds[i].manifold
		);
}

void solve_collisions(struct world *w) {
	for (int it = 0; it < w->iterations; it++)
		for (int i = 0; i < w->manifoldCount; i++) {
			const struct keys_manifold *km = &w->manifolds[i];
			pf_manifold_apply_impulse(&km->manifold, &w->bodies[km->a_key], &w->bodies[km->b_key]);
		}
}

void integrate_velocities(struct world *w) {
	for (int i = 0; i < w->body_count; i++)
		pf_integrate_velocity(w->dt, w->gravity, &w->bodies[i]);
}

void correct_positions(struct world *w) {
	for (int i = 0; i < w->body_count; i++) {
		const struct keys_manifold *km = &w->manifolds[i];
		pf_positional_correction(&km->manifold, &w->bodies[km->a_key], &w->bodies[km->b_key]);
	}
}

void reset_collisions(struct world *w) {
	for (int i = 0; i < w->body_count; i++)
		w->bodies[i].force = mkv2f(0,0);
	w->manifoldCount = 0;
}

void render_demo(struct demo *d) {
	SDL_SetRenderDrawColor(d->renderer, 0x00, 0x00, 0x00, 0xff);
	SDL_RenderClear(d->renderer);

	SDL_SetRenderDrawColor(d->renderer, 0xff, 0xff, 0xff, 0xff);
	for (int i = 0; i < d->world.body_count; i++) {
		const struct pf_body *a = &d->world.bodies[i];
		switch (a->shape.tag) {
		case PF_SH_RECT: {
			SDL_Rect rect = {
				.x = 10 * (a->position.x - a->shape.radii.x),
				.y = 10 * (a->position.y - a->shape.radii.y),
				.w = 10 * (a->shape.radii.x * 2.0f),
				.h = 10 * (a->shape.radii.y * 2.0f),
			};
			SDL_RenderDrawRect(d->renderer, &rect);
			break;
		}
		case PF_SH_CIRCLE: {
			const int len = 21;
			SDL_Point lines[len];
			for (int i = 0; i < len; i++) {
				const float theta = (float)i * 2.0f * M_PI / (float)(len - 1);
				lines[i].x = 10 * ((float)a->position.x + (float)a->shape.radius * cosf(theta));
				lines[i].y = 10 * ((float)a->position.y + (float)a->shape.radius * sinf(theta));
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
