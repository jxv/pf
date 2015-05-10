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
	int prev_supported[MAX_BODIES];
	int supported[MAX_BODIES];
	struct pf_body bodies[MAX_BODIES];
	int body_num;
	struct keys_manifold manifolds[MAX_MANIFOLDS];
	int manifold_num;
	float dt;
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
	SDL_Window *win = SDL_CreateWindow("demo", 0, 0, 320, 240, 0);
	SDL_Renderer *renderer = SDL_CreateRenderer(win, -1,
						    SDL_RENDERER_PRESENTVSYNC);
	struct demo demo;
	make_demo(&demo, renderer);
	loop_demo(&demo);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(win);
	SDL_Quit();
	return EXIT_SUCCESS;
}

void make_world(struct world *w) {
	w->body_num = 0;
	w->manifold_num = 0;
	w->dt = 1.0 / 60.0;
	w->iterations = 10;
	memset(w->prev_supported, -1, sizeof(int));
	memset(w->supported, -1, sizeof(int));

	// Large circle
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		a->shape = pf_circle(3);
		//a->gravity = _v2f(9.5, -9.8);
		a->position = _v2f(4,5);
		pf_bouncy_ball_esque(a);
	}

	// Small circle
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		a->shape = pf_circle(0.5);
		//a->gravity = _v2f(10,3);
		a->position = _v2f(2,3);
		pf_super_ball_esque(a);
	}

	// Rectangle
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		//a->gravity = _v2f(-10, 6);
		a->shape = pf_rect(2,3);
		a->position = _v2f(14,4);
		pf_pillow_esque(a);
	}

	// Platform
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		pf_body_set_mass(0, a);
		a->shape = pf_rect(7,1);
		a->position = _v2f(8,16.25);
	}

	// Walls
	// Top
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		pf_body_set_mass(0, a);
		a->shape = pf_rect(32,0.5);
		a->position = _v2f(0,0);
	}
	// Bottom
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		pf_body_set_mass(0, a);
		a->shape = pf_rect(32,0.5);
		a->position = _v2f(0,24);
	}
	// Left
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		pf_body_set_mass(0, a);
		a->shape = pf_rect(0.5,24);
		a->position = _v2f(0,0);
	}
	// Right
	{
		struct pf_body *a = &w->bodies[w->body_num];
		w->body_num++;
		*a = _pf_body();
		pf_body_set_mass(0, a);
		a->shape = pf_rect(0.5,24);
		a->position = _v2f(32,0);
	}

}

void make_demo(struct demo *d, SDL_Renderer *renderer) {
	d->renderer = renderer;
	make_world(&d->world);
}

void read_input(struct input *input) {
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		if (event.type == SDL_KEYDOWN) {
			input->quit = true;
		}
	}
}

unsigned int delay_time(unsigned int goal, unsigned int start,
			unsigned int end) {
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
		SDL_Delay(delay_time(16, start_tick, end_tick));
		printf("x:%f y:%f\n", d->world.bodies[0].position.x,
		       d->world.bodies[0].position.y);
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

float normf(float x) {
	return nearzerof(x) ? 0 : (x > 0 ? 1 : -1);
}

void generate_collisions(struct world *w) {
	for (int i = 0; i < w->body_num; i++) {
		w->prev_supported[i] = w->supported[i];
		w->supported[i] = -1;
	}

	for (int i = 0; i < w->body_num; i++) {
		const struct pf_body *i_body = &w->bodies[i];
		const bool i_near_zero = nearzerov2f(i_body->velocity);
		const bool i_no_mass = i_body->mass == 0;

		for (int j = i + 1; j < w->body_num; j++) {
			const struct pf_body *j_body = &w->bodies[j];
			const bool j_near_zero = nearzerov2f(j_body->velocity);
			const bool j_no_mass = j_body->mass == 0;

			if (i_no_mass && j_body->mass == 0) {
				continue;
			}
			if (i_near_zero && j_near_zero) {
				continue;
			}

			struct keys_manifold *km =
				&w->manifolds[w->manifold_num];
			if (pf_solve_collision(i_body, j_body, &km->manifold)) {
				km->a_key = i;
				km->b_key = j;
				// Try support connection
				if (i_no_mass && !j_no_mass &&
				    i_body->shape.tag == PF_SH_RECT &&
				    !eqf(j_body->gravity.x, j_body->gravity.y)
				    ) {
					if (fabsf(j_body->gravity.x) >
					    fabsf(j_body->gravity.y)) {
						if (normf(j_body->gravity.x) ==
						    normf(i_body->position.x -
							  j_body->position.x)) {
							w->supported[j] = i;
						}
					} else {
						if (normf(j_body->gravity.y) ==
						    normf(i_body->position.y -
							  j_body->position.y)) {
							w->supported[j] = i;
						}
					}
				}
				if (j_no_mass && !i_no_mass &&
				    j_body->shape.tag == PF_SH_RECT &&
				    !eqf(i_body->gravity.x, i_body->gravity.y)
				    ) {
					if (fabsf(i_body->gravity.x) >
					    fabsf(i_body->gravity.y)) {
						if (normf(i_body->gravity.x) ==
						    normf(j_body->position.x -
							  i_body->position.x)) {
							w->supported[i] = j;
						}
					} else {
						if (normf(i_body->gravity.y) ==
						    normf(j_body->position.y -
							  i_body->position.y)) {
							w->supported[i] = j;
						}
					}
				}
				//
				w->manifold_num++;
				if (w->manifold_num == MAX_MANIFOLDS) {
					return;
				}
			}
		}
	}
}

void integrate_forces(struct world *w) {
	w->bodies[3].force = _v2f(1,0);
	for (int i = 0; i < w->body_num; i++) {
		const int j = w->supported[i];
		if (w->supported[i] != -1 &&
		    w->supported[i] == w->prev_supported[i]) {
			printf("%d on %d (%f,%f)\n", i, j, w->bodies[j].force.x,
			       w->bodies[j].force.y);
			w->bodies[i].parent_velocity = w->bodies[j].force;
		}
	}
	for (int i = 0; i < w->body_num; i++) {
		pf_integrate_force(w->dt, w->bodies + i);
	}
}

void initialize_collisions(struct world *w) {
	for (int i = 0; i < w->manifold_num; i++) {
		pf_manifold_initialize(
			w->bodies + w->manifolds[i].a_key,
			w->bodies + w->manifolds[i].b_key,
			&w->manifolds[i].manifold
		);
	}
}

void solve_collisions(struct world *w) {
	for (int it = 0; it < w->iterations; it++) {
		for (int i = 0; i < w->manifold_num; i++) {
			const struct keys_manifold *km = &w->manifolds[i];
			pf_manifold_apply_impulse(&km->manifold,
						  &w->bodies[km->a_key],
						  &w->bodies[km->b_key]);
		}
	}
}

void integrate_velocities(struct world *w) {
	for (int i = 0; i < w->body_num; i++) {
		pf_integrate_velocity(w->dt, &w->bodies[i]);
	}
}

void correct_positions(struct world *w) {
	for (int i = 0; i < w->body_num; i++) {
		const struct keys_manifold *km = &w->manifolds[i];
		pf_positional_correction(&km->manifold, &w->bodies[km->a_key],
					 &w->bodies[km->b_key]);
	}
}

void reset_collisions(struct world *w) {
	for (int i = 0; i < w->body_num; i++) {
		w->bodies[i].force = _v2f(0,0);
		w->bodies[i].parent_velocity = _v2f(0,0);
	}
	w->manifold_num = 0;
}

void render_demo(struct demo *d) {
	SDL_SetRenderDrawColor(d->renderer, 0x00, 0x00, 0x00, 0xff);
	SDL_RenderClear(d->renderer);
	SDL_SetRenderDrawColor(d->renderer, 0xff, 0xff, 0xff, 0xff);
	for (int i = 0; i < d->world.body_num; i++) {
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
				const float percent = (float)i * 2.0f * M_PI;
				const float theta = percent / (float)(len - 1);
				lines[i].x = 10
					* ((float)a->position.x
				        + (float)a->shape.radius * cosf(theta));
				lines[i].y = 10
					* ((float)a->position.y
					+ (float)a->shape.radius * sinf(theta));
			}
			lines[len - 1] = lines[0];
			SDL_RenderDrawLines(d->renderer, lines, len);
			break;
		}
		default: assert(false);
		}
	}
	putchar('\n');
	SDL_RenderPresent(d->renderer);
}
