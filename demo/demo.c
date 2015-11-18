#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <ml.h>
#include <pf.h>
#include <SDL2/SDL.h>
#include <math.h>

typedef struct key_body {
    int key;
    const pf_body_t *body;
} key_body_t;

typedef struct keys_manifold {
    int a_key;
    int b_key;
    pf_manifold_t manifold;
} keys_manifold_t;

#define MAX_BODIES 4096 /* So many bodies */
#define MAX_MANIFOLDS (MAX_BODIES * 2) /* Large enough */
#define CANNON_BALL_RADIUS 1.5

typedef struct world {
    pf_body_t bodies[MAX_BODIES];
    int body_num;
    keys_manifold_t manifolds[MAX_MANIFOLDS];
    int manifold_num;
    float dt;
    int iterations;
} world_t;

typedef struct input {
    bool quit;
    bool left;
    bool right;
    bool up;
    bool down;
    bool clear_parents;
} input_t;

typedef struct demo {
    SDL_Renderer *renderer;
    world_t world;
    input_t input;
} demo_t;

void make_demo(demo_t *d, SDL_Renderer *renderer);
void loop_demo(demo_t *d);

int main() {
    if (SDL_Init(SDL_INIT_EVERYTHING) > 0) {
        return EXIT_FAILURE;
    }
    SDL_Window *win = SDL_CreateWindow("demo", 0, 0, 320, 240, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_PRESENTVSYNC);
    demo_t demo;
    make_demo(&demo, renderer);
    loop_demo(&demo);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return EXIT_SUCCESS;
}

void make_world(world_t *w) {
    w->body_num = 0;
    w->manifold_num = 0;
    w->dt = 1.0 / 60.0;
    w->iterations = 10;

    // Large circle
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->shape = pf_circle(3);
        //a->gravity = _v2f(9.5, -9.8);
        a->position = _v2f(4,5);
        pf_bouncy_ball_esque(a);
    }

    // Small circle
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->shape = pf_circle(0.5);
        //a->gravity = _v2f(10,3);
        a->position = _v2f(2,3);
        pf_super_ball_esque(a);
    }

    // Rectangle
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        //a->gravity = _v2f(-10, 6);
        a->shape = pf_rect(2,3);
        a->position = _v2f(14,4);
        pf_pillow_esque(a);
    }

    // Platform
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_BM_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(7,1);
        a->position = _v2f(8,16.25);
    }

    // Walls
    // Top
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_BM_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32,0.5);
        a->position = _v2f(0,0);
    }
    // Bottom
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_BM_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32,0.5);
        a->position = _v2f(0,24);
    }
    // Left
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_BM_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(0.5,24);
        a->position = _v2f(0,0);
    }
    // Right
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_BM_STATIC;
        pf_body_set_mass(0, a);
        a->mode = PF_BM_STATIC;
        a->shape = pf_rect(0.5,24);
        a->position = _v2f(32,0);
    }

}

void make_demo(demo_t *d, SDL_Renderer *renderer) {
    d->renderer = renderer;
    make_world(&d->world);
}

void read_input(struct input *input) {
    SDL_Event event;
    input->clear_parents = false;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_KEYDOWN) {
            switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
                input->quit = true;
                break;
            case SDLK_SPACE:
                input->clear_parents = true;
                break;
            default:
                break;
            }
        }
    }
}

unsigned int delay_time(unsigned int goal, unsigned int start,
            unsigned int end) {
    const unsigned int frame = end - start;
    return frame >= goal ? 0 : (goal - frame);
}

void step_world(world_t *w);
void render_demo(demo_t *d);

void loop_demo(demo_t *d) {
    d->input.quit = false;
    do {
        const unsigned int start_tick = SDL_GetTicks();
        read_input(&d->input);
        for (int i = 0; i < d->world.body_num; i++) {
            d->world.bodies[i].parent = NULL;
        }
        step_world(&d->world);
        render_demo(d);
        const unsigned int end_tick = SDL_GetTicks();
        //printf("FPS: %f\n", 1000.0f / (end_tick - start_tick));
        SDL_Delay(delay_time(16, start_tick, end_tick));
        //printf("x:%f y:%f\n", d->world.bodies[0].position.x,
        //       d->world.bodies[0].position.y);
    } while (!d->input.quit);
}

void object_platform_relations(world_t *w);
void move_platforms(world_t *w);
void update_object_positions_on_platforms(world_t *w);
void generate_collisions(world_t *w);
void integrate_forces(world_t *w);
void solve_collisions(world_t *w);
void cap_velocities(world_t *w);
void integrate_velocities(world_t *w);
void correct_positions(world_t *w);
void reset_collisions(world_t *w);

void step_world(world_t *w) {
    // Define objects and platforms relationships
    object_platform_relations(w);
    // Move platforms (no collisions)
    move_platforms(w);
    // Assign objects' positions if on platform
    update_object_positions_on_platforms(w);
    // Step objects as normally
    generate_collisions(w);
    integrate_forces(w);
    solve_collisions(w);
    cap_velocities(w);
    integrate_velocities(w);
    correct_positions(w);
    reset_collisions(w);
}

float normf(float x) {
    return nearzerof(x) ? 0 : (x > 0 ? 1 : -1);
}

void try_child_connect_parent(pf_body_t *a, pf_body_t *b) {
    if (a->mass == 0 &&
        b->mass != 0 &&
        a->shape.tag == PF_SH_RECT &&
        b->enable_gravity &&
        !nearzerov2f(b->gravity)
        ) {
        if (fabsf(b->gravity.x) > fabsf(b->gravity.y)) {
            if (normf(b->gravity.x) == normf(a->position.x - b->position.x) &&
                !b->parent) {
                    b->parent = a;
            }
        } else {
            if (normf(b->gravity.y) == normf(a->position.y - b->position.y) &&
                !b->parent) {
                b->parent = a;
            }
        }
    }
}

void object_platform_relations(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        // Find a valid reason to detach
        // Don't detach and possibly reattach each iteration
        w->bodies[i].parent = NULL;
    }

    for (int i = 0; i < w->body_num; i++) {
        pf_body_t *a = &w->bodies[i];
        for (int j = i + 1; j < w->body_num; j++) {
            pf_body_t *b = &w->bodies[j];
            if ((a->mass == 0 && b->mass == 0) ||
                (nearzerov2f(a->velocity) && nearzerov2f(b->velocity))) {
                continue;
            }
            pf_manifold_t manifold;
            if (pf_solve_collision(a, b, &manifold)) {
                try_child_connect_parent(a, b);
                try_child_connect_parent(b, a);
            }
        }
    }
}



void move_platforms(world_t *w) {
    w->bodies[3].force = _v2f(1,0);
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_BM_STATIC) {
            pf_integrate_force(w->dt, &w->bodies[i]);
            pf_integrate_velocity(w->dt, &w->bodies[i]);
        }
    }
}

void update_object_positions_on_platforms(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_BM_DYNAMIC &&
            w->bodies[i].parent &&
            w->bodies[i].parent->mode == PF_BM_STATIC) {

            w->bodies[i].position = addv2f(w->bodies[i].position, w->bodies[i].parent->velocity);
        }
    }
}

void generate_collisions(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        //don't disable gravity
        //w->bodies[i].enable_gravity = !w->bodies[i].parent;
    }

    for (int i = 0; i < w->body_num; i++) {
        const pf_body_t *i_body = &w->bodies[i];
        const bool i_near_zero = nearzerov2f(i_body->velocity);
        const bool i_no_mass = i_body->mass == 0;

        for (int j = i + 1; j < w->body_num; j++) {
            const pf_body_t *j_body = &w->bodies[j];
            const bool j_near_zero = nearzerov2f(j_body->velocity);
            const bool j_no_mass = j_body->mass == 0;

            if (i_no_mass && j_no_mass) {
                continue;
            }
            if (i_near_zero && j_near_zero) {
                continue;
            }

            struct keys_manifold *km = &w->manifolds[w->manifold_num];
            if (pf_solve_collision(i_body, j_body, &km->manifold)) {
                km->a_key = i;
                km->b_key = j;
                w->manifold_num++;
                if (w->manifold_num == MAX_MANIFOLDS) {
                    return;
                }
            }
        }
    }
}

void integrate_forces(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_BM_DYNAMIC) {
            pf_integrate_force(w->dt, &w->bodies[i]);
        }
    }
}

void solve_collisions(world_t *w) {
    for (int it = 0; it < w->iterations; it++) {
        for (int i = 0; i < w->manifold_num; i++) {
            const keys_manifold_t *km = &w->manifolds[i];
            const pf_manifold_t *m = &km->manifold;
            pf_body_t *a = &w->bodies[km->a_key];
            pf_body_t *b = &w->bodies[km->b_key];

            if (a->parent == b) {
                a->position = subv2f(a->position, mulv2nf(m->normal, m->penetration / w->iterations));
            } else if (b->parent == a) {
                b->position = subv2f(b->position, mulv2nf(m->normal, m->penetration / w->iterations));
            } else {
                pf_manifold_apply_impulse(m, a, b);
            }
        }
    }
}

void cap_velocities(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        w->bodies[i].velocity = clampv2f(_v2f(-300,-300), _v2f(300,300), w->bodies[i].velocity);
    }
}

void integrate_velocities(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        pf_integrate_velocity(w->dt, &w->bodies[i]);
    }
}

void correct_positions(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        const struct keys_manifold *km = &w->manifolds[i];
        pf_positional_correction(&km->manifold, &w->bodies[km->a_key], &w->bodies[km->b_key]);
    }
}

void reset_collisions(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        w->bodies[i].force = _v2f(0,0);
        //w->bodies[i].parent_velocity = _v2f(0,0);
    }
    w->manifold_num = 0;
}

void render_demo(demo_t *d) {
    SDL_SetRenderDrawColor(d->renderer, 0x00, 0x00, 0x00, 0xff);
    SDL_RenderClear(d->renderer);
    SDL_SetRenderDrawColor(d->renderer, 0xff, 0xff, 0xff, 0xff);
    for (int i = 0; i < d->world.body_num; i++) {
        const pf_body_t *a = &d->world.bodies[i];
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
    //putchar('\n');
    SDL_RenderPresent(d->renderer);
}
