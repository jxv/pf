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
    bool platform_dir;
} world_t;

typedef struct input {
    bool quit;
    bool left;
    bool right;
    bool up;
    bool down;
    bool change_axis;
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
    w->iterations = 1;
    w->platform_dir = false;;

    // Large circle
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->gravity.accel = 1;;
        a->gravity.cap = 1;
        a->shape = pf_circle(3);
        a->pos = _v2f(4,5);
        pf_bouncy_ball_esque(a);
    }

    // Small circle
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->gravity.accel = 1;;
        a->gravity.cap = 1;
        a->shape = pf_circle(1);
        a->pos = _v2f(2,3);
        pf_super_ball_esque(a);
    }

    // Rectangle
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->gravity.accel = 1;;
        a->gravity.cap = 1;
        a->shape = pf_rect(2,3);
        a->pos = _v2f(14,4);
        pf_pillow_esque(a);
    }

    // Platform
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(7,1);
        a->pos = _v2f(8,16.25);
    }

    // Walls
    // Top
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32,0.5);
        a->pos = _v2f(0,0);
    }
    // Bottom
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32,0.5);
        a->pos = _v2f(0,24);
    }
    // Left
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(0.5,24);
        a->pos = _v2f(0,0);
    }
    // Right
    {
        pf_body_t *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->mode = PF_MODE_STATIC;
        a->shape = pf_rect(0.5,24);
        a->pos = _v2f(32,0);
    }

}

void make_demo(demo_t *d, SDL_Renderer *renderer) {
    d->renderer = renderer;
    make_world(&d->world);
}

void read_input(struct input *input) {
    SDL_Event event;
    memset(input, 0, sizeof(input_t));
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_KEYDOWN) {
            switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
                input->quit = true;
                break;
            case SDLK_LEFT:
                input->left = true;
                break;
            case SDLK_UP:
                input->up = true;
                break;
            case SDLK_RIGHT:
                input->right = true;
                break;
            case SDLK_DOWN:
                input->down = true;
                break;
            case SDLK_SPACE:
                input->change_axis = true;
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
        //
        pf_body_t *ch = &d->world.bodies[2];
        const float force = 30;
        if (d->input.left) {
            ch->in.impulse = _v2f(-force, 0);
        }
        if (d->input.right) {
            ch->in.impulse = _v2f(force, 0);
        }
        if (d->input.up) {
            ch->in.impulse = _v2f(0, -force);
        }
        if (d->input.down && !ch->parent) {
            ch->in.impulse = _v2f(0, force);
        }
        if (d->input.change_axis) {
            d->world.platform_dir = !d->world.platform_dir;
        }
        //
        step_world(&d->world);
/*
        printf("dpos: (%.2f,%.2f)\tintern: (%.2f,%.2f)\textern: (%.2f,%.2f)\t gravity: (%.2f,%.2f)\tparent: %p\n",
            ch->dpos.x, ch->dpos.y,
            ch->in.impulse.x, ch->in.impulse.y,
            ch->ex.impulse.x, ch->ex.impulse.y,
            ch->gravity_vel.x, ch->gravity_vel.y,
            (void*)ch->parent
        );
*/
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
void step_forces(world_t *w);
void update_dpos(world_t *w);
void integrate_forces(world_t *w);
void solve_object_collisions(world_t *w);
void solve_platform_collisions(world_t *w);
void apply_dpos(world_t *w);
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
    update_dpos(w);
    apply_dpos(w);
    step_forces(w);
    solve_object_collisions(w);

    object_platform_relations(w);
    reset_collisions(w);
    generate_collisions(w);
  
    solve_platform_collisions(w);
    correct_positions(w);

    reset_collisions(w);
}

float normf(float x) {
    return nearzerof(x) ? 0 : (x > 0 ? 1 : -1);
}

bool try_child_connect_parent(pf_body_t *a, pf_body_t *b) {
    if (a->mass == 0 &&
        b->mass != 0 &&
        a->shape.tag == PF_SHAPE_RECT &&
        !nearzerof(b->gravity.vel)
        ) {
        if (b->gravity.dir == PF_DIR_L || b->gravity.dir == PF_DIR_R) {
            if (normf(b->gravity.vel) == normf(a->pos.x - b->pos.x)) {
                b->parent = a;
                return true;
            }
        } else {
            if (normf(b->gravity.vel) == normf(a->pos.y - b->pos.y)) {
                b->parent = a;
                return true;
            }
        }
    }
    return false;
}

void object_platform_relations(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        pf_body_t *a = &w->bodies[i];
        pf_manifold_t m;

        // Decide to detach from parent
        if (a->parent) {
            if (!pf_solve_collision(a, a->parent, &m)) {
                a->parent = NULL;
            }
        }
        if (a->parent) {
            continue;
        }

        // Find parent to to attach
        for (int j = i + 1; j < w->body_num; j++) {
            pf_body_t *b = &w->bodies[j];
            if ((a->mass == 0 && b->mass == 0)) {
                continue;
            }
            if (pf_solve_collision(a, b, &m)) {
                v2f penetration = mulv2nf(m.normal, m.penetration);
                penetration = subv2f(penetration, mulv2nf(m.normal, 0.00011));
                if (try_child_connect_parent(a, b)) {
                    b->pos = addv2f(b->pos, penetration);
                }
                if (try_child_connect_parent(b, a)) {
                    a->pos = subv2f(a->pos, penetration);
                }
            }
        }
    }
}



void move_platforms(world_t *w) {
    {
        static bool dir = false;
        pf_body_t *a = &w->bodies[3];
        if (w->platform_dir) {
            if (dir) {
                if (a->pos.x < 0) {
                    dir = false;
                }
            } else {
                if (a->pos.x > 32) {
                    dir = true;
                }
            }
            a->in.impulse = _v2f(dir ? -10 : 10, 0);
        } else {
            if (dir) {
                if (a->pos.y < 10) {
                    dir = false;
                }
            } else {
                if (a->pos.y > 25) {
                    dir = true;
                }
            }
            a->in.impulse = _v2f(0, dir ? -10 : 10);

        }
    }
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_STATIC) {
            pf_update_dpos(w->dt, &w->bodies[i]);
            pf_apply_dpos(&w->bodies[i]);
            pf_step_forces(w->dt, &w->bodies[i]);
        }
    }
}

void update_object_positions_on_platforms(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_DYNAMIC &&
            w->bodies[i].parent &&
            w->bodies[i].parent->mode == PF_MODE_STATIC) {
            w->bodies[i].pos = addv2f(w->bodies[i].pos, w->bodies[i].parent->dpos);
        }
    }
}

void generate_collisions(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        const pf_body_t *a = &w->bodies[i];
        for (int j = i + 1; j < w->body_num; j++) {
            const pf_body_t *b = &w->bodies[j];
            if (a->mass == 0 && b->mass == 0) {
                continue;
            }
            struct keys_manifold *km = &w->manifolds[w->manifold_num];
            if (pf_solve_collision(a, b, &km->manifold)) {
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

void step_forces(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_DYNAMIC) {
            pf_step_forces(w->dt, &w->bodies[i]);
        }
    }
}

void solve_object_collisions(world_t *w) {
    for (int it = 0; it < w->iterations; it++) {
        for (int i = 0; i < w->manifold_num; i++) {
            const keys_manifold_t *km = &w->manifolds[i];
            const pf_manifold_t *m = &km->manifold;
            pf_body_t *a = &w->bodies[km->a_key];
            pf_body_t *b = &w->bodies[km->b_key];

            if (!((a->mode == PF_MODE_STATIC || b->mode == PF_MODE_STATIC) &&
                 (a->parent != b && b->parent != a))) {
                pf_body_t *item = &w->bodies[1]; // small circle
                // bounce off other dynamic bodies
                if (a != item && b != item && a->mode == PF_MODE_DYNAMIC && b->mode == PF_MODE_DYNAMIC) {
                    a->ex.impulse = subv2f(a->ex.impulse, mulv2nf(m->normal, m->penetration / w->iterations));
                    b->ex.impulse = addv2f(b->ex.impulse, mulv2nf(m->normal, m->penetration / w->iterations));
                }
                if (a != item && b != item && a->mode == PF_MODE_DYNAMIC && b->mode == PF_MODE_DYNAMIC) {
                    // don't let player bodies 'pop-up' if they're on a platform (which might be moving)
                    a->ex.impulse.y = 0;
                    b->ex.impulse.y = 0;
                }
                pf_apply_manifold(m, a, b);
            }
        }
    }
}

void solve_platform_collisions(world_t *w) {
    for (int it = 0; it < w->iterations; it++) {
        for (int i = 0; i < w->manifold_num; i++) {
            const keys_manifold_t *km = &w->manifolds[i];
            const pf_manifold_t *m = &km->manifold;
            pf_body_t *a = &w->bodies[km->a_key];
            pf_body_t *b = &w->bodies[km->b_key];

            if ((a->mode == PF_MODE_STATIC || b->mode == PF_MODE_STATIC) &&
                (a->parent != b && b->parent != a)) {
                if (b->mode == PF_MODE_STATIC)
                    a->pos = subv2f(a->pos, mulv2nf(m->normal, m->penetration / w->iterations));
                if (a->mode == PF_MODE_STATIC)
                    b->pos = addv2f(b->pos, mulv2nf(m->normal, m->penetration / w->iterations));
            }
        }
    }
}

void update_dpos(world_t *w) {
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_DYNAMIC) {
            pf_update_dpos(w->dt, &w->bodies[i]);
        }
    }
}

void apply_dpos(world_t *w) {
    //printf("%d: %f, %f\n", 2, w->bodies[2].pos.x, w->bodies[2].pos.y);
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_DYNAMIC) {
            pf_apply_dpos(&w->bodies[i]);
        }
    }
}

void correct_positions(world_t *w) {
    for (int i = 0; i < w->manifold_num; i++) {
        const struct keys_manifold *km = &w->manifolds[i];
        const pf_manifold_t *m = &km->manifold;
        pf_body_t *a = &w->bodies[km->a_key];
        pf_body_t *b = &w->bodies[km->b_key];
        if (a->mode == PF_MODE_STATIC || b->mode == PF_MODE_STATIC) {
            pf_pos_correction(m, a, b);
        }
    }
}

void reset_collisions(world_t *w) {
    w->manifold_num = 0;
}

void render_demo(demo_t *d) {
    SDL_SetRenderDrawColor(d->renderer, 0x00, 0x00, 0x00, 0xff);
    SDL_RenderClear(d->renderer);
    SDL_SetRenderDrawColor(d->renderer, 0xff, 0xff, 0xff, 0xff);
    for (int i = 0; i < d->world.body_num; i++) {
        const pf_body_t *a = &d->world.bodies[i];
        switch (a->shape.tag) {
        case PF_SHAPE_RECT: {
            SDL_Rect rect = {
                .x = 10 * (a->pos.x - a->shape.radii.x),
                .y = 10 * (a->pos.y - a->shape.radii.y),
                .w = 10 * (a->shape.radii.x * 2.0f),
                .h = 10 * (a->shape.radii.y * 2.0f),
            };
            SDL_RenderDrawRect(d->renderer, &rect);
            break;
        }
        case PF_SHAPE_CIRCLE: {
            const int len = 21;
            SDL_Point lines[len];
            for (int i = 0; i < len; i++) {
                const float percent = (float)i * 2.0f * M_PI;
                const float theta = percent / (float)(len - 1);
                lines[i].x = 10
                    * ((float)a->pos.x
                        + (float)a->shape.radius * cosf(theta));
                lines[i].y = 10
                    * ((float)a->pos.y
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
