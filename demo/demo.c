#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <ml.h>
#include <pf.h>
#include <SDL2/SDL.h>
#include <math.h>

typedef struct {
    int key;
    const pf_body *body;
} key_body;

typedef struct {
    int a_key;
    int b_key;
    pf_manifold manifold;
} keys_manifold;

#define MAX_BODIES 4096 /* So many bodies */
#define MAX_MANIFOLDS (MAX_BODIES * 2) /* Large enough */
#define CANNON_BALL_RADIUS 1.5

typedef struct {
    pf_body bodies[MAX_BODIES];
    int body_num;
    keys_manifold manifolds[MAX_MANIFOLDS];
    int manifold_num;
    float dt;
    int iterations;
    bool platform_dir;
} world;

typedef struct {
    bool quit;
    bool left;
    bool right;
    bool up;
    bool down;
    bool change_axis;
} input;

typedef struct {
    SDL_Renderer *renderer;
    world world;
    input input;
} demo;

void make_demo(demo *d, SDL_Renderer *renderer);
void loop_demo(demo *d);

int main() {
    if (SDL_Init(SDL_INIT_EVERYTHING) > 0) {
        return EXIT_FAILURE;
    }
    SDL_Window *win = SDL_CreateWindow("demo", 0, 0, 320, 240, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_PRESENTVSYNC);
    demo demo;
    make_demo(&demo, renderer);
    loop_demo(&demo);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return EXIT_SUCCESS;
}

void make_world(world *w) {
    w->body_num = 0;
    w->manifold_num = 0;
    w->dt = 1.0 / 60.0;
    w->iterations = 1;
    w->platform_dir = false;

    // Large circle
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->gravity.accel = 1;
        a->gravity.cap = 0.5;
        a->shape = pf_circle(1.2);
        a->pos = _v2f(4,5);
        pf_bouncy_ball_esque(a);
    }

    // Small circle
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->gravity.accel = 1;
        a->gravity.cap = 0.5;
        a->shape = pf_circle(1);
        a->pos = _v2f(28,2);
        pf_super_ball_esque(a);
    }

    // Rectangle
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->gravity.accel = 1;
        a->gravity.cap = 0.5;
        a->shape = pf_rect(2,0.5);
        a->pos = _v2f(14,4);
        pf_pillow_esque(a);
    }

    // Platform
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(7,0.2);
        a->pos = _v2f(8,16.25);
    }

    // Walls
    // Top
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32,0.5);
        a->pos = _v2f(0,0);
    }
    // Bottom
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32,0.5);
        a->pos = _v2f(0,24);
    }
    // Left
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(0.5,24);
        a->pos = _v2f(0,0);
    }
    // Right
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->mode = PF_MODE_STATIC;
        a->shape = pf_rect(0.5,24);
        a->pos = _v2f(32,0);
    }
    // Triangles
    // Upper left
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape.tri = _pf_tri(_v2f(3,0.5), true, PF_CORNER_UL);
        a->shape.tag = PF_SHAPE_TRI;
        a->pos = _v2f(9,20);
    }
    // Upper right (on left)
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape.tri = _pf_tri(_v2f(0.5,5), false, PF_CORNER_UR);
        a->shape.tag = PF_SHAPE_TRI;
        a->pos = _v2f(1,8);
    }
    // Upper right
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape.tri = _pf_tri(_v2f(2,2.8), true, PF_CORNER_UR);
        a->shape.tag = PF_SHAPE_TRI;
        a->pos = _v2f(21,20);
    }

    // Down left
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape.tri = _pf_tri(_v2f(4,2), false, PF_CORNER_DL);
        a->shape.tag = PF_SHAPE_TRI;
        a->pos = _v2f(6,5);
    }
    // Down right
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape.tri = _pf_tri(_v2f(3,1), false, PF_CORNER_DR);
        a->shape.tag = PF_SHAPE_TRI;
        a->pos = _v2f(23,5);
    }
    // Down right (on left)
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape.tri = _pf_tri(_v2f(0.5, 5), false, PF_CORNER_DR);
        a->shape.tag = PF_SHAPE_TRI;
        a->pos = _v2f(1,18);
    }

    for (int i = 0; i < w->body_num; i++) {
        w->bodies[i].gravity.dir = PF_DIR_D;
    }

}

void make_demo(demo *d, SDL_Renderer *renderer) {
    d->renderer = renderer;
    make_world(&d->world);
}

void read_input(input *inp) {
    SDL_Event event;
    memset(inp, 0, sizeof(*inp));
    
    const Uint8 *ks = SDL_GetKeyboardState(NULL);
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_KEYDOWN) {
            switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
                inp->quit = true;
                break;
            case SDLK_SPACE:
                inp->change_axis = true;
                break;
            default:
                break;
            }
        }
    }

    inp->left |= !!ks[SDL_SCANCODE_LEFT];
    inp->right |= !!ks[SDL_SCANCODE_RIGHT];
    inp->down |= !!ks[SDL_SCANCODE_DOWN];
    inp->up |= !!ks[SDL_SCANCODE_UP];

}

unsigned int delay_time(unsigned int goal, unsigned int start, unsigned int end) {
    const unsigned int frame = end - start;
    return frame >= goal ? 0 : (goal - frame);
}

void step_world(world *w);
void render_demo(demo *d);

void loop_demo(demo *d) {
    d->input.quit = false;
    do {
        const unsigned int start_tick = SDL_GetTicks();
        read_input(&d->input);
        //
        pf_body *ch = &d->world.bodies[2];
        const float force = 15;
        if (d->input.left) {
            ch->in.impulse = _v2f(-force, 0);
        }
        if (d->input.right) {
            ch->in.impulse = _v2f(force, 0);
        }
        if (d->input.up) {
            ch->in.impulse = _v2f(0, -force);
        }
        if (d->input.down && !ch->group.object.parent) {
            ch->in.impulse = _v2f(0, force);
        }
        pf_transform_move_on_slope(ch, d->world.dt);
        if (d->input.change_axis) {
            d->world.platform_dir = !d->world.platform_dir;
        }
        //
        step_world(&d->world);
        /*
        printf("dpos: (%.2f,%.2f)\tintern: (%.2f,%.2f)\textern: (%.2f,%.2f)\t gravity: %.2f\tparent: %p\n",
            ch->dpos.x, ch->dpos.y,
            ch->in.impulse.x, ch->in.impulse.y,
            ch->ex.impulse.x, ch->ex.impulse.y,
            ch->gravity.vel,
            (void*)ch->group.object.parent
        );
        */
        render_demo(d);
        const unsigned int end_tick = SDL_GetTicks();
        //printf("FPS: %f\n", 1000.0f / (end_tick - start_tick));
        SDL_Delay(delay_time(16, start_tick, end_tick));
        //printf("x:%f y:%f\n", d->world.bodies[0].position.x, d->world.bodies[0].position.y);
    } while (!d->input.quit);
}

void object_platform_relations(world *w);
void move_platforms(world *w);
void update_object_positions_on_platforms(world *w);
void generate_collisions(world *w);
void step_forces(world *w);
void update_dpos(world *w);
void integrate_forces(world *w);
void solve_object_collisions(world *w);
void solve_platform_collisions(world *w);
void apply_dpos(world *w);
void correct_positions(world *w);
void reset_collisions(world *w);

void step_world(world *w) {
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
    //
    object_platform_relations(w);
    reset_collisions(w);
    generate_collisions(w);
    //
    solve_platform_collisions(w);
    correct_positions(w);
    //
    reset_collisions(w);
}

float normf(float x) {
    return nearzerof(x) ? 0 : (x > 0 ? 1 : -1);
}

// TODO: into library
bool try_child_connect_parent(const pf_manifold *m, pf_body *a, pf_body *b) {
    if (a->mass == 0 &&
        b->mass != 0 &&
        //(a->shape.tag == PF_SHAPE_RECT || (a->shape.tag == PF_SHAPE_TRI && b->shape.tag != PF_SHAPE_CIRCLE)) &&
        !nearzerof(b->gravity.vel)
        ) {
        if (b->gravity.dir == PF_DIR_L || b->gravity.dir == PF_DIR_R) {
            if (m->normal.y < -0.23) {
                b->group.object.parent = a;
                return true;
            }
        } else {
            if (m->normal.y > 0.23) {
                b->group.object.parent = a;
                return true;
            }
        }
    }
    return false;
}

void object_platform_relations(world *w) {
    for (int i = 0; i < w->body_num; i++) {
        pf_body *a = &w->bodies[i];
        pf_manifold m;

        // TODO: refactor (some may go into library)

        // Decide to detach from parent
        if (a->group.object.parent) {
            if (!pf_solve_collision(a, a->group.object.parent, &m)) {
                a->group.object.parent = NULL;
            }
        }
        if (a->group.object.parent) {
            continue;
        }
 
        // Find parent to to attach
        for (int j = i + 1; j < w->body_num; j++) {
            pf_body *b = &w->bodies[j];
            if ((a->mass == 0 && b->mass == 0)) {
                continue;
            }
            if (pf_solve_collision(a, b, &m)) {
                v2f penetration = mulv2nf(m.normal, m.penetration);
                // 0.00011 = magic number to stop jitter
                penetration = subv2f(penetration, mulv2nf(m.normal, 0.00011));
                if (try_child_connect_parent(&m, a, b)) {
                    b->pos = addv2f(b->pos, penetration);
                }
                if (try_child_connect_parent(&m, b, a)) {
                    a->pos = subv2f(a->pos, penetration);
                }
            }
        }
    }
}

void move_platforms(world *w) {
    {
        static bool dir = false;
        const int MOVING_PLATFORM = 3;
        pf_body *a = &w->bodies[MOVING_PLATFORM];
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
            a->in.impulse = _v2f(dir ? -5 : 5, 0);
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
            a->in.impulse = _v2f(0, dir ? -5 : 5);

        }
    }

    {
        static float angle = 0;
        const int MOVING_TRIANGLE = 8;
        pf_body *a = &w->bodies[MOVING_TRIANGLE];
        angle += 2 * M_PI * w->dt * 0.3;
        a->in.impulse = _v2f(cos(angle) * 4, sin(angle * 3) * 1);
    }

    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_STATIC) {
            pf_update_dpos(w->dt, &w->bodies[i]);
            pf_apply_dpos(&w->bodies[i]);
            pf_step_forces(w->dt, &w->bodies[i]);
        }
    }
}

void update_object_positions_on_platforms(world *w) {
    for (int i = 0; i < w->body_num; i++) {
        // TODO: into library
        if (w->bodies[i].mode == PF_MODE_DYNAMIC &&
            w->bodies[i].group.object.parent &&
            w->bodies[i].group.object.parent->mode == PF_MODE_STATIC) {
            w->bodies[i].pos = addv2f(w->bodies[i].pos, w->bodies[i].group.object.parent->dpos);
        }
    }
}

void generate_collisions(world *w) {
    for (int i = 0; i < w->body_num; i++) {
        const pf_body *a = &w->bodies[i];
        for (int j = i + 1; j < w->body_num; j++) {
            const pf_body *b = &w->bodies[j];
            if (a->mass == 0 && b->mass == 0) {
                continue;
            }
            keys_manifold *km = &w->manifolds[w->manifold_num];
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

void step_forces(world *w) {
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_DYNAMIC) {
            pf_step_forces(w->dt, &w->bodies[i]);
        }
    }
}

void solve_object_collisions(world *w) {
    for (int it = 0; it < w->iterations; it++) {
        for (int i = 0; i < w->manifold_num; i++) {
            const keys_manifold *km = &w->manifolds[i];
            const pf_manifold *m = &km->manifold;
            pf_body *a = &w->bodies[km->a_key];
            pf_body *b = &w->bodies[km->b_key];

            // TODO: into library
            if (!((a->mode == PF_MODE_STATIC || b->mode == PF_MODE_STATIC) &&
                 (a->group.object.parent != b && b->group.object.parent != a))) {
                pf_body *item = &w->bodies[1]; // small circle
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

void solve_platform_collisions(world *w) {
    for (int it = 0; it < w->iterations; it++) {
        for (int i = 0; i < w->manifold_num; i++) {
            const keys_manifold *km = &w->manifolds[i];
            const pf_manifold *m = &km->manifold;
            pf_body *a = &w->bodies[km->a_key];
            pf_body *b = &w->bodies[km->b_key];

            // TODO: into library
            if ((a->mode == PF_MODE_STATIC || b->mode == PF_MODE_STATIC) &&
                (a->group.object.parent != b && b->group.object.parent != a)) {
                if (b->mode == PF_MODE_STATIC)
                    a->pos = subv2f(a->pos, mulv2nf(m->normal, m->penetration / w->iterations));
                if (a->mode == PF_MODE_STATIC)
                    b->pos = addv2f(b->pos, mulv2nf(m->normal, m->penetration / w->iterations));
            }
        }
    }
}

void update_dpos(world *w) {
    for (int i = 0; i < w->body_num; i++) {
        pf_body *a = &w->bodies[i];
        if (a->mode == PF_MODE_DYNAMIC) {

            pf_update_dpos(w->dt, a);
        }
    }
}

void apply_dpos(world *w) {
    for (int i = 0; i < w->body_num; i++) {
        if (w->bodies[i].mode == PF_MODE_DYNAMIC) {
            pf_apply_dpos(&w->bodies[i]);
        }
    }
}

void correct_positions(world *w) {
    for (int i = 0; i < w->manifold_num; i++) {
        const keys_manifold *km = &w->manifolds[i];
        const pf_manifold *m = &km->manifold;
        pf_body *a = &w->bodies[km->a_key];
        pf_body *b = &w->bodies[km->b_key];
        if (a->mode == PF_MODE_STATIC || b->mode == PF_MODE_STATIC) {
            pf_pos_correction(m, a, b);
        }
    }
}

void reset_collisions(world *w) {
    w->manifold_num = 0;
}

void render_demo(demo *d) {
    SDL_SetRenderDrawColor(d->renderer, 0x00, 0x00, 0x00, 0xff);
    SDL_RenderClear(d->renderer);
    SDL_SetRenderDrawColor(d->renderer, 0xff, 0xff, 0xff, 0xff);

    const float scale = 10; 

    for (int i = 0; i < d->world.body_num; i++) {
        const pf_body *a = &d->world.bodies[i];
        switch (a->shape.tag) {
        case PF_SHAPE_RECT: {
            SDL_Rect rect = {
                .x = scale * (a->pos.x - a->shape.radii.x),
                .y = scale * (a->pos.y - a->shape.radii.y),
                .w = scale * (a->shape.radii.x * 2.0f),
                .h = scale * (a->shape.radii.y * 2.0f),
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
                lines[i].x = scale
                    * ((float)a->pos.x
                        + (float)a->shape.radius * cosf(theta));
                lines[i].y = scale
                    * ((float)a->pos.y
                    + (float)a->shape.radius * sinf(theta));
            }
            lines[len - 1] = lines[0];
            SDL_RenderDrawLines(d->renderer, lines, len);
            break;
        }
        case PF_SHAPE_TRI: {
            pf_aabb tri = pf_body_to_aabb(a);
            const bool noLine = !a->shape.tri.line;
            tri.min = mulv2nf(tri.min, scale);
            tri.max = mulv2nf(tri.max, scale);
            switch (a->shape.tri.hypotenuse) {
            case PF_CORNER_UL:
                SDL_RenderDrawLine(d->renderer, tri.min.x, tri.max.y, tri.max.x, tri.min.y); // dl-ur
                if (noLine) {
                    SDL_RenderDrawLine(d->renderer, tri.min.x, tri.max.y, tri.max.x, tri.max.y); // dl-dr
                    SDL_RenderDrawLine(d->renderer, tri.max.x, tri.min.y, tri.max.x, tri.max.y); // ur-dr
                }
                break;
           case PF_CORNER_UR:
                SDL_RenderDrawLine(d->renderer, tri.min.x, tri.min.y, tri.max.x, tri.max.y); // ul-dr
                if (noLine) {
                    SDL_RenderDrawLine(d->renderer, tri.min.x, tri.max.y, tri.max.x, tri.max.y); // dl-dr
                    SDL_RenderDrawLine(d->renderer, tri.min.x, tri.min.y, tri.min.x, tri.max.y); // ul-dl
                }
                break;
            case PF_CORNER_DL:
                SDL_RenderDrawLine(d->renderer, tri.min.x, tri.min.y, tri.max.x, tri.max.y); // ul-dr
                if (noLine) {
                    SDL_RenderDrawLine(d->renderer, tri.min.x, tri.min.y, tri.max.x, tri.min.y); // ul-ur
                    SDL_RenderDrawLine(d->renderer, tri.max.x, tri.min.y, tri.max.x, tri.max.y); // ur-dr
                }
                break;
            case PF_CORNER_DR:
                SDL_RenderDrawLine(d->renderer, tri.min.x, tri.max.y, tri.max.x, tri.min.y); // dl-ur
                if (noLine) {
                    SDL_RenderDrawLine(d->renderer, tri.min.x, tri.min.y, tri.max.x, tri.min.y); // ul-ur
                    SDL_RenderDrawLine(d->renderer, tri.min.x, tri.min.y, tri.min.x, tri.max.y); // ul-dl
                }
                break;
          default:
                assert(false);
            }
            break;
        }
        default: assert(false);
        }
    }
    SDL_RenderPresent(d->renderer);
}
