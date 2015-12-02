#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <ml.h>
#include <pf.h>
#include <SDL2/SDL.h>
#include <math.h>

#define IGNORE_MP\
                const int MOVING_PLATFORM = 3;\
                if ((km->a_key == MOVING_PLATFORM && km->manifold.normal.y >= 0) ||\
                    (km->b_key == MOVING_PLATFORM && km->manifold.normal.y <= 0)) {\
                    continue;\
                }\
 
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

void world_add_tri(world *w, float rw, float rh, float px, float py, pf_corner hypotenuse, bool line) {
    pf_body *a = &w->bodies[w->body_num];
    w->body_num++;
    *a = _pf_body();
    a->mode = PF_MODE_STATIC;
    pf_body_set_mass(0, a);
    a->shape.tri = _pf_tri(_v2f(rw,rh), line, hypotenuse);
    a->shape.tag = PF_SHAPE_TRI;
    a->pos = _v2f(px, py);
}

void world_add_rect(world *w, float rw, float rh, float px, float py) {
    pf_body *a = &w->bodies[w->body_num];
    w->body_num++;
    *a = _pf_body();
    a->mode = PF_MODE_STATIC;
    pf_body_set_mass(0, a);
    a->shape = pf_rect(rw, rh);
    a->pos = _v2f(px, py);
}

void make_world(world *w) {
    w->body_num = 0;
    w->manifold_num = 0;
    w->dt = 1.0 / 60.0;
    w->iterations = 1;
    w->platform_dir = true; 

    // Large circle
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->gravity.accel = 1;
        a->gravity.cap = 0.5;
        a->shape = pf_circle(1.2);
        a->pos = _v2f(30,5);
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
        a->shape = pf_rect(2,1);
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
        a->shape = pf_rect(24.6, 1.2);
        a->pos = _v2f(32,38.4);
    }

    /*
    // Walls
    // Top
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32, 0.5);
        a->pos = _v2f(32, 0);
    }
    // Bottom
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(32,0.5);
        a->pos = _v2f(32, 24 * 2);
    }
    // Left
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->shape = pf_rect(0.5,24);
        a->pos = _v2f(0, 24);
    }
    // Right
    {
        pf_body *a = &w->bodies[w->body_num];
        w->body_num++;
        *a = _pf_body();
        a->mode = PF_MODE_STATIC;
        pf_body_set_mass(0, a);
        a->mode = PF_MODE_STATIC;
        a->shape = pf_rect(0.5, 24);
        a->pos = _v2f(32 * 2, 24);
    }
    */
    /*
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
    */

    // Bottom Platform
    {
        const float x = 32;
        const float y = 33.6;

        world_add_tri(w, 2.95, 0.8,  x - 11, y - 2.8, PF_CORNER_UL, false);
        world_add_rect(w, 2.95, 2.8, x - 11, y + 0.8);
        world_add_rect(w, 8.05, 3.6, x + 0,  y + 0);
        world_add_rect(w, 2.95, 2.8, x + 11, y + 0.8);
        world_add_tri(w, 2.95, 0.8,  x + 11, y - 2.8, PF_CORNER_UR, false);
    }
    // Top Platform
    {
        const float x = 27;
        const float y = 33.6 - 6.8 - 3.6;
        const float sx = 1.2;
        const float h = 0.001;

        world_add_tri(w, 2.4 * sx, 0.8,  x + sx * (- 8.2 - 2.4), y - 0.8, PF_CORNER_UR, true);
        world_add_rect(w, 8.2 * sx , h, x, y + h);
        world_add_tri(w, 2.8 * sx, 0.25,  x + sx * (8.2 + 2.8), y - 0.25, PF_CORNER_UL, true);
        world_add_tri(w, 2.4 * sx, 1.2,  x + sx * (8.2 + 2.8 * 2 + 2.4), y - 0.25 + 1.2, PF_CORNER_UR, true); // 0.9 is adjusted from 1.2
        world_add_rect(w, 2.0 * sx, h, x + sx * (8.2 + 2.8 * 2 + 2.4 * 2 + 2.0), y + h - 0.25 + sx * 2);
    }
    // tri guard
    {
        const float x = 32;
        const float y = 16;
        world_add_tri(w, 1.9, 2.3, x - 30, y, PF_CORNER_UR, false);
        world_add_tri(w, 1.9, 2.3, x + 30, y, PF_CORNER_UL, false);
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
        const float force = 25;
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
        {
            for (int i = 0; i < 3; i++) {
                pf_body *a = &d->world.bodies[i];
                if (a->pos.y > 80) {
                     a->pos = _v2f(32, -20);
                }
            }
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

                const int MOVING_PLATFORM = 3;
                if ((i == MOVING_PLATFORM && m.normal.y >= 0) ||
                    (j == MOVING_PLATFORM && m.normal.y <= 0)) {
                    continue;
                }

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
        static float delay = 0;
        const int MOVING_PLATFORM = 3;
        pf_body *a = &w->bodies[MOVING_PLATFORM];
        if (dir) {
            if (a->pos.x < 12) {
                dir = false;
                delay = 5;
            }
        } else {
            if (a->pos.x > 32 * 2 - 12) {
                dir = true;
                delay = 5;
            }
        }
        if (delay <= 0) {
            a->in.impulse = _v2f(dir ? -1 : 1, 0);
        }
        delay -= w->dt;
    }

/*
    {
        static float angle = 0;
        const int MOVING_TRIANGLE = 8;
        pf_body *a = &w->bodies[MOVING_TRIANGLE];
        angle += 2 * M_PI * w->dt * 0.3;
        a->in.impulse = _v2f(cos(angle) * 4, sin(angle * 3) * 1);
    }
*/
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

                IGNORE_MP
                
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

            IGNORE_MP
 
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
    
            IGNORE_MP

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


        IGNORE_MP

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

    // (8,6) are 1/8 of the main arena
    const v2f p1_pos = clampv2f(_v2f(0, 0), _v2f(64, 48), d->world.bodies[0].pos);
    const v2f p2_pos = clampv2f(_v2f(0, 0), _v2f(64, 48), d->world.bodies[2].pos);

    static v2f cam;
    static float angle = 0;
    angle += d->world.dt;
    const float SCREEN_W2 = 160;
    const float SCREEN_H2 = 120;
    const float scale = clampf(0.3, 9, SCREEN_H2 / lenv2f(subv2f(p1_pos, p2_pos))); 
    // v2f target = _v2f(32, 24); // center
    static v2f target = {.x = 32, .y = 24};
    v2f final_target = mulv2nf(addv2f(addv2f(p1_pos, p2_pos), _v2f(32, 24)), 1.0 / 3.0);
    const float lerp_weight = 0.99;
    target = addv2f(mulv2nf(target, lerp_weight), mulv2nf(final_target, 1 - lerp_weight));
    cam = _v2f(SCREEN_W2 / scale - target.x, SCREEN_H2 / scale - target.y);

    for (int i = 0; i < d->world.body_num; i++) {
        const pf_body *a = &d->world.bodies[i];
        switch (a->shape.tag) {
        case PF_SHAPE_RECT: {
            SDL_Rect rect = {
                .x = scale * (cam.x + a->pos.x - a->shape.radii.x),
                .y = scale * (cam.y + a->pos.y - a->shape.radii.y),
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
                    * (
                        cam.x +
                        (float)a->pos.x +
                        (float)a->shape.radius * cosf(theta));
                lines[i].y = scale
                    * (
                        cam.y +
                        (float)a->pos.y +
                        (float)a->shape.radius * sinf(theta));
            }
            lines[len - 1] = lines[0];
            SDL_RenderDrawLines(d->renderer, lines, len);
            break;
        }
        case PF_SHAPE_TRI: {
            pf_aabb tri = pf_body_to_aabb(a);
            const bool noLine = !a->shape.tri.line;
            tri.min = mulv2nf(addv2f(tri.min, cam), scale);
            tri.max = mulv2nf(addv2f(tri.max, cam), scale);
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
