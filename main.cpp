#include <SDL2/SDL.h>
#include <cmath>
#include <vector>

SDL_Window *window = NULL;
SDL_Renderer *renderer = NULL;

#define WINDOW_W 800
#define WINDOW_H 600

struct vector3d
{
    double x;
    double y;
    double z;

    vector3d()
    {
        x = 0.0; y = 0.0; z = 0.0;
    }

    vector3d operator +(vector3d const& vec)
    {
        vector3d out;
        out.x = this->x + vec.x;
        out.y = this->y + vec.y;
        out.z = this->z + vec.z;
        return out;
    }

    vector3d operator -(vector3d const& vec)
    {
        vector3d out;
        out.x = this->x - vec.x;
        out.y = this->y - vec.y;
        out.z = this->z - vec.z;
        return out;
    }

    // scaling
    vector3d operator *(const double scalar)
    {
        vector3d out;
        out.x = this->x * scalar;
        out.y = this->y * scalar;
        out.z = this->z * scalar;
        return out;
    }

    vector3d operator /(const double scalar)
    {
        vector3d out;
        out.x = this->x / scalar;
        out.y = this->y / scalar;
        out.z = this->z / scalar;
        return out;
    }

    double operator *(vector3d const& vec)
    {
        double out = this->x * vec.x + this->y * vec.y + this->z * vec.z;
        return out;
    }

    double mag()
    {
        double out = sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
        return out;
    }

    // normal mathematical: counter-clockwise is positive
    vector3d rotate(double theta)
    {
        vector3d out;
        out.x = this->x * cos(theta) - this->y * sin(theta);
        out.y = this->x * sin(theta) + this->y * cos(theta);
        out.z = this->z;
        return out;
    }
};

struct entity
{
    vector3d pos;
    vector3d vel;
    vector3d acc;

    void act(const double dt)
    {
        pos = pos + vel * dt;
        vel = vel + acc * dt;
    }
};

struct viewport
{
    entity e;
    int w;
    int h;

    viewport(int w, int h)
    {
        this->w = w;
        this->h = h;
    }
};

enum control
{
    CTRL_NONE,
    CTRL_FORWARD,
    CTRL_BACKWARD,
    CTRL_TURN_L,
    CTRL_TURN_R,
    CTRL_FIRE
};

// converts global coordinates to SDL surface coordinates w/ viewport entity
// no viewport rotation supported yet
void cart2sdl(const viewport &vp, vector3d pos, int *x, int *y)
{
    // relative position of entity to viewport center
    vector3d corrected = pos - vp.e.pos;
    *x= (int)(corrected.x + ((double) vp.w / 2));
    *y= (int)(((double)vp.h / 2) - corrected.y);
}

void render_clear(SDL_Renderer *renderer)
{
    SDL_RenderClear(renderer);
}

void render_background(SDL_Renderer *renderer, const viewport &vp)
{
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_Rect r = {0, 0, vp.w, vp.h};
    SDL_RenderFillRect(renderer, &r);
}

void render_player(SDL_Renderer *renderer, const viewport &vp, vector3d pos)
{
    // triangle points
    vector3d t; t.x =  5.0; t.y =  0.0;
    vector3d b; b.x = -5.0; b.y =  0.0;
    vector3d l; l.x = -5.0; l.y =  5.0;
    vector3d r; r.x = -5.0; r.y = -5.0;
    t = pos + t.rotate(pos.z);
    b = pos + b.rotate(pos.z);
    l = pos + l.rotate(pos.z);
    r = pos + r.rotate(pos.z);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_Point points[5];
    cart2sdl(vp, t, &points[0].x, &points[0].y);
    cart2sdl(vp, l, &points[1].x, &points[1].y);
    cart2sdl(vp, r, &points[2].x, &points[2].y);
    points[3] = points[0];
    cart2sdl(vp, b, &points[4].x, &points[4].y);
    SDL_RenderDrawLines(renderer, points, 5);
}

struct renderable
{
    const entity *e;

    renderable(const entity *e)
    {
        this->e = e;
    }
};

void render_flip(SDL_Renderer *renderer)
{
    SDL_RenderPresent(renderer);
}

void render(SDL_Renderer *renderer, const viewport &vp, std::vector<renderable> render_list)
{
    render_clear(renderer);
    render_background(renderer, vp);
    for (renderable r:render_list) {
        render_player(renderer, vp, r.e->pos);
    }
    render_flip(renderer);
}

control key2ctrl(void)
{
    const uint8_t *keyStates = SDL_GetKeyboardState(NULL);

    if (keyStates[SDL_SCANCODE_W])          return CTRL_FORWARD;
    else if (keyStates[SDL_SCANCODE_S])     return CTRL_BACKWARD;
    else if (keyStates[SDL_SCANCODE_A])     return CTRL_TURN_L;
    else if (keyStates[SDL_SCANCODE_D])     return CTRL_TURN_R;
    else if (keyStates[SDL_SCANCODE_SPACE]) return CTRL_FIRE;
    else                                    return CTRL_NONE;
}

double get_time(void)
{
    double t = ((double)SDL_GetTicks()) / 1000.0;
    return t;
}

#define F_ACC_MAG 100.0
#define B_ACC_MAG 100.0
#define T_ACC_MAG 10.0
void handle_controls(entity *e, control c)
{
    e->acc = vector3d();
    switch (c)
    {
    case CTRL_FORWARD:
        e->acc.x = F_ACC_MAG * cos(e->pos.z);
        e->acc.y = F_ACC_MAG * sin(e->pos.z);
        break;
    case CTRL_BACKWARD:
        e->acc.x = -B_ACC_MAG * cos(e->pos.z);
        e->acc.y = -B_ACC_MAG * sin(e->pos.z);
        break;
    case CTRL_TURN_L:
        e->acc.z = +T_ACC_MAG;
        break;
    case CTRL_TURN_R:
        e->acc.z = -T_ACC_MAG;
        break;
    default:
        break;
    }
}

#define X_VEL_CAP 400.0
#define Y_VEL_CAP 400.0
#define Z_VEL_CAP 5.0
void cap_vel(entity *e)
{
    if      (e->vel.x >  X_VEL_CAP) e->vel.x =  X_VEL_CAP;
    else if (e->vel.x < -X_VEL_CAP) e->vel.x = -X_VEL_CAP;
    else if (e->vel.y >  Y_VEL_CAP) e->vel.y =  Y_VEL_CAP;
    else if (e->vel.y < -Y_VEL_CAP) e->vel.y = -Y_VEL_CAP;
    else if (e->vel.z >  Z_VEL_CAP) e->vel.z =  Z_VEL_CAP;
    else if (e->vel.z < -Z_VEL_CAP) e->vel.z = -Z_VEL_CAP;
}

int main(int argc, char *argv[])
{
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0 ) {
        fprintf(stderr,"Error: could not initialized SDL\n");
        return 1;
    }

    window = SDL_CreateWindow("ts", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    //renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);

    // test render
    viewport vp(WINDOW_W, WINDOW_H);
    entity  player;
    control player_ctrl = CTRL_NONE;
    std::vector<renderable> render_list;
    render_list.push_back(renderable(&player));
    render(renderer, vp, render_list);

    SDL_Event event;
    double t = get_time();
    double t_new;
    double dt;
    while(1) {
        // event loop

        // get dt, update time
        t_new  = get_time();
        dt = t_new - t;
        t = t_new;

        // check for events
        if (SDL_PollEvent(&event)) {
            switch(event.type) {
            case SDL_QUIT:
                goto cleanup;
            case SDL_KEYDOWN:
            case SDL_KEYUP:
                player_ctrl = key2ctrl();
                break;
            }
        }

        // process controls
        handle_controls(&player, player_ctrl);

        // act all entities
        player.act(dt);
        cap_vel(&player);
        printf("%0.2f: %0.2f, %0.2f, %0.2f\n", dt, player.vel.x, player.vel.y, player.vel.z);


        // render final results
        render(renderer, vp, render_list);
        SDL_Delay(20);
    }

cleanup:
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
