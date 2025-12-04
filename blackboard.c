#define _XOPEN_SOURCE_EXTENDED
#include <locale.h>
#include <ncurses.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include "common.h"

#define N_OBSTACLES 10
#define N_TARGETS   10
#define DRONE_X0 50
#define DRONE_Y0 50

float OBS_RHO = 8.0f;
float OBS_ETA = 6.0f;
float OBS_STEP = 2.5f;

float WALL_RHO = 6.0f;
float WALL_ETA = 5.0f;
float WALL_MAX = 2.0f;

int SAFE_DIST = 10;
int BORDER_MARGIN = 5;

// main view window
static WINDOW *viewWin;

// clean exit on CTRL+C
static void cleanup(int sig){
    (void)sig;
    endwin();
    system("stty sane");
    _exit(0);
}

static void init_ui(){
    int H,W;
    getmaxyx(stdscr,H,W);
    viewWin = newwin(H, W, 0, 0);
    box(viewWin,0,0);
    wrefresh(viewWin);
}  

typedef struct {
    float x_ob;
    float y_ob;
} Obstacle;

typedef struct {
    float x_tar;
    float y_tar;
} Targets;

Obstacle obs[N_OBSTACLES];
Targets  tar[N_TARGETS];

void load_params() {
    FILE *f = fopen("params.txt", "r");
    if(!f) return;

    char key[64];
    float val;

    while (fscanf(f, "%63[^=]=%f\n", key, &val) == 2) {

        if(strcmp(key, "OBS_RHO") == 0)        OBS_RHO = val;
        else if(strcmp(key, "OBS_ETA") == 0)   OBS_ETA = val;
        else if(strcmp(key, "OBS_STEP") == 0)  OBS_STEP = val;

        else if(strcmp(key, "WALL_RHO") == 0)  WALL_RHO = val;
        else if(strcmp(key, "WALL_ETA") == 0)  WALL_ETA = val;
        else if(strcmp(key, "WALL_MAX") == 0)  WALL_MAX = val;

        else if(strcmp(key, "SAFE_DIST") == 0) SAFE_DIST = (int)val;
        else if(strcmp(key, "BORDER_MARGIN") == 0) BORDER_MARGIN = (int)val;
    }

    fclose(f);
}

int check_spawn_ok(int x, int y, int w, int h)
{
    float wx = (x - 1.0f) * 100.0f / (float)(w - 2);
    float wy = (y - 1.0f) * 100.0f / (float)(h - 2);

    // distance frome drone
    float dx = wx - DRONE_X0;
    float dy = wy - DRONE_Y0;

    if (dx*dx + dy*dy < SAFE_DIST*SAFE_DIST)
        return 0;

    // check borders 
    if (x <= BORDER_MARGIN || x >= w - BORDER_MARGIN - 1)
        return 0;

    if (y <= BORDER_MARGIN || y >= h - BORDER_MARGIN - 1)
        return 0;

    return 1;
}

int is_occupied(int y, int x, Obstacle obs[], int n_obs, Targets tar[], int n_tar)
{
    for (int i = 0; i < n_obs; i++) {
        if ((int)obs[i].y_ob == y && (int)obs[i].x_ob == x)
            return 1;
    }

    for (int i = 0; i < n_tar; i++) {
        if ((int)tar[i].y_tar == y && (int)tar[i].x_tar == x)
            return 1;
    }

    return 0;
}

// repulsive force 
static void compute_repulsive_force(const StateMsg *state,
                                    Obstacle obs[], int n_obs,
                                    int w, int h,
                                    float *Frx, float *Fry)
{
    const float RHO  = 8.0f;
    const float ETA  = 6.0f;
    const float STEP = 2.5f;

    float Px = 0.0f;
    float Py = 0.0f;

    if (w <= 2 || h <= 2) {
        *Frx = *Fry = 0.0f;
        return;
    }

    for (int i = 0; i < n_obs; i++) {

        float ox = (obs[i].x_ob - 1.0f) * 100.0f / (float)(w - 2);
        float oy = (obs[i].y_ob - 1.0f) * 100.0f / (float)(h - 2);

        float dx = state->x - ox;
        float dy = state->y - oy;
        float dist = sqrtf(dx*dx + dy*dy);

        if (dist < 1.0f) dist = 1.0f;
        if (dist > RHO)  continue;

        float coeff = ETA * powf((1.0f/dist - 1.0f/RHO), 2.0f);

        Px += coeff * (dx / dist);
        Py += coeff * (dy / dist);
    }

    float Pnorm = sqrtf(Px*Px + Py*Py);

    if (Pnorm < 1e-4f) {
        *Frx = *Fry = 0.0f;
        return;
    }

    const float s = 1.0f / sqrtf(2.0f);
    const float dirs[8][2] = {
        {  1.0f,  0.0f },
        { -1.0f,  0.0f },
        {  0.0f, -1.0f },
        {  0.0f,  1.0f },
        {  s,   -s   },
        {  s,    s   },
        { -s,   -s   },
        { -s,    s   }
    };

    int best_i = -1;
    float best_dot = 0.0f;

    for (int i = 0; i < 8; i++) {
        float dot = Px * dirs[i][0] + Py * dirs[i][1];
        if (dot > best_dot) {
            best_dot = dot;
            best_i = i;
        }
    }

    if (best_i >= 0) {
        *Frx = dirs[best_i][0] * STEP;
        *Fry = dirs[best_i][1] * STEP;
    } else {
        *Frx = *Fry = 0.0f;
    }
}

int main(int argc,char **argv){
    if(argc < 6){
        fprintf(stderr,"blackboard: missing fds\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);
    int fdBtoD = atoi(argv[2]);
    int fdDtoB = atoi(argv[3]);
    int fdOtoB = atoi(argv[4]);
    int fdTtoB = atoi(argv[5]);

    signal(SIGINT,cleanup);

    setlocale(LC_ALL,"");
    initscr();
    noecho();
    cbreak();
    keypad(stdscr,TRUE);
    nodelay(stdscr,TRUE);
    curs_set(0);

    init_ui();
    load_params();

    int w = getmaxx(viewWin);
    int h = getmaxy(viewWin);

    // obstacle spawn 
    for (int i = 0; i < N_OBSTACLES; i++) {
        int y, x;
        do {
            y = rand() % (h - 2) + 1;
            x = rand() % (w - 2) + 1;
        } while (!check_spawn_ok(x,y,w,h) ||
                 is_occupied(y, x, obs, i, tar, 0));

        obs[i].y_ob = y;
        obs[i].x_ob = x;
    }

    // target spawn 
    for (int i = 0; i < N_TARGETS; i++) {
        int y, x;
        do {
            y = rand() % (h - 2) + 1;
            x = rand() % (w - 2) + 1;
        } while (!check_spawn_ok(x,y,w,h) ||
                 is_occupied(y, x, obs, N_OBSTACLES, tar, i));

        tar[i].y_tar = y;
        tar[i].x_tar = x;
    }

    float Fx=0,Fy=0;
    float M=0.2f,K=0.1f,T=0.05f;

    StateMsg state={50,50,0,0};

    while(1){

        load_params();

        // resize 
        int ch = getch();
        if (ch == KEY_RESIZE) {

            int w_old = w;
            int h_old = h;

            // update ncurses internal size info
            resize_term(0,0);

            int newH, newW;
            getmaxyx(stdscr, newH, newW);

            delwin(viewWin);
            viewWin = newwin(newH, newW, 0, 0);

            w = newW;
            h = newH;

            box(viewWin,0,0);

            // rescale obstacles proportionally to new size
            for (int i = 0; i < N_OBSTACLES; i++) {

                float rel_ox = (obs[i].x_ob - 1) / (float)(w_old - 2);
                float rel_oy = (obs[i].y_ob - 1) / (float)(h_old - 2);

                obs[i].x_ob = 1 + rel_ox * (w - 2);
                obs[i].y_ob = 1 + rel_oy * (h - 2);

                mvwaddch(viewWin, (int)obs[i].y_ob, (int)obs[i].x_ob, 'O');
            }

            // targets proportionally to new size
            for (int i = 0; i < N_TARGETS; i++) {

                float rel_ox = (tar[i].x_tar - 1) / (float)(w_old - 2);
                float rel_oy = (tar[i].y_tar - 1) / (float)(h_old - 2);

                tar[i].x_tar = 1 + rel_ox * (w - 2);
                tar[i].y_tar = 1 + rel_oy * (h - 2);

                mvwaddch(viewWin, (int)tar[i].y_tar, (int)tar[i].x_tar, '*');
            }

            wrefresh(viewWin);
            continue;
        }
        
        fd_set s;
        FD_ZERO(&s);
        FD_SET(fdItoB,&s);
        FD_SET(fdDtoB,&s);
        FD_SET(fdOtoB,&s);
        FD_SET(fdTtoB,&s);

        int maxfd = fdItoB;
        if(fdDtoB>maxfd) maxfd=fdDtoB;
        if(fdOtoB>maxfd) maxfd=fdOtoB;
        if(fdTtoB>maxfd) maxfd=fdTtoB;

        struct timeval tv={0,20000};
        int rv = select(maxfd+1,&s,NULL,NULL,&tv);

        if(rv>0){

            // input 
            if(FD_ISSET(fdItoB,&s)){
                KeyMsg km;
                if(read(fdItoB,&km,sizeof(km))>0){

                    if(km.cmd==9) break;

                    else if(km.cmd==1){
                        Fx=Fy=0;
                    }

                    else if(km.cmd==2){
                        Fx=Fy=0;
                        ForceMsg reset={0,0,M,K,T,1};
                        write(fdBtoD,&reset,sizeof(reset));
                    }

                    else {
                        Fx += km.dFx;
                        Fy += km.dFy;
                    }
                }
            }

            // drone state 
            if(FD_ISSET(fdDtoB,&s)){
                StateMsg sm;
                if(read(fdDtoB,&sm,sizeof(sm))>0)
                    state = sm;
            }

            // obstacles
            if(FD_ISSET(fdOtoB,&s)){
                ObjMsg om;
                if(read(fdOtoB,&om,sizeof(om))>0){
                    if(om.id >= 0 && om.id < N_OBSTACLES){

                        int y = (int)((om.y/100.0f)*(h-2));
                        int x = (int)((om.x/100.0f)*(w-2));

                        if(!is_occupied(y,x,obs,N_OBSTACLES,tar,N_TARGETS))
                        {
                            obs[om.id].y_ob=y;
                            obs[om.id].x_ob=x;
                        }
                    }
                }
            }

            // targets
            if(FD_ISSET(fdTtoB,&s)){
                ObjMsg om;
                if(read(fdTtoB,&om,sizeof(om))>0){
                    if(om.id >= 0 && om.id < N_TARGETS){

                        int y = (int)((om.y/100.0f)*(h-2));
                        int x = (int)((om.x/100.0f)*(w-2));

                        if(!is_occupied(y,x,obs,N_OBSTACLES,tar,N_TARGETS))
                        {
                            tar[om.id].y_tar=y;
                            tar[om.id].x_tar=x;
                        }
                    }
                }
            }
        }

        // computation of forces 
        float Frx=0,Fry=0;
        compute_repulsive_force(&state, obs, N_OBSTACLES, w, h, &Frx, &Fry);

        float WFx=0, WFy=0;
        float rho_wall = 6.0f;
        float eta_wall = 5.0f;
        float MAX_REP_WALL = 2.0f;

        // wall forces
        if (state.x < rho_wall) {
            float d = fmaxf(state.x,0.001f);
            float F = eta_wall * ((1.0f/d)-(1.0f/rho_wall)) * (1.0f/(d*d));
            if(F > MAX_REP_WALL) F=MAX_REP_WALL;
            WFx += F;
        }

        if ((100.0f - state.x) < rho_wall) {
            float d = fmaxf((100.0f - state.x),0.001f);
            float F = eta_wall * ((1.0f/d)-(1.0f/rho_wall)) * (1.0f/(d*d));
            if(F > MAX_REP_WALL) F=MAX_REP_WALL;
            WFx -= F;
        }

        if (state.y < rho_wall) {
            float d = fmaxf(state.y,0.001f);
            float F = eta_wall * ((1.0f/d)-(1.0f/rho_wall)) * (1.0f/(d*d));
            if(F > MAX_REP_WALL) F=MAX_REP_WALL;
            WFy += F;
        }

        if ((100.0f - state.y) < rho_wall) {
            float d = fmaxf((100.0f - state.y),0.001f);
            float F = eta_wall * ((1.0f/d)-(1.0f/rho_wall)) * (1.0f/(d*d));
            if(F > MAX_REP_WALL) F=MAX_REP_WALL;
            WFy -= F;
        }

        float totalFx = Fx + Frx + WFx;
        float totalFy = Fy + Fry + WFy;

        ForceMsg fm = { totalFx, totalFy, M,K,T,0 };
        write(fdBtoD, &fm, sizeof(fm));

        // drawing the map 
        werase(viewWin);
        box(viewWin,0,0);

        int cx=(int)((state.x/100.0f)*(w-3));
        int cy=(int)((state.y/100.0f)*(h-3));

        for(int i=0;i<N_OBSTACLES;i++){
            mvwaddch(viewWin,(int)obs[i].y_ob,(int)obs[i].x_ob,'O');
        }

        for(int i=0;i<N_TARGETS;i++){
            mvwaddch(viewWin,(int)tar[i].y_tar,(int)tar[i].x_tar,'*');
        }

        mvwaddch(viewWin,cy+1,cx+1,'+');
        wrefresh(viewWin);
    }

    cleanup(0);
    return 0;
}
