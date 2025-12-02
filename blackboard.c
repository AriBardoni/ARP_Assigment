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
    getmaxyx(stdscr,H,W);   // get terminal size
    viewWin = newwin(H, W, 0, 0);
    box(viewWin,0,0);
    wrefresh(viewWin);
}

#define N_OBSTACLES 10
#define N_TARGETS   10

typedef struct {
    float x_ob;   // window coordinate X
    float y_ob;   // window coordinate Y
} Obstacle;

typedef struct {
    float x_tar;   // window coordinate X
    float y_tar;   // window coordinate Y
} Targets;

// checks for occupied positions 
int is_occupied(int y, int x,
                Obstacle obs[], int n_obs,
                Targets tar[], int n_tar)
{
    for (int i = 0; i < n_obs; i++) {
        if ((int)obs[i].y_ob == y && (int)obs[i].x_ob == x)
            return 1;
    }

    for (int i = 0; i < n_tar; i++) {
        if ((int)tar[i].y_tar == y && (int)tar[i].x_tar == x)
            return 1;
    }

    return 0;   // free position 
}

Obstacle obs[N_OBSTACLES];
Targets  tar[N_TARGETS];

static void compute_repulsive_force(const StateMsg *state,
                                    Obstacle obs[], int n_obs,
                                    int w, int h,
                                    float *Frx, float *Fry)
{
    const float RHO  = 8.0f;      // raggio di influenza (world coords 0..100)
    const float ETA  = 6.0f;      // intensità campo repulsivo
    const float STEP = 2.5f;      // modulo del "tasto virtuale"

    float Px = 0.0f;
    float Py = 0.0f;

    if (w <= 2 || h <= 2) {
        *Frx = *Fry = 0.0f;
        return;
    }

    // 1) Calcolo P = somma delle forze repulsive continue
    for (int i = 0; i < n_obs; i++) {

        float ox = (obs[i].x_ob - 1.0f) * 100.0f / (float)(w - 2);
        float oy = (obs[i].y_ob - 1.0f) * 100.0f / (float)(h - 2);

        float dx = state->x - ox;
        float dy = state->y - oy;
        float dist = sqrtf(dx*dx + dy*dy);
        if (dist < 1.0f) dist = 1.0f;
        if (dist > RHO) continue;

        // Forza più “ripida” vicino all’ostacolo
        float coeff = ETA * powf((1.0f/dist - 1.0f/RHO), 2.0f);
        Px += coeff * (dx / dist);
        Py += coeff * (dy / dist);
    }

    // if P=0 no correction 
    float Pnorm = sqrtf(Px*Px + Py*Py);
    if (Pnorm < 1e-4f) {
        *Frx = *Fry = 0.0f;
        return;
    }

    // projection on the 8 directions 
    const float s = 1.0f / sqrtf(2.0f);
    const float dirs[8][2] = {
        {  1.0f,  0.0f },   // rr
        { -1.0f,  0.0f },   // ll
        {  0.0f, -1.0f },   // uu
        {  0.0f,  1.0f },   // dd
        {  s,   -s   },     // ur
        {  s,    s   },     // dr
        { -s,   -s   },     // ul
        { -s,    s   }      // dl
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
    if(argc<6){
        fprintf(stderr,"blackboard: missing fds\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);  // input -> blackboard
    int fdBtoD = atoi(argv[2]);  // blackboard -> drone
    int fdDtoB = atoi(argv[3]);  // drone -> blackboard
    int fdOtoB = atoi(argv[4]);  // obstacles -> blackboard 
    int fdTtoB = atoi(argv[5]);  // targets -> blackboard 

    signal(SIGINT,cleanup);

    setlocale(LC_ALL,"");
    initscr();
    noecho();
    cbreak();
    keypad(stdscr,TRUE);
    nodelay(stdscr,TRUE);
    curs_set(0);

    init_ui();

    int w = getmaxx(viewWin);
    int h = getmaxy(viewWin);

    // ---- Inizializzazione ostacoli ----
    for (int i = 0; i < N_OBSTACLES; i++) {
        int y, x;
        do {
            y = rand() % (h - 2) + 1;
            x = rand() % (w - 2) + 1;
        } while (is_occupied(y, x, obs, i, tar, 0)); 

        obs[i].y_ob = y;
        obs[i].x_ob = x;
    }

    // ---- Inizializzazione target ----
    for (int i = 0; i < N_TARGETS; i++) {
        int y, x;
        do {
            y = rand() % (h - 2) + 1;
            x = rand() % (w - 2) + 1;
        } while (is_occupied(y, x, obs, N_OBSTACLES, tar, i));

        tar[i].y_tar = y;
        tar[i].x_tar = x;
    }

    float Fx=0,Fy=0;
    float M=0.2f,K=0.1f,T=0.05f;

    StateMsg state={50,50,0,0};

    while(1){
        int ch = getch();
        if (ch == KEY_RESIZE) {
            int w_old=w,h_old=h;
            resize_term(0,0);
            getmaxyx(stdscr,h,w);
            delwin(viewWin);
            viewWin=newwin(h,w,0,0);
            box(viewWin,0,0);
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
        if(fdBtoD>maxfd) maxfd=fdBtoD;
        if(fdDtoB>maxfd) maxfd=fdDtoB;
        if(fdOtoB>maxfd) maxfd=fdOtoB;
        if(fdTtoB>maxfd) maxfd=fdTtoB;

        struct timeval tv={0,20000};
        int rv=select(maxfd+1,&s,NULL,NULL,&tv);

        if(rv>0){
            // ---- INPUT ----
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
                    } else {
                        Fx+=km.dFx;
                        Fy+=km.dFy;
                    }
                }
            }

            // ---- STATO DRONE ----
            if(FD_ISSET(fdDtoB,&s)){
                StateMsg sm;
                if(read(fdDtoB,&sm,sizeof(sm))>0)
                    state=sm;
            }

            // ---- OSTACOLI ----
            if(FD_ISSET(fdOtoB,&s)){
                ObjMsg om;
                if(read(fdOtoB,&om,sizeof(om))>0){
                    int y=(int)((om.y/100.0f)*(h-2));
                    int x=(int)((om.x/100.0f)*(w-2));
                    if(!is_occupied(y,x,obs,N_OBSTACLES,tar,N_TARGETS)){
                        obs[om.id].y_ob=y;
                        obs[om.id].x_ob=x;
                    }
                }
            }

            // ---- TARGETS ----
            if(FD_ISSET(fdTtoB,&s)){
                ObjMsg om;
                if(read(fdTtoB,&om,sizeof(om))>0){
                    int y=(int)((om.y/100.0f)*(h-2));
                    int x=(int)((om.x/100.0f)*(w-2));
                    if(!is_occupied(y,x,obs,N_OBSTACLES,tar,N_TARGETS)){
                        tar[om.id].y_tar=y;
                        tar[om.id].x_tar=x;
                    }
                }
            }
        }

        // ---- Calcolo forze totali ----
        float Frx=0,Fry=0;
        compute_repulsive_force(&state, obs, N_OBSTACLES, w, h, &Frx, &Fry);

        float totalFx = Fx + Frx;
        float totalFy = Fy + Fry;

        // --- Fermati vicino ai bordi ---
        /*const float MARGIN = 5.0f;
        if(state.x < MARGIN && totalFx < 0.0f)  totalFx = 0.0f;
        if(state.x > 100.0f - MARGIN && totalFx > 0.0f) totalFx = 0.0f;
        if(state.y < MARGIN && totalFy < 0.0f)  totalFy = 0.0f;
        if(state.y > 100.0f - MARGIN && totalFy > 0.0f) totalFy = 0.0f;
        */

        const float MARGIN = 5.0f;
        // left border
        if(state.x < MARGIN && totalFx < 0.0f)  totalFx = -totalFx;
        // right border
        if(state.x > 100.0f - MARGIN && totalFx > 0.0f) totalFx = -totalFx;
        // upper border 
        if(state.y < MARGIN && totalFy < 0.0f)  totalFy = -totalFy;
        // bottom border 
        if(state.y > 100.0f - MARGIN && totalFy > 0.0f) totalFy = -totalFy;


        ForceMsg fm={totalFx, totalFy, M,K,T,0};
        write(fdBtoD,&fm,sizeof(fm));

        werase(viewWin);
        box(viewWin,0,0);

        int cx=(int)((state.x/100.0f)*(w-3));
        int cy=(int)((state.y/100.0f)*(h-3));

        for(int i=0;i<N_OBSTACLES;i++)
            mvwaddch(viewWin,(int)obs[i].y_ob,(int)obs[i].x_ob,'O');

        for(int i=0;i<N_TARGETS;i++)
            mvwaddch(viewWin,(int)tar[i].y_tar,(int)tar[i].x_tar,'*');

        mvwaddch(viewWin,cy+1,cx+1,'+');
        wrefresh(viewWin);
    }

    cleanup(0);
    return 0;
}
