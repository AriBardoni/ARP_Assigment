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
#include <string.h>
#include "common.h"
#include <time.h>

#define N_OBSTACLES 10
#define N_TARGETS   10
#define TARGET_RADIUS 4.0f
#define DRONE_X0 50
#define DRONE_Y0 50
#define ATTRACTION 1.5f
#define ATTRACT_RADIUS 6.0f   // <-- AGGIUNTO: raggio entro cui parte l'attrazione

// draw period in seconds
double DRAW_T = 0.05;

float OBS_RHO = 8.0f;
float OBS_ETA = 6.0f;
float OBS_STEP = 2.5f;

float WALL_RHO = 6.0f;
float WALL_ETA = 5.0f;
float WALL_MAX = 2.0f;

int SAFE_DIST = 10;
int BORDER_MARGIN = 5;

int current_target = 0;

// main view window
static WINDOW *viewWin;

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

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
    int active;
    int taken;
} Targets;

Obstacle obs[N_OBSTACLES];
Targets  tar[N_TARGETS];

// function for reading the parameters from the file
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

// check if the position of the spaw isn't on the borders or too near to the drone
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

// check that the position for the spawn isn't already occupied
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

void draw_map(StateMsg state, int w, int h){

    // drawing the map
    werase(viewWin);
    box(viewWin,0,0);

    int cx=(int)((state.x/100.0f)*(w-3));
    int cy=(int)((state.y/100.0f)*(h-3));

    for(int i = 0; i < N_OBSTACLES; i++){
        mvwaddch(
            viewWin,
            (int)obs[i].y_ob,
            (int)obs[i].x_ob,
            'X'
        );
    }

    for (int i = 0; i < N_TARGETS; i++) {

        if (tar[i].taken)
            continue;   // target taken, no redraw

        if (tar[i].active) {
            wattron(viewWin, COLOR_PAIR(2));
        }

        mvwprintw(
            viewWin,
            (int)tar[i].y_tar,
            (int)tar[i].x_tar,
            "%d",
            i
        );

        if (tar[i].active) {
            wattroff(viewWin, COLOR_PAIR(2));
        }
    }

    wattron(viewWin, COLOR_PAIR(1));
    mvwaddch(viewWin, cy+1, cx+1, '+');
    wattroff(viewWin, COLOR_PAIR(1));

    wrefresh(viewWin);
}

// repulsive force
static void compute_repulsive_force(const StateMsg *state, Obstacle obs[], int n_obs, int w, int h, float *Frx, float *Fry)
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

static void compute_attractive_force(const StateMsg *state, Targets tar[], int n_targets, int w, int h,
                                     float attraction, float *Fax, float *Fay)
{
    *Fax = 0.0f;
    *Fay = 0.0f;

    int active_idx = -1;

    // trova il target attivo
    for (int i = 0; i < n_targets; i++) {
        if (tar[i].active && !tar[i].taken) {
            active_idx = i;
            break;
        }
    }

    if (active_idx < 0)
        return;   // nessun target attivo

    // target screen -> world (COERENTE con il drone)
    float tx = (tar[active_idx].x_tar - 1.0f) * 100.0f / (float)(w - 3);
    float ty = (tar[active_idx].y_tar - 1.0f) * 100.0f / (float)(h - 3);

    float dx = tx - state->x;
    float dy = ty - state->y;
    float dist = sqrtf(dx*dx + dy*dy);

    // <-- AGGIUNTO: attrazione solo se entro un certo raggio
    if (dist > ATTRACT_RADIUS)
        return;

    // troppo vicino → niente forza (evita jitter)
    if (dist < 1.0f)
        return;

    // direzioni discrete (stesse della repulsione)
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

    // vettore unitario verso il target
    float vx = dx / dist;
    float vy = dy / dist;

    int best_i = -1;
    float best_dot = -1e9f;

    for (int i = 0; i < 8; i++) {
        float dot = vx * dirs[i][0] + vy * dirs[i][1];
        if (dot > best_dot) {
            best_dot = dot;
            best_i = i;
        }
    }

    if (best_i >= 0) {
        *Fax = dirs[best_i][0] * attraction;
        *Fay = dirs[best_i][1] * attraction;
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
    pid_t wd_pid = (pid_t)atoi(argv[6]);

    signal(SIGINT,cleanup);

    setlocale(LC_ALL,"");
    initscr();

    if (has_colors()) {
        start_color();
        use_default_colors();

        // Pair 1: drone rosa (magenta)
        init_pair(1, COLOR_RED, -1);
        init_pair(2, COLOR_YELLOW, -1);
    }

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
        tar[i].taken = 0;
        tar[i].active = (i == 0);
    }

    float Fx=0,Fy=0;
    float M=0.2f,K=0.1f,T=0.05f;

    StateMsg state={50,50,0,0};
    int counter = 0;

    double last_draw_time = 0.0;

    // Log startup
    log_message("LogFile2", "Blackboard", "Process started");

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

                mvwaddch(viewWin, (int)obs[i].y_ob, (int)obs[i].x_ob, 'X');
            }

            // targets proportionally to new size
            for (int i = 0; i < N_TARGETS; i++) {

                float rel_ox = (tar[i].x_tar - 1) / (float)(w_old - 2);
                float rel_oy = (tar[i].y_tar - 1) / (float)(h_old - 2);

                tar[i].x_tar = 1 + rel_ox * (w - 2);
                tar[i].y_tar = 1 + rel_oy * (h - 2);

                mvwprintw(
                    viewWin,
                    (int)tar[i].y_tar,
                    (int)tar[i].x_tar,
                    "%d",
                    i + 1
                );
            }

            wrefresh(viewWin);
            draw_map(state, w, h);
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
                        ForceMsg reset = {0, 0, M, K, T, 1, 0, 0.0f};
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

                        if(tar[om.id].taken) continue;

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
        float Frx = 0.0f, Fry = 0.0f;
        compute_repulsive_force(&state, obs, N_OBSTACLES, w, h, &Frx, &Fry);

        float Fax = 0.0f, Fay = 0.0f;
        compute_attractive_force(&state, tar, N_TARGETS, w, h, ATTRACTION, &Fax, &Fay);

        float totalFx = Fx + Frx + Fax;
        float totalFy = Fy + Fry + Fay;

        ForceMsg fm = { totalFx, totalFy, M, K, T, 0, 0, 0.0f };
        write(fdBtoD, &fm, sizeof(fm));

        // target collection check
        if (current_target < N_TARGETS &&
            tar[current_target].taken == 0 &&
            tar[current_target].active == 1)
        {
            float tx = (tar[current_target].x_tar - 1.0f) * 100.0f / (w - 2);
            float ty = (tar[current_target].y_tar - 1.0f) * 100.0f / (h - 2);

            float dx = state.x - tx;
            float dy = state.y - ty;

            if (dx*dx + dy*dy < TARGET_RADIUS*TARGET_RADIUS) {

                // target taken
                tar[current_target].taken  = 1;
                tar[current_target].active = 0;

                current_target++;

                // next target becomes active
                if (current_target < N_TARGETS) {
                    tar[current_target].active = 1;
                } else {
                    // all target collected
                    // HERE: SCORE, FINISH GAME
                }
            }
        }

        double current_time = now_sec();

        if (current_time - last_draw_time >= DRAW_T) {
            draw_map(state, w, h);
            last_draw_time = current_time;
        }

        // Watchdog signal
        counter++;
        if (counter >= 50) { // 50 * 20ms = 1s
            union sigval value;
            value.sival_int = PROCESS_BLACKBOARD | (AREA_UPDATE_MAP << 8);
            sigqueue(wd_pid, SIGUSR1, value);
            counter = 0;
        }
    }

    cleanup(0);
    return 0;
}



