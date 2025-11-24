#define _XOPEN_SOURCE_EXTENDED
#include <locale.h>
#include <ncurses.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
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
#define N_TARGETS 10

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

    return 0;   //free position 
}


Obstacle obs[N_OBSTACLES];
Targets tar[N_TARGETS];

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
    nodelay(stdscr,TRUE);   // getch() non-blockin
    curs_set(0);   // hide cursor


    init_ui();

    int w = getmaxx(viewWin);
    int h = getmaxy(viewWin);

    // obstacles 
    for (int i = 0; i < N_OBSTACLES; i++) {
        int y, x;
        do {
            y = rand() % (h - 2) + 1;
            x = rand() % (w - 2) + 1;
        } while (is_occupied(y, x, obs, i, tar, 0)); 
        // tar,0 perché ancora non hai inserito target

        obs[i].y_ob = y;
        obs[i].x_ob = x;
    }


    // targets
    for (int i = 0; i < N_TARGETS; i++) {
        int y, x;
        do {
            y = rand() % (h - 2) + 1;
            x = rand() % (w - 2) + 1;
        } while (is_occupied(y, x, obs, N_OBSTACLES, tar, i));
        // ora tutti gli ostacoli sono validi, e anche i primi i target

        tar[i].y_tar = y;
        tar[i].x_tar = x;
    }


    // Forces and physical parameters
    float Fx=0,Fy=0;
    float M=0.2,K=0.1,T=0.05;

    // Current drone state (world coordinates 0..100)
    StateMsg state={50,50,0,0};

    while(1){

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
        if(fdBtoD > maxfd) maxfd = fdBtoD;
        if(fdDtoB > maxfd) maxfd = fdDtoB;
        if(fdOtoB > maxfd) maxfd = fdOtoB;
        if(fdTtoB > maxfd) maxfd = fdTtoB;


        //int maxfd = (fdItoB>fdDtoB?fdItoB:fdDtoB);
        struct timeval tv={0,20000};  // 20 ms

        int rv=select(maxfd+1,&s,NULL,NULL,&tv);

        if(rv>0){

            // Input process -> blackboard
            if(FD_ISSET(fdItoB,&s)){
                KeyMsg km;
                if(read(fdItoB,&km,sizeof(km))>0){

                    if(km.cmd==9) break;       // quit
                    else if(km.cmd==1) {       // brake
                        Fx=Fy=0;
                    }
                    else if(km.cmd==2){        // reset
                        Fx=Fy=0;
                        ForceMsg fm={0,0,M,K,T,1};
                        write(fdBtoD,&fm,sizeof(fm));
                    }
                    else {
                        Fx+=km.dFx;
                        Fy+=km.dFy;
                    }
                }
            }

            // Drone -> blackboard (state update)
            if(FD_ISSET(fdDtoB,&s)){
                StateMsg sm;
                if(read(fdDtoB,&sm,sizeof(sm))>0)
                    state=sm;
            }

            if(FD_ISSET(fdOtoB, &s)){
                ObjMsg om;
                if(read(fdOtoB, &om, sizeof(om)) > 0){
                    int y = (int)((om.y/100.0f)*(h-2));
                    int x = (int)((om.x/100.0f)*(w-2));
                    if(!is_occupied(y,x,obs,N_OBSTACLES,tar,N_TARGETS)){
                        obs[om.id].y_ob = y;
                        obs[om.id].x_ob = x;
                    }
                }
            }

            if(FD_ISSET(fdTtoB, &s)){
                ObjMsg om;
                if(read(fdTtoB, &om, sizeof(om)) > 0){
                    int y = (int)((om.y/100.0f)*(h-2));
                    int x = (int)((om.x/100.0f)*(w-2));
                    if(!is_occupied(y,x,obs,N_OBSTACLES,tar,N_TARGETS)){
                        tar[om.id].y_tar = y;
                        tar[om.id].x_tar = x;
                    }
                }
            }

        }

        ForceMsg fm={Fx,Fy,M,K,T,0};
        write(fdBtoD,&fm,sizeof(fm));

        werase(viewWin);
        box(viewWin,0,0);

        // window coords for drone (from world coords 0..100)
        int cx = (int)((state.x/100.0f)*(w-2));
        int cy = (int)((state.y/100.0f)*(h-2));

        // draw obstacles
        for(int i=0; i<N_OBSTACLES; i++){
            mvwaddch(viewWin, (int)obs[i].y_ob, (int)obs[i].x_ob, 'O');
        }

        // draw targets
        for (int i = 0; i < N_TARGETS; i++) {
            mvwaddch(viewWin, (int)tar[i].y_tar, (int)tar[i].x_tar, '*');
        }

        // draw drone
        mvwaddch(viewWin, cy+1, cx+1, '+');
        wrefresh(viewWin);
    }

    cleanup(0);
    return 0;
}


