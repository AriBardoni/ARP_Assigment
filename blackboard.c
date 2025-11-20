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

typedef struct {
    float x_ob;   // window coordinate X
    float y_ob;   // window coordinate Y
} Obstacle;

Obstacle obs[N_OBSTACLES];

int main(int argc,char **argv){
    if(argc<4){
        fprintf(stderr,"blackboard: missing fds\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);  // input -> blackboard
    int fdBtoD = atoi(argv[2]);  // blackboard -> drone
    int fdDtoB = atoi(argv[3]);  // drone -> blackboard

    signal(SIGINT,cleanup);

    setlocale(LC_ALL,"");
    initscr();
    noecho();
    cbreak();
    keypad(stdscr,TRUE);
    nodelay(stdscr,TRUE);   // getch() non-blocking

    init_ui();

    int w = getmaxx(viewWin);
    int h = getmaxy(viewWin);

    // --- generate static obstacles (window coords) ---
    srand( (unsigned) time(NULL) );
    for(int i=0; i<N_OBSTACLES; i++){
        obs[i].y_ob = rand() % (h-2) + 1;  // inside box
        obs[i].x_ob = rand() % (w-2) + 1;
    }

    // Forces and physical parameters
    float Fx=0,Fy=0;
    float M=1,K=1,T=0.05;

    // Current drone state (world coordinates 0..100)
    StateMsg state={50,50,0,0};

    while(1){

        // ------------- HANDLE RESIZE (mouse) -------------
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

            // il drone NON va riscalato qui.
            // Verrà ridisegnato con il nuovo w,h più sotto,
            // usando state.x/state.y (0..100) -> mapping proporzionale automatico.

            wrefresh(viewWin);
            // dopo il resize, saltiamo al prossimo frame
            continue;
        }

        // ------------- SELECT SU PIPE -------------
        fd_set s;
        FD_ZERO(&s);
        FD_SET(fdItoB,&s);
        FD_SET(fdDtoB,&s);

        int maxfd = (fdItoB>fdDtoB?fdItoB:fdDtoB);
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
        }

        // ------------- SEND FORCE TO DRONE -------------
        ForceMsg fm={Fx,Fy,M,K,T,0};
        write(fdBtoD,&fm,sizeof(fm));

        // ------------- DRAW SECTION -----------------
        werase(viewWin);
        box(viewWin,0,0);

        // window coords for drone (from world coords 0..100)
        int cx = (int)((state.x/100.0f)*(w-2));
        int cy = (int)((state.y/100.0f)*(h-2));

        // draw obstacles
        for(int i=0; i<N_OBSTACLES; i++){
            mvwaddch(viewWin, (int)obs[i].y_ob, (int)obs[i].x_ob, 'O');
        }

        // draw drone
        mvwaddch(viewWin, cy+1, cx+1, '+');
        wrefresh(viewWin);
    }

    cleanup(0);
    return 0;
}

