#define _XOPEN_SOURCE_EXTENDED
#include <locale.h>
#include <ncurses.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "common.h"

// Windows used for drawing (map + info)
static WINDOW *viewWin,*infoWin;

// Called when user presses CTRL+C or the process must exit
static void cleanup(int sig){
    endwin();               // restore terminal from ncurses mode
    system("stty sane");    // clean terminal state
    _exit(0);               // force exit
}

// Initialize the two panels: simulation view + info panel
static void init_ui(){
    int H,W;
    getmaxyx(stdscr,H,W);   // get terminal size

    int info_h=6;           // height of the info panel
    int view_h=H-info_h;    // rest of the screen is the view

    // create the two windows
    viewWin=newwin(view_h,W,0,0);
    infoWin=newwin(info_h,W,view_h,0);

    // draw borders
    box(viewWin,0,0);
    box(infoWin,0,0);

    // refresh both
    wrefresh(viewWin);
    wrefresh(infoWin);
}

int main(int argc,char **argv){
    if(argc<4){
        fprintf(stderr,"blackboard: missing fds\n");
        return 1;
    }

    // pipe descriptors passed via command line
    int fdItoB = atoi(argv[1]);  // input -> blackboard
    int fdBtoD = atoi(argv[2]);  // blackboard -> drone
    int fdDtoB = atoi(argv[3]);  // drone -> blackboard

    signal(SIGINT,cleanup);

    setlocale(LC_ALL,"");
    initscr();       // start ncurses
    noecho();        // do not print typed chars
    cbreak();        // disable line buffering
    keypad(stdscr,TRUE);

    init_ui();

    // Forces and physical parameters
    float Fx=0,Fy=0;   // accumulated force
    float M=1,K=1,T=0.05;

    // Current drone state received from drone process
    StateMsg state={50,50,0,0};

    while(1){

        // We prepare for select() to listen on two fds:
        // - fdItoB : input commands
        // - fdDtoB : drone state updates
        fd_set s;
        FD_ZERO(&s);
        FD_SET(fdItoB,&s);
        FD_SET(fdDtoB,&s);

        int maxfd = (fdItoB>fdDtoB?fdItoB:fdDtoB);

        // Wait max 20ms for input or drone updates
        struct timeval tv={0,20000};

        int rv=select(maxfd+1,&s,NULL,NULL,&tv);

        if(rv>0){

            // If input process sent a message
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
                        // normal movement: accumulate forces
                        Fx+=km.dFx;
                        Fy+=km.dFy;
                    }
                }
            }

            // If drone sent updated position
            if(FD_ISSET(fdDtoB,&s)){
                StateMsg sm;
                if(read(fdDtoB,&sm,sizeof(sm))>0)
                    state=sm;
            }
        }

        // Send forces to the drone
        ForceMsg fm={Fx,Fy,M,K,T,0};
        write(fdBtoD,&fm,sizeof(fm));

        // DRAWING SECTION ----------------------

        werase(viewWin);
        box(viewWin,0,0);

        int w=getmaxx(viewWin);
        int h=getmaxy(viewWin);

        // Convert world coordinates (0-100) to window coordinates
        int cx = (int)((state.x/100)*(w-2));
        int cy = (int)((state.y/100)*(h-2));

        // Draw drone as '+'
        mvwaddch(viewWin,cy+1,cx+1,'+');
        wrefresh(viewWin);

        // INFO PANEL
        werase(infoWin);
        box(infoWin,0,0);

        mvwprintw(infoWin,1,2,"Pos: %.1f %.1f",state.x,state.y);
        mvwprintw(infoWin,2,2,"Vel: %.2f %.2f",state.vx,state.vy);
        mvwprintw(infoWin,3,2,"Force: %.2f %.2f",Fx,Fy);

        mvwprintw(infoWin,4,2,"Press q in INPUT tab to quit");
        wrefresh(infoWin);
    }

    cleanup(0);
    return 0;
}
