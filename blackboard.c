#define _XOPEN_SOURCE_EXTENDED
#include <locale.h>
#include <ncurses.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "common.h"

static WINDOW *viewWin,*infoWin;

static void cleanup(int sig){
    endwin();
    system("stty sane");
    _exit(0);
}

static void init_ui(){
    int H,W;
    getmaxyx(stdscr,H,W);

    int info_h=6;
    int view_h=H-info_h;

    viewWin=newwin(view_h,W,0,0);
    infoWin=newwin(info_h,W,view_h,0);

    box(viewWin,0,0);
    box(infoWin,0,0);

    wrefresh(viewWin);
    wrefresh(infoWin);
}

int main(int argc,char **argv){
    if(argc<4){
        fprintf(stderr,"blackboard: missing fds\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);
    int fdBtoD = atoi(argv[2]);
    int fdDtoB = atoi(argv[3]);

    signal(SIGINT,cleanup);

    setlocale(LC_ALL,"");
    initscr();
    noecho();
    cbreak();
    keypad(stdscr,TRUE);

    init_ui();

    float Fx=0,Fy=0,M=1,K=1,T=0.05;
    StateMsg state={50,50,0,0};

    while(1){

        // ------ READ INPUT PIPE ------
        fd_set s;
        FD_ZERO(&s);
        FD_SET(fdItoB,&s);
        FD_SET(fdDtoB,&s);

        int maxfd = (fdItoB>fdDtoB?fdItoB:fdDtoB);
        struct timeval tv={0,20000};

        int rv=select(maxfd+1,&s,NULL,NULL,&tv);

        if(rv>0){
            if(FD_ISSET(fdItoB,&s)){
                KeyMsg km;
                if(read(fdItoB,&km,sizeof(km))>0){
                    if(km.cmd==9) break;
                    else if(km.cmd==1) { Fx=Fy=0; }
                    else if(km.cmd==2){ Fx=Fy=0;
                        ForceMsg fm={0,0,M,K,T,1};
                        write(fdBtoD,&fm,sizeof(fm));
                    } else {
                        Fx+=km.dFx;
                        Fy+=km.dFy;
                    }
                }
            }

            if(FD_ISSET(fdDtoB,&s)){
                StateMsg sm;
                if(read(fdDtoB,&sm,sizeof(sm))>0) state=sm;
            }
        }

        // SEND FORCE
        ForceMsg fm={Fx,Fy,M,K,T,0};
        write(fdBtoD,&fm,sizeof(fm));

        // -------- DRAW VIEW --------
        werase(viewWin);
        box(viewWin,0,0);

        int w=getmaxx(viewWin);
        int h=getmaxy(viewWin);

        int cx = (int)((state.x/100)*(w-2));
        int cy = (int)((state.y/100)*(h-2));

        mvwaddch(viewWin,cy+1,cx+1,'+');
        wrefresh(viewWin);

        // -------- DRAW INFO --------
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

