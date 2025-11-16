/*#define _XOPEN_SOURCE_EXTENDED
#include <locale.h>
#include <ncurses.h>
#include <unistd.h>
#include <sys/select.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include "common.h"

# define Hmap 100
# define Lmap 100
static WINDOW *frameWin=NULL,*viewWin=NULL,*infoWin=NULL;

static void restore_terminal(int sig){
    (void)sig; // evita warning unused parameter
    endwin();
    system("stty sane");
    _exit(0);
}

static void layout_and_draw(WINDOW *win,int *outW,int *outH){
    int H,W; getmaxyx(stdscr,H,W);
    int wh=(H>6)?H-6:H;
    int ww=(W>10)?W-10:W;

    if(wh<3) wh=3;
    if(ww<3) ww=3;

    wresize(win,wh,ww);
    mvwin(win,(H-wh)/2,(W-ww)/2);
    werase(stdscr); 
    werase(win);
    box(win,0,0);
    mvprintw(0,0,"Resize the shell; press 'q' to quit.");
    mvwprintw(win,1,2,"stdscr: %dx%d  frame: %dx%d",H,W,wh,ww);
    refresh(); wrefresh(win);

    if(outW){ *outW=ww; }
    if(outH){ *outH=wh; }
}

typedef struct{float Wm,Hm;int Wc,Hc;}Map;
static Map gmap={Hmap,Lmap,0,0};

static void world_to_screen(float x,float y,int *cx,int *cy){
    if(gmap.Wc<=1||gmap.Hc<=1){*cx=*cy=0;return;}
    float sx=(x/gmap.Wm)*(gmap.Wc-1);
    float sy=(y/gmap.Hm)*(gmap.Hc-1);
    if(sx<0)sx=0;else if(sx>gmap.Wc-1)sx=gmap.Wc-1;
    if(sy<0)sy=0;else if(sy>gmap.Hc-1)sy=gmap.Hc-1;
    *cx=(int)(sx+0.5f);
    *cy=(int)(sy+0.5f);
}

static void setup_panes(int frame_w,int frame_h){
    int inner_w=frame_w-2, inner_h=frame_h-2;
    if(inner_w<10) inner_w=10;
    if(inner_h<5)  inner_h=5;
    int panel_w=inner_w/3;
    if(panel_w<24) panel_w=(inner_w>28)?24:inner_w/2;
    int view_w=inner_w-panel_w-1;
    if(view_w<8) view_w=8;
    int fy,fx; getbegyx(frameWin,fy,fx);
    if(viewWin){ delwin(viewWin); viewWin=NULL; }
    if(infoWin){ delwin(infoWin); infoWin=NULL; }
    viewWin=newwin(inner_h,view_w,fy+1,fx+1);
    infoWin=newwin(inner_h,panel_w,fy+1,fx+1+view_w+1);
    werase(frameWin); box(frameWin,0,0);
    mvwvline(frameWin,1,1+view_w,ACS_VLINE,inner_h);
    wrefresh(frameWin);
    int vw_h,vw_w; getmaxyx(viewWin,vw_h,vw_w);
    gmap.Wc=vw_w; gmap.Hc=vw_h;
}

int main(int argc,char **argv){
    if(argc<4){fprintf(stderr,"blackboard fds missing\n");return 1;}
    int fdItoB=atoi(argv[1]);
    int fdBtoD=atoi(argv[2]);
    int fdDtoB=atoi(argv[3]);
    signal(SIGINT,restore_terminal);
    signal(SIGTERM,restore_terminal);

    setlocale(LC_ALL,"");
    initscr(); 
    cbreak(); 
    noecho();
    keypad(stdscr,TRUE);
    nodelay(stdscr,TRUE);

    frameWin=newwin(3,3,0,0);
    int fw=0,fh=0;
    layout_and_draw(frameWin,&fw,&fh);
    setup_panes(fw,fh);

    float Fx=0,Fy=0,M=1.f,K=1.f,T=0.05f;
    StateMsg state={50.f,50.f,0.f,0.f};
    const int tick_ms=20;

    int running=1;

    while(running){

        fd_set rfds; 
        FD_ZERO(&rfds);
        FD_SET(fdItoB,&rfds);
        FD_SET(fdDtoB,&rfds);

        int fdmax=(fdItoB>fdDtoB?fdItoB:fdDtoB);
        struct timeval tv={0,tick_ms*1000};

        // checks among the ready processes that want to transmitt and select one 
        // tv=timeout 
        // rfds checks for file ready to be ridden 
        // NULL because doesn't check for files ready to write or exceptional fd

        int rv=select(fdmax+1,&rfds,NULL,NULL,&tv);

        if(rv>0){

            // case I->B
            if(FD_ISSET(fdItoB,&rfds)){
                KeyMsg km; 
                ssize_t r=read(fdItoB,&km,sizeof(km));
                if(r==0) break;
                
                int sx = (int)(state.x + 0.5f);
                int sy = (int)(state.y + 0.5f);

                if(r==sizeof(km)){
                    // 9==quit 
                    if(km.cmd==9){
                        running=0;
                    }
                    // 2==reset 
                    else if(km.cmd==2){
                        Fx=Fy=0;
                        ForceMsg fm={Fx,Fy,M,K,T,1};
                        write(fdBtoD,&fm,sizeof(fm));
                    }
                    // 1==brake 
                    else if(km.cmd==1){
                        Fx=Fy=0;
                    }

                    else{
                        /*if(sx==Hmap-1 && km.dFx>0) continue;
                        else if(sx==-Hmap-1 && km.dFx<0) {
                            mvwprintw(infoWin,11,0,"input non valido");
                            wrefresh(infoWin);
                            continue;
                        }
                        else if(sy==Lmap-1 && km.dFy>0) continue;
                        else if(sy==-Lmap-1 && km.dFy<0) continue;
                        else {
                            Fx+=km.dFx; 
                            Fy+=km.dFy;
                        }
                    }
                }
            }

            // case D->B
            if (FD_ISSET(fdDtoB,&rfds)){
                StateMsg sm; 
                ssize_t r=read(fdDtoB,&sm,sizeof(sm));
                if(r==0) break;
                if(r==sizeof(sm)) state=sm;
            }
        }

        ForceMsg fm={Fx,Fy,M,K,T,0};
        write(fdBtoD,&fm,sizeof(fm));

        int ch=getch();
        if(ch==KEY_RESIZE){
            resize_term(0,0);
            layout_and_draw(frameWin,&fw,&fh);
            setup_panes(fw,fh);
        }else if(ch=='q'){
            running=0;
        }

        // window for infos 
        werase(viewWin);
        int cx,cy; 
        world_to_screen(state.x,state.y,&cx,&cy);

        if(cy>=0&&cy<gmap.Hc&&cx>=0&&cx<gmap.Wc)
            mvwaddch(viewWin,cy,cx,'+');
        wrefresh(viewWin);

        // infos 
        werase(infoWin);
        //mvwprintw(infoWin,0,0,"[INSPECTION PANEL]");
        mvwprintw(infoWin,2,0,"Frame: %dx%d",fw,fh);
        mvwprintw(infoWin,4,0,"Force F=(%.2f,%.2f)",Fx,Fy);
        mvwprintw(infoWin,5,0,"Mass M=%.2f  K=%.2f  T=%.0f ms",M,K,T*1000);
        mvwprintw(infoWin,7,0,"Pos (x,y)=(%.1f,%.1f)",state.x,state.y);
        mvwprintw(infoWin,8,0,"Vel (vx,vy)=(%.2f,%.2f)",state.vx,state.vy);
        mvwprintw(infoWin,10,0,"Keys: i/j/k/l,u/o/n/,  b=brake, r=reset, q=quit");
        wrefresh(infoWin);
    }

    // quitting 
    close(fdItoB);
    close(fdBtoD);
    close(fdDtoB);

    if(viewWin) delwin(viewWin);
    if(infoWin) delwin(infoWin);
    if(frameWin) delwin(frameWin);

    endwin();
    system("stty sane");
    return 0;
}
*/

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


