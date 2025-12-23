#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <signal.h>
#include "log.h"
#include "common.h"

static WINDOW *infoWin;

int main(int argc, char **argv){

    if(argc < 3){
        fprintf(stderr,"input: missing fd\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);
    pid_t wd_pid = (pid_t)atoi(argv[2]);
    log_init("input.log");

    FILE *term_in  = fopen("/dev/tty", "r");
    FILE *term_out = fopen("/dev/tty", "w");

    SCREEN *scr = newterm(NULL, term_out, term_in);
    set_term(scr);

    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(50);

    printw("=== INPUT CONTROLLER ===\n");
    printw("i/j/k/l/u/o/n/, = movement\n");
    printw("b = brake | r = reset | q = quit\n");
    refresh();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    infoWin = newwin(5, cols, rows - 5, 0);
    box(infoWin, 0, 0);
    wrefresh(infoWin);

    KeyMsg km = {0,0,0};
    KeyMsg last_km = {0,0,0};
    const char *dir = "NONE";
    const char *last_dir = "NONE";
    int counter = 0;

    while(1){

        int c = getch();
        int step = 1;

        if(c != ERR){

            km = (KeyMsg){0,0,0};
            dir = "NONE";

            switch(c){
                case 'q': km.cmd = 9; dir = "QUIT"; break;

                case 'i': km.dFy = -step; dir = "UP"; break;
                case 'k': km.dFy = +step; dir = "DOWN"; break;
                case 'j': km.dFx = -step; dir = "LEFT"; break;
                case 'l': km.dFx = +step; dir = "RIGHT"; break;

                case 'u': km.dFx = -step; km.dFy = -step; dir = "UP-LEFT"; break;
                case 'o': km.dFx = +step; km.dFy = -step; dir = "UP-RIGHT"; break;
                case 'n': km.dFx = -step; km.dFy = +step; dir = "DOWN-LEFT"; break;
                case ',': km.dFx = +step; km.dFy = +step; dir = "DOWN-RIGHT"; break;

                case 'b': km.cmd = 1; dir = "BRAKE"; break;
                case 'r': km.cmd = 2; dir = "RESET"; break;

                default: break;
            }

            last_km = km;
            last_dir = dir;

            write(fdItoB, &km, sizeof(km));

            if(km.cmd == 9)
                break;
        }
        else {
            km.dFx = 0;
            km.dFy = 0;
            write(fdItoB, &km, sizeof(km));
        }

        werase(infoWin);
        box(infoWin, 0, 0);
        mvwprintw(infoWin, 0, 2, " INPUT CONTROLS ");

        mvwprintw(infoWin, 1, 2, "i/k/j/l : up/down/left/right");
        mvwprintw(infoWin, 2, 2, "u/o/n/, : diagonals");

        mvwprintw(infoWin, 3, 2, "b=brake  r=reset  q=quit");
        mvwprintw(infoWin, 3, 35,
                  "Dir=%-10s Fx=%+5.2f Fy=%+5.2f",
                  last_dir, last_km.dFx, last_km.dFy);

        wrefresh(infoWin);

        // Watchdog signal
        counter++;
        if (counter >= 20) { // 20 * 50ms = 1s
            union sigval value;
            value.sival_int = PROCESS_INPUT | (AREA_WAIT_INPUT << 8);
            sigqueue(wd_pid, SIGUSR1, value);
            counter = 0;
        }
    }

    endwin();
    delscreen(scr);
    fclose(term_in);
    fclose(term_out);

    return 0;
}
