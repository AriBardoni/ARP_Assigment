#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include "log.h"
#include "common.h"

static WINDOW *infoWin;

int main(int argc, char **argv){

    if(argc < 2){
        fprintf(stderr,"input: missing fd\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);  // pipe to blackboard
    if (!log_init("input.log")) {
        perror("log_init input");
    }

    // open /dev/tty to use ncurses in this terminal
    FILE *term_in  = fopen("/dev/tty", "r");
    FILE *term_out = fopen("/dev/tty", "w");

    if(!term_in || !term_out){
        perror("fopen /dev/tty");
        return 1;
    }

    SCREEN *scr = newterm(NULL, term_out, term_in);
    if(!scr){
        fprintf(stderr, "newterm() failed\n");
        return 1;
    }

    set_term(scr);

    // setup ncurses mode
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(50);

    // print controller instructions
    printw("=== INPUT CONTROLLER ===\n");
    printw("i/j/k/l/u/o/n/, = movement\n");
    printw("b = brake | r = reset | q = quit\n");
    refresh();

    // create the info window at the bottom
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    infoWin = newwin(5, cols, rows - 5, 0);
    box(infoWin, 0, 0);
    mvwprintw(infoWin, 0, 2, " INPUT DEBUG INFO ");
    wrefresh(infoWin);

    while(1){

        int c = getch();
        if(c == ERR) continue;

        KeyMsg km = (KeyMsg){0,0,0};
        int step = 1;

        // map keys to force increments
        switch(c){
            case 'q': km.cmd = 9; break;

            case 'i': km.dFy = -step; break;
            case 'k': km.dFy = +step; break;
            case 'j': km.dFx = -step; break;
            case 'l': km.dFx = +step; break;

            // diagonals
            case 'u': km.dFx = -step; km.dFy = -step; break;
            case 'o': km.dFx = +step; km.dFy = -step; break;
            case 'n': km.dFx = -step; km.dFy = +step; break;
            case ',': km.dFx = +step; km.dFy = +step; break;

            case 'b': km.cmd = 1; break;
            case 'r': km.cmd = 2; break;

            default: continue;
        }

        // log key
        char buf[64];
        snprintf(buf, sizeof(buf), "KEY '%c' cmd=%d", c, km.cmd);
        log_write2(buf, km.dFx, km.dFy);

        // send to blackboard
        write(fdItoB, &km, sizeof(km));

        // update info window 
        werase(infoWin);
        box(infoWin, 0, 0);
        mvwprintw(infoWin, 0, 2, " INPUT DEBUG INFO ");

        mvwprintw(infoWin, 1, 2, "Key pressed: %c", (c >= 32 && c < 127) ? c : '?');
        mvwprintw(infoWin, 2, 2, "Fx = %d   Fy = %d", km.dFx, km.dFy);

        if(km.cmd == 1) mvwprintw(infoWin, 3, 2, "Command: BRAKE");
        else if(km.cmd == 2) mvwprintw(infoWin, 3, 2, "Command: RESET");
        else if(km.cmd == 9) mvwprintw(infoWin, 3, 2, "Command: QUIT");
        else mvwprintw(infoWin, 3, 2, "Command: FORCE");

        wrefresh(infoWin);

        if(km.cmd == 9)
            break;
    }

    endwin();
    delscreen(scr);
    fclose(term_in);
    fclose(term_out);

    return 0;
}
