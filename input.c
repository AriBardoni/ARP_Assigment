#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include "common.h"

int main(int argc, char **argv){

    if(argc < 2){
        fprintf(stderr,"input: missing fd\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);

    // -----------------------------
    //  FORZA USO DI /dev/tty
    // -----------------------------
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

    // -----------------------------
    //  CONFIGURAZIONE CURSES
    // -----------------------------
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(50);

    printw("=== INPUT CONTROLLER ===\n");
    printw("i/j/k/l = movimento\n");
    printw("b = brake | r = reset | q = quit\n");
    refresh();

    // -----------------------------
    //     LOOP DI INPUT
    // -----------------------------
    while(1){
        int c = getch();
        if(c == ERR) continue;

        KeyMsg km = (KeyMsg){0,0,0};

        switch(c){
            case 'q': km.cmd = 9; break;
            case 'i': km.dFy = -1; break;
            case 'k': km.dFy = +1; break;
            case 'j': km.dFx = -1; break;
            case 'l': km.dFx = +1; break;
            case 'b': km.cmd = 1; break;
            case 'r': km.cmd = 2; break;
            default: continue;
        }

        write(fdItoB, &km, sizeof(km));

        if(km.cmd == 9)
            break;
    }

    // -----------------------------
    //  EXIT PULITA
    // -----------------------------
    endwin();
    delscreen(scr);
    fclose(term_in);
    fclose(term_out);

    return 0;
}
