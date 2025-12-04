#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include "log.h"
#include "common.h"

int main(int argc, char **argv){

    if(argc < 2){
        fprintf(stderr,"input: missing fd\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);  // pipe to blackboard
    if (!log_init("input.log")) {
        perror("log_init input");
        // puoi comunque continuare senza log se vuoi
    }

    // Open /dev/tty to use ncurses in this terminal
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

    // Setup ncurses mode
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(50); // getch waits max 50ms

    printw("=== INPUT CONTROLLER ===\n");
    printw("i/j/k/l/u/o/n/, = movement\n");
    printw("b = brake | r = reset | q = quit\n");
    refresh();

        while(1){

        int c = getch();     // check keyboard
        if(c == ERR) continue;

        KeyMsg km = (KeyMsg){0,0,0};
        int step = 1;   // force increment

        // Map keys to force increments
        switch(c){
            case 'q': km.cmd = 9; break;

            case 'i': km.dFy = -step; break;   // up
            case 'k': km.dFy = +step; break;   // down
            case 'j': km.dFx = -step; break;   // left
            case 'l': km.dFx = +step; break;   // right

            // diagonals
            case 'u': km.dFx = -step; km.dFy = -step; break;
            case 'o': km.dFx = +step; km.dFy = -step; break;
            case 'n': km.dFx = -step; km.dFy = +step; break;
            case ',': km.dFx = +step; km.dFy = +step; break;

            case 'b': km.cmd = 1; break;
            case 'r': km.cmd = 2; break;

            default: continue;
        }

        // >>> LOG: registra il tasto premuto e il delta forza
        char buf[64];
        snprintf(buf, sizeof(buf), "KEY '%c' cmd=%d", c, km.cmd);
        log_write2(buf, km.dFx, km.dFy);
        // <<<

        // Send message to blackboard
        write(fdItoB, &km, sizeof(km));
        // fsync(fdItoB);  // puoi TOGLIERLO: su una pipe non serve a nulla

        if(km.cmd == 9)   // quit
            break;
    }

    endwin();
    delscreen(scr);
    fclose(term_in);
    fclose(term_out);

    return 0;
}