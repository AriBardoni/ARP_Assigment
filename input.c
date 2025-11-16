/*#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "common.h"

static void set_raw(int fd, struct termios *orig){
    struct termios t;
    if (tcgetattr(fd, orig) < 0) {
        perror("tcgetattr");
        exit(1);
    }
    t = *orig;
    cfmakeraw(&t);
    t.c_cc[VTIME] = 0;
    t.c_cc[VMIN]  = 0;
    if (tcsetattr(fd, TCSANOW, &t) < 0) {
        perror("tcsetattr");
        exit(1);
    }
}

int main(int argc, char **argv){
    if (argc < 2){
        fprintf(stderr, "input fd missing\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);

    // Leggiamo direttamente dal terminale (Konsole)
    int tty = open("/dev/tty", O_RDONLY | O_NONBLOCK);
    if (tty < 0){
        perror("open /dev/tty");
        return 1;
    }

    struct termios orig;
    set_raw(tty, &orig);

    const float step = 1.0f;
    char c;
    KeyMsg km;

    while (1){
        ssize_t r = read(tty, &c, 1);
        if (r == 1){
            memset(&km, 0, sizeof(km));

            switch (c){
                // movimenti singoli
                case 'i': km.dFy = -step; break; // su
                case 'k': km.dFy =  step; break; // giù
                case 'j': km.dFx = -step; break; // sinistra
                case 'l': km.dFx =  step; break; // destra

                // diagonali
                case 'u': km.dFx = -step; km.dFy = -step; break; // su-sx
                case 'o': km.dFx =  step; km.dFy = -step; break; // su-dx
                case 'n': km.dFx = -step; km.dFy =  step; break; // giù-sx
                case ',': km.dFx =  step; km.dFy =  step; break; // giù-dx

                // comandi speciali
                case 'b': // brake
                    km.cmd = 1;
                    break;
                case 'r': // reset
                    km.cmd = 2;
                    break;
                case 'q': // quit
                    km.cmd = 9;
                    write(fdItoB, &km, sizeof(km));
                    close(fdItoB);
                    tcsetattr(tty, TCSANOW, &orig);
                    close(tty);
                    return 0;

                default:
                    break;
            }

            if (km.cmd || km.dFx != 0.0f || km.dFy != 0.0f){
                write(fdItoB, &km, sizeof(km));
            }
        }

        usleep(10000); // 10 ms
    }
}*/

/*#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <termios.h>
#include "common.h"

static void set_raw(){
    struct termios t;
    tcgetattr(STDIN_FILENO,&t);
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN]  = 1;
    t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO,TCSANOW,&t);
}

int main(int argc, char **argv){

    initscr();
    cbreak();
    noecho();
    keypad(stdscr,TRUE);

    if(argc<2){
        fprintf(stderr,"input: missing fd\n");
        return 1;
    }

    int fdItoB = atoi(argv[1]);
    set_raw();

    printf("INPUT TAB ATTIVO (q per uscire)\n");

    while(1){
        char c;
        c = getch();

        KeyMsg km = {0,0,0};

        if(c=='q') km.cmd=9;
        else if(c=='i') { km.dFy=-1; }
        else if(c=='k') { km.dFy=+1; }
        else if(c=='j') { km.dFx=-1; }
        else if(c=='l') { km.dFx=+1; }
        else if(c=='b') km.cmd=1;
        else if(c=='r') km.cmd=2;
        else continue;

        write(fdItoB, &km, sizeof(km));

        if(km.cmd == 9) break;
    }

    return 0;
}*/

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

    // -------------------------
    // INIZIALIZZAZIONE NCURSES
    // -------------------------
    setenv("TERM", "xterm-256color", 1);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);

    // getch() NON blocca, ma controlla la tastiera ogni 50 ms
    timeout(50);

    printw("=== INPUT TAB ATTIVO ===\n");
    printw("Usa i/j/k/l per muovere\n");
    printw("q per uscire\n");
    refresh();

    while(1){
        int c = getch();

        if(c == ERR){
            // nessun tasto premuto
            continue;
        }

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

        ssize_t wr = write(fdItoB, &km, sizeof(km));
        if(wr < 0){
            endwin();
            perror("write to pipe");
            return 1;
        }

        if(km.cmd == 9) break;
    }

    endwin();
    return 0;
}





