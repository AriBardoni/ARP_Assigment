#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "common.h"

#define N_TARGETS 10

int main(int argc, char **argv){
    if(argc < 3){
        fprintf(stderr,"obstacles: missing fd\n");
        return 1;
    }

    int fdOtoB = atoi(argv[1]);   // pipe to blackboard 
    int fdW = atoi(argv[2]);      // watchdog pipe
    srand(time(NULL) ^ getpid());

    int w = 100, h = 100; 
    ObjMsg msg = {.type = '=0'};

    for(int i = 0; i < N_TARGETS; i++){
        int valid = 0;
        while(!valid){
            msg.x = rand() % w;
            msg.y = rand() % h;
            msg.id = i;
            write(fdOtoB, &msg, sizeof(msg));
            valid = 1;
        }
        usleep(10000);
    }

    int counter = 0;
    while(1){
        // for static obstacles 
        usleep(500000);

        // Watchdog signal
        counter++;
        if (counter >= 2) { // 2 * 0.5s = 1s
            int pid = PROCESS_TARGETS;
            write(fdW, &pid, sizeof(int));
            counter = 0;
        }
    }
}