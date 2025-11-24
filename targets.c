#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "common.h"

#define N_TARGETS 10

int main(int argc, char **argv){
    if(argc < 2){
        fprintf(stderr,"obstacles: missing fd\n");
        return 1;
    }

    int fdOtoB = atoi(argv[1]);   // pipe to blackboard 
    srand(time(NULL) ^ getpid());

    int w = 100, h = 100; 
    ObjMsg msg = {.type = 'T'};

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

    while(1){
        // for static obstacles 
        usleep(500000);
    }
}
