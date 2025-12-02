#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "common.h"

#define N_TARGETS 10
#define W 100
#define H 100

int main(int argc, char **argv){
    if(argc < 2){
        fprintf(stderr,"targets: missing fd\n");
        return 1;
    }

    int fdTtoB = atoi(argv[1]);
    srand(time(NULL) ^ getpid());

    ObjMsg msg;
    msg.type = 'T';

    for(int i = 0; i < N_TARGETS; i++){
        msg.id = i;            // id 0..9
        msg.x = rand() % W;    // world coord
        msg.y = rand() % H;

        write(fdTtoB, &msg, sizeof(msg));
        usleep(10000);
    }

    // static targets
    while(1){
        usleep(500000);
    }

    return 0;
}
