/*#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "common.h"

#define N_OBS 10

int main(int argc, char **argv){
    if(argc < 3){
        fprintf(stderr,"obstacles: missing fd\n");
        return 1;
    }

    int fdOtoB = atoi(argv[1]);   // pipe to blackboard 
    int fdW = atoi(argv[2]);      // watchdog pipe
    srand(time(NULL) ^ getpid());

    int w = 100, h = 100; 
    ObjMsg msg = {.type = 'X'};

    for(int i = 0; i < N_OBS; i++){
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
        // for static targets 
        usleep(500000);

        // Watchdog signal
        counter++;
        if (counter >= 2) { // 2 * 0.5s = 1s
            int pid = PROCESS_OBSTACLES;
            write(fdW, &pid, sizeof(int));
            counter = 0;
        }
    }
}*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "common.h"

#define N_OBS 10
#define WORLD_W 100
#define WORLD_H 100

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv){
    if(argc < 3){
        fprintf(stderr,"obstacles: missing fd\n");
        return 1;
    }

    int fdOtoB = atoi(argv[1]);   // pipe to blackboard
    int fdW    = atoi(argv[2]);   // watchdog pipe

    srand(time(NULL) ^ getpid());

    ObjMsg msg = {.type = 'X'};
    double last_spawn = 0.0;
    int counter = 0;

    while(1){

        double t = now_sec();

        // respawn every 20 seconds
        if (t - last_spawn >= RESPAWN_T) {

            for(int i = 0; i < N_OBS; i++){
                msg.id = i;
                msg.x  = rand() % WORLD_W;
                msg.y  = rand() % WORLD_H;
                write(fdOtoB, &msg, sizeof(msg));
            }

            last_spawn = t;
        }

        // watchdog every 1 second
        usleep(20000); // 20 ms
        counter++;
        if (counter >= 50) {
            int pid = PROCESS_OBSTACLES;
            write(fdW, &pid, sizeof(int));
            counter = 0;
        }
    }
}
