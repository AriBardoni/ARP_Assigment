#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include "logger.h"
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

    logger_init("../logs");
    log_process_register("OBSTACLES", getpid());
    log_system("OBSTACLES", "started");

    int fdOtoB = atoi(argv[1]);   // pipe to blackboard
    pid_t wd_pid = (pid_t)atoi(argv[2]);   // watchdog pid

    srand(time(NULL) ^ getpid());
    double last_spawn = now_sec();

    ObjMsg msg = {.type = 'X'};
    int counter = 0;

    // Log startup
    log_message("LogFile2", "Obstacles", "Process started");

    while(1){

        double t = now_sec();

        // respawn every 20 seconds
        if (t - last_spawn >= RESPAWN_T) {

            log_system("OBSTACLES", "respawning obstacles");

            for(int i = 0; i < N_OBS; i++){
                msg.id = i;
                msg.x  = rand() % WORLD_W;
                msg.y  = rand() % WORLD_H;
                write(fdOtoB, &msg, sizeof(msg));
            }

            last_spawn = t;
        }

        // watchdog every 1 second
        usleep(100000); // 100 ms
        counter++;
        if (counter >= 10) { // 100ms * 10 = 1s
            union sigval value;
            value.sival_int = PROCESS_OBSTACLES | (AREA_SPAWN << 8);
            if (wd_pid > 0) {
                sigqueue(wd_pid, SIGUSR1, value);
            }
            counter = 0;
        }
    }
}
