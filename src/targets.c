#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include "logger.h"
#include "common.h"

#define N_TARGETS 10
#define WORLD_W 100
#define WORLD_H 100 

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv){
    if(argc < 3){
        logger_init("../logs");
        log_system("TARGETS", "missing fd");
        return 1;
    }

    logger_init("../logs");
    log_process_register("TARGETS", getpid());
    log_system("TARGETS", "started");

    int fdTtoB = atoi(argv[1]);   // pipe to blackboard
    pid_t wd_pid = (pid_t)atoi(argv[2]);   // watchdog pid

    srand(time(NULL) ^ (getpid() << 1));
    double last_spawn = now_sec();

    ObjMsg msg = {.type = 'T'};
    int counter = 0;

    // Log startup
    log_message("LogFile2", "Targets", "Process started");

    while(1){

        double t = now_sec();

        // respawn every 20 seconds
        if (t - last_spawn >= RESPAWN_T) {

            log_system("TARGETS", "respawning targets");

            for(int i = 0; i < N_TARGETS; i++){
                msg.id = i;
                msg.x  = rand() % WORLD_W;
                msg.y  = rand() % WORLD_H;
                write(fdTtoB, &msg, sizeof(msg));
            }

            last_spawn = t;
        }

        // watchdog every 1 second
        usleep(20000); // 20 ms
        counter++;
        if (counter >= 50) {
            union sigval value;
            value.sival_int = PROCESS_TARGETS | (AREA_SPAWN << 8);
            if (wd_pid > 0) {
                sigqueue(wd_pid, SIGUSR1, value);
            }
            counter = 0;
        }
    }
}
