#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include "logger.h"
#include "common.h"

float MASS = 1.0f;
float DAMPING = 1.0f;
float TIMESTEP = 0.1f;

void load_params_drone() {
    FILE *f = fopen("params.txt", "r");
    if(!f) return;

    char key[64];
    float val;

    while (fscanf(f, "%63[^=]=%f\n", key, &val) == 2) {

        if(strcmp(key, "MASS") == 0)           MASS = val;
        else if(strcmp(key, "DAMPING") == 0)   DAMPING = val;
        else if(strcmp(key, "TIMESTEP") == 0)  TIMESTEP = val;
    }

    fclose(f);
}

int main(int argc,char **argv){
    if(argc < 3){
        logger_init("../logs");
        log_system("DRONE", "missing fds");
        return 1;
    }

    logger_init("../logs");
    log_process_register("DRONE", getpid());
    log_system("DRONE", "started");

    load_params_drone();

    // Pipes passed by the blackboard
    int fdBtoD = atoi(argv[1]);  // blackboard → drone
    int fdDtoB = atoi(argv[2]);  // drone → blackboard
    pid_t wd_pid = (pid_t)atoi(argv[3]);     // watchdog pid

    // Initial drone state
    float x = 20, y = 15, vx = 0, vy = 0;

    float Fx = 0, Fy = 0;
    int counter = 0;

    while(1){

        load_params_drone();
        ForceMsg fm;
        ssize_t r = read(fdBtoD, &fm, sizeof(fm));

        if(r == sizeof(fm)){
            Fx = fm.Fx;
            Fy = fm.Fy;

            if(fm.reset){
                log_system("DRONE", "reset received");
                x = 20; y = 15;
                vx = 0; vy = 0;
            }
        }

        if(r == 0){
            log_system("DRONE", "pipe closed, exiting");
            return 0;
        }

        // Physics
        float ax = (Fx - DAMPING * vx) / MASS;
        float ay = (Fy - DAMPING * vy) / MASS;

        vx += ax * TIMESTEP;
        vy += ay * TIMESTEP;
        x  += vx * TIMESTEP;
        y  += vy * TIMESTEP;

        if (fabsf(vx) < 1e-3f) vx = 0.0f;
        if (fabsf(vy) < 1e-3f) vy = 0.0f;

        // Boundaries
        if(x < 0)   x = 0,   vx = 0;
        if(x > 100) x = 100, vx = 0;
        if(y < 0)   y = 0,   vy = 0;
        if(y > 100) y = 100, vy = 0;

        // Send updated state
        StateMsg sm = { x, y, vx, vy };
        write(fdDtoB, &sm, sizeof(sm));

        // Watchdog
        counter++;
        if (counter >= 10) {
            union sigval value;
            value.sival_int = PROCESS_DRONE | (AREA_COMPUTE << 8);
            if (wd_pid > 0) {
                sigqueue(wd_pid, SIGUSR1, value);
            }
            counter = 0;
        }

        usleep((int)(TIMESTEP * 100000));
    }
}
