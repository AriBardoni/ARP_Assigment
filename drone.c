#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "log.h"
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

        if(strcmp(key, "MASS") == 0)      MASS = val;
        else if(strcmp(key, "DAMPING") == 0)   DAMPING = val;
        else if(strcmp(key, "TIMESTEP") == 0)  TIMESTEP = val;
    }

    fclose(f);
}

int main(int argc,char **argv){
    if(argc<3){ fprintf(stderr,"drone: missing fds\n"); return 1; }

    load_params_drone();

    // Pipes passed by the blackboard
    int fdBtoD = atoi(argv[1]);  // blackboard → drone
    int fdDtoB = atoi(argv[2]);  // drone → blackboard

    // Initial drone state
    float x=50,y=50,vx=0,vy=0;

     if (!log_init("drone.log")) {
        perror("log_init");
        // opzionale: puoi continuare anche senza log
    }

    // Force and physics params
    float Fx=0,Fy=0;

    while(1){

        load_params_drone();
        ForceMsg fm;
        ssize_t r = read(fdBtoD,&fm,sizeof(fm));

        if(r==sizeof(fm)){
            // Receive force from blackboard
            Fx = fm.Fx;
            Fy = fm.Fy;

            if(fm.reset){
                // Reset drone
                x=50; y=50;
                vx=0; vy=0;
            }
        }
        if(r==0) return 0; // pipe closed → exit

        // acceleration = (Force - damping*velocity) / mass
        float ax = (Fx - DAMPING * vx) / MASS;
        float ay = (Fy - DAMPING * vy) / MASS;

        vx += ax * TIMESTEP;
        vy += ay * TIMESTEP;
        x  += vx * TIMESTEP;
        y  += vy * TIMESTEP;

        log_write2("POS", x, y);
        log_write2("VEL", vx, vy);

        // Keep drone in map bounds
        if(x<0)x=0,vx=0;
        if(x>100)x=100,vx=0;
        if(y<0)y=0,vy=0;
        if(y>100)y=100,vy=0;

        // Send updated state back to blackboard
        StateMsg sm={x,y,vx,vy};
        write(fdDtoB,&sm,sizeof(sm));

        usleep((int)(TIMESTEP*100000)); // wait until next step
    }

    log_close();
}