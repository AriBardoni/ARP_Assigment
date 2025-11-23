#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "common.h"

int main(int argc,char **argv){
    if(argc<3){ fprintf(stderr,"drone: missing fds\n"); return 1; }

    // Pipes passed by the blackboard
    int fdBtoD = atoi(argv[1]);  // blackboard → drone
    int fdDtoB = atoi(argv[2]);  // drone → blackboard

    // Initial drone state
    float x=50,y=50,vx=0,vy=0;

    // Force and physics params
    float Fx=0,Fy=0;
    float M=1;
    float K=0.4f;      // damping
    float T=0.02f;     // update at 50Hz

    while(1){
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
        float ax=(Fx - K*vx)/M;
        float ay=(Fy - K*vy)/M;

        // Euler integration
        vx += ax*T;
        vy += ay*T;
        x  += vx*T;
        y  += vy*T;

        // Keep drone in map bounds
        if(x<0)x=0,vx=0;
        if(x>100)x=100,vx=0;
        if(y<0)y=0,vy=0;
        if(y>100)y=100,vy=0;

        // Send updated state back to blackboard
        StateMsg sm={x,y,vx,vy};
        write(fdDtoB,&sm,sizeof(sm));

        usleep((int)(T*1000000)); // wait until next step
    }
}
