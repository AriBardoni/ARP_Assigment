/*#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "common.h"

int main(int argc, char **argv) {

    if (argc < 3) {
        fprintf(stderr, "drone fds missing\n");
        return 1;
    }

    int fdBtoD = atoi(argv[1]);  
    int fdDtoB = atoi(argv[2]);  

    // Position & velocity
    float x  = 50.0f;
    float y  = 50.0f;
    float vx = 0.0f;
    float vy = 0.0f;

    // Parameters
    float Fx = 0.0f;
    float Fy = 0.0f;
    float M  = 1.0f;
    float K  = 0.15f;      // VERY IMPORTANT: low damping → responsive!
    float T  = 0.02f;      // 50 Hz update

    int running = 1;

    while (running) {

        // Read force from blackboard
        ForceMsg fm;
        ssize_t r = read(fdBtoD, &fm, sizeof(fm));
        if (r == 0) break;

        if (r == sizeof(fm)) {

            if (fm.reset) {
                x = 50.0f; y = 50.0f;
                vx = 0.0f; vy = 0.0f;
            }

            Fx = fm.Fx;
            Fy = fm.Fy;
            M  = fm.M;
            K  = fm.K;
            T  = fm.T;
        }

        // --------------------
        //  DRONE DYNAMICS
        // --------------------

        // acceleration
        float ax = (Fx - K * vx) / M;
        float ay = (Fy - K * vy) / M;

        // semi-implicit Euler (stable)
        vx += ax * T;
        vy += ay * T;

        x  += vx * T;
        y  += vy * T;

        // Limits
        if (x < 0)   { x = 0;   vx = 0; }
        if (x > 100) { x = 100; vx = 0; }
        if (y < 0)   { y = 0;   vy = 0; }
        if (y > 100) { y = 100; vy = 0; }

        // Send state to blackboard
        StateMsg sm = { x, y, vx, vy };
        write(fdDtoB, &sm, sizeof(sm));

        usleep((useconds_t)(T * 1000000.0f));
    }

    close(fdBtoD);
    close(fdDtoB);

    return 0;
}*/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "common.h"

int main(int argc,char **argv){
    if(argc<3){ fprintf(stderr,"drone: missing fds\n"); return 1; }

    int fdBtoD = atoi(argv[1]); 
    int fdDtoB = atoi(argv[2]);

    float x=50,y=50,vx=0,vy=0;
    float Fx=0,Fy=0, M=1, K=0.4f, T=0.02f;

    while(1){
        ForceMsg fm;
        ssize_t r = read(fdBtoD,&fm,sizeof(fm));

        if(r==sizeof(fm)){
            Fx = fm.Fx;
            Fy = fm.Fy;
            if(fm.reset){ x=50;y=50;vx=vy=0; }
        }
        if(r==0) return 0;

        float ax=(Fx - K*vx)/M;
        float ay=(Fy - K*vy)/M;

        vx += ax*T;
        vy += ay*T;
        x  += vx*T;
        y  += vy*T;

        if(x<0)x=0,vx=0;
        if(x>100)x=100,vx=0;
        if(y<0)y=0,vy=0;
        if(y>100)y=100,vy=0;

        StateMsg sm={x,y,vx,vy};
        write(fdDtoB,&sm,sizeof(sm));

        usleep((int)(T*1000000));
    }
}


