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


