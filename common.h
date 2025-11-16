#pragma once
#include <stdint.h>

// structure of the messages sent on the pipe

// input -> blackboard
typedef struct{
    float dFx;      // difference of force applied to the drone in the directions
    float dFy;
    int cmd;        // brake, reset, quit
} KeyMsg;

// blackboard -> drone
typedef struct{
    float Fx, Fy;       // resulting force on the drone
    float M,K,T;        // dynamic parameters 
    int reset;          // drone reset
} ForceMsg;

// drone -> blackboard 
typedef struct{
    float x,y;          // position
    float vx, vy;       // velocity 
} StateMsg;


