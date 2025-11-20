#pragma once
#include <stdint.h>

// Messages passed through pipes between processes

// From input → blackboard
typedef struct{
    float dFx;   // change in force along X
    float dFy;   // change in force along Y
    int cmd;     // 1=brake, 2=reset, 9=quit
} KeyMsg;

// From blackboard → drone
typedef struct{
    float Fx, Fy;   // total force applied to drone
    float M,K,T;    // mass, damping, timestep
    int reset;      // flag to reset drone state
} ForceMsg;

// From drone → blackboard
typedef struct{
    float x,y;      // drone position
    float vx, vy;   // drone velocity
} StateMsg;
