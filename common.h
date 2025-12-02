#pragma once
#include <stdint.h>

// Messages passed through pipes between processes

// From input -> blackboard
typedef struct{
    float dFx;   // change in force along X
    float dFy;   // change in force along Y
    int cmd;     // 1=brake, 2=reset, 9=quit
} KeyMsg;

// From blackboard -> drone
typedef struct{
    float Fx, Fy;   // total force applied to drone
    float M,K,T;    // mass, damping, timestep
    int reset;      // flag to reset drone state
    int intensity;  // intensity of the force field
} ForceMsg;

// From drone -> blackboard
typedef struct{
    float x,y;      // drone position
    float vx, vy;   // drone velocity
} StateMsg;

// From obstacles/targets -> blackboard
typedef struct{
    char type;  //O=obstacles, T=targets 
    int id;
    float x,y;
}ObjMsg;
