#pragma once
#include <stdint.h>

#define RESPAWN_T 30.0
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
    float wall_damping; // intensity of the wall force 
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

// Process IDs for Watchdog
typedef enum {
    PROCESS_DRONE = 0,
    PROCESS_INPUT,
    PROCESS_BLACKBOARD,
    PROCESS_OBSTACLES,
    PROCESS_TARGETS,
    PROCESS_COUNT
} ProcessID;

// Code Areas for Logging
typedef enum {
    AREA_INIT = 0,
    AREA_COMPUTE,
    AREA_WAIT_INPUT,
    AREA_UPDATE_MAP,
    AREA_SPAWN,
    AREA_SLEEP
} CodeArea;
