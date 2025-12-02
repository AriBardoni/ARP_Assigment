#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "common.h"
#include "FORCE_FIELD.h"

#define N_OBS 10
#define ETHA 0.1
#define RHO 5

// Helper function to calculate the magnitude of a 2D vector
double magnitude(Vector2D v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

// Helper function to calculate the 2D cross product magnitude (R x F_drone)
// This scalar determines the rotational tendency (torque) which is used for steering direction.
double cross_product_2d_magnitude(Vector2D r, Vector2D f) {
    return r.x * f.y - r.y * f.x;
}

double calculate_net_force(
    Vector2D obstacle_pos, 
    Vector2D drone_pos, 
    Vector2D drone_force, 
    Vector2D *net_force
) {
    // 1. Calculate Repulsion Vector (R): Obstacle to Drone
    Vector2D R = {
        drone_pos.x - obstacle_pos.x,
        drone_pos.y - obstacle_pos.y
    };

    double distance = magnitude(R);
    
    // Check for collision (or near-zero distance) to prevent division by zero
    // In a real simulation, this condition would trigger a collision response.
    if (distance < 0.1) {
        printf("WARNING: Drone too close to obstacle. Applying emergency brake.\n");
        net_force->x = 0.0;
        net_force->y = 0.0;
        return 0.0;
    }

    // 2. Calculate 2D Cross Product Magnitude (C)
    // C tells us which side the drone is approaching relative to the obstacle.
    double C = cross_product_2d_magnitude(R, drone_force);

    // 3. Calculate Tangential Vector (T)
    // T is perpendicular to R. T = (-Ry, Rx) gives a 90-degree counter-clockwise rotation.
    Vector2D T = { -R.y, R.x };
    double tangent_mag = magnitude(T);
    
    // Normalize T to get the unit tangential direction
    Vector2D T_unit = { T.x / tangent_mag, T.y / tangent_mag };

    // 4. Calculate Steering Force Magnitude (M)
    // The steering force scales with the predefined intensity and decays with distance.
    // Inverse distance squared + 1.0 (for numerical stability) is a common pattern.
    double steering_magnitude = K_REPULSION_INTENSITY / (distance * distance + 1.0);

    // 5. Determine Steering Direction Sign (S)
    // If C > 0, the drone is approaching from one side, so the steering force
    // needs to push it in a specific tangential direction (T_unit or -T_unit).
    double sign_C = (C > 0) ? 1.0 : (C < 0 ? -1.0 : 0.0);
    
    // In path planning, the sign usually dictates whether to steer left or right 
    // around the obstacle. We use sign_C to modulate the steering.
    
    // 6. Calculate Steering Force Vector (F_steer)
    // F_steer is the steering magnitude multiplied by the direction (T_unit or -T_unit).
    Vector2D F_steer = {
        steering_magnitude * sign_C * T_unit.x,
        steering_magnitude * sign_C * T_unit.y
    };

    // 7. Calculate Net Force (F_net)
    // F_net = F_drone + F_steer
    net_force->x = drone_force.x + F_steer.x;
    net_force->y = drone_force.y + F_steer.y;

    // 8. Return Intensity (Magnitude) of the Net Force
    return magnitude(*net_force);
}

#define W 100
#define H 100

int main(int argc, char **argv){
    if(argc < 2){
        fprintf(stderr,"obstacles: missing fd\n");
        return 1;
    }

    int fdOtoB = atoi(argv[1]);
    srand(time(NULL) ^ getpid());

    ObjMsg msg;
    msg.type = 'O';

    for(int i = 0; i < N_OBS; i++){
        msg.id = i;              // id must be 0..9
        msg.x = rand() % W;      // world coords 0..100
        msg.y = rand() % H;

        write(fdOtoB, &msg, sizeof(msg));
        usleep(10000);  
    }
}