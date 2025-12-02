#ifndef FORCE_FIELD_H
#define FORCE_FIELD_H

#include <math.h>

// The intensity of the repulsive force field. 
// As requested, this variable has no computation related to its definition.
#define K_REPULSION_INTENSITY 50.0 

/**
 * @brief Structure to represent a 2D vector (position or force).
 */
typedef struct {
    double x; // X-component or X-coordinate
    double y; // Y-component or Y-coordinate
} Vector2D;

/**
 * @brief Calculates the net force on the drone due to its original force and the repulsive field.
 * * The field generates a steering force that is tangential (perpendicular) to the 
 * vector from the obstacle to the drone, pushing it to the side. The direction of 
 * the steering (clockwise or counter-clockwise) is determined using the sign of 
 * the 2D cross product between the repulsion vector and the drone's current force.
 *
 * @param obstacle_pos Position (x, y) of the center of the repulsive field.
 * @param drone_pos Position (x, y) of the drone.
 * @param drone_force The drone's current force vector (Fx, Fy).
 * @param net_force Output structure to store the resulting net force vector.
 * @return The intensity (magnitude) of the resulting net force.
 */
double calculate_net_force(
    Vector2D obstacle_pos, 
    Vector2D drone_pos, 
    Vector2D drone_force, 
    Vector2D *net_force
);

#endif // FORCE_FIELD_H