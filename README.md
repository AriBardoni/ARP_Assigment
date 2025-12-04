# ARP Assignment - Drone Simulation

This project implements a multi-process simulation of a drone moving in a 2D space with 2 degrees of freedom (horizontal and vertical), influenced by user inputs, obstacles and targets. The system is built in C and uses pipes for inter-process communication and `ncurses` for the user interface.

## Project Structure

The project consists of several independent processes managed by a master process:

*   **`main`**: The entry point. It spawns all other processes and sets up the communication pipes.
*   **`drone`**: Simulates the physics and dynamics of the drone. It receives forces and updates its position and velocity.
*   **`input`**: Captures keyboard input from the user to control the drone.
*   **`blackboard`**: The central hub and visualization window. It receives data from all other processes, computes repulsive forces from obstacles and walls, and displays the simulation state.
*   **`obstacles`**: Generates static obstacles in random positions.
*   **`targets`**: Generates static targets in random positions.

## Prerequisites

*   GCC compiler
*   `ncurses` library (libncurses)
*   `konsole` (used for spawning separate terminal windows)

## Build Instructions

To build the project, run `make` in the root directory:

```bash
make
```

This will compile all the components and generate the following executables: `main`, `drone`, `input`, `blackboard`, `obstacles`, `targets`.

To clean the build artifacts, run:

```bash
make clean
```

## Usage

To start the simulation, run the `main` executable:

```bash
./main
```

This will open separate `konsole` windows for the `input` and `blackboard` processes.

### Controls

Focus on the **Input Controller** window to control the drone.

*   **Movement (Force application):**
    *   `i`: Up
    *   `k`: Down
    *   `j`: Left
    *   `l`: Right
    *   `u`: Up-Left
    *   `o`: Up-Right
    *   `n`: Down-Left
    *   `,`: Down-Right
*   **Commands:**
    *   `b`: Brake (stops the drone)
    *   `r`: Reset (resets drone position to center)
    *   `q`: Quit the simulation

### Visualization

The **Blackboard** window displays the simulation map:

*   `+`: Drone position
*   `O`: Obstacles
*   `*`: Targets
*   The drone is subject to repulsive forces from the walls and obstacles.

## Architecture & Data Flow

The processes communicate via unnamed pipes. The data flow is as follows:

1.  **Input -> Blackboard**: User commands and force increments (`KeyMsg`).
2.  **Blackboard -> Drone**: Total calculated force (User input + Repulsive forces) (`ForceMsg`).
3.  **Drone -> Blackboard**: Current state (Position, Velocity) (`StateMsg`).
4.  **Obstacles -> Blackboard**: Obstacle positions (`ObjMsg`).
5.  **Targets -> Blackboard**: Target positions (`ObjMsg`).

## Logging

The system generates log files for debugging purposes:
*   `drone.log`: Logs drone position and velocity.
*   `input.log`: Logs key presses.
