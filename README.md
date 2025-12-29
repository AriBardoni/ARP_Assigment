# ARP Assignment - Drone Simulation

This project implements a multi-process simulation of a drone moving in a 2D space with 2 degrees of freedom (horizontal and vertical), influenced by user inputs, obstacles and targets. The system is built in C and uses **pipes** for data exchange and **signals** for process monitoring. The user interface is built using `ncurses`.

## Project Structure

The project consists of several independent processes managed by a master process:

*   **`main`**: The entry point. Spawns all child processes, creates pipes, and manages the system shutdown.
*   **`watchdog`**: Monitors the health of all other processes. It receives heartbeats signals, logs activity, and triggers a system-wide shutdown if any process becomes unresponsive.
*   **`drone`**: Simulates the physics and dynamics of the drone. It receives forces and updates its position and velocity.
*   **`input`**: Captures keyboard input from the user to control the drone.
*   **`blackboard`**: The central hub and visualization window. It receives data from other processes, computes repulsive forces, and displays the simulation state.
*   **`obstacles`**: Generates static obstacles in random positions.
*   **`targets`**: Generates static targets in random positions.

### Directory Structure
*   `src/`: Source code modules.
*   `include/`: Header files (`common.h`, `logger.h`).
*   `logs/`: Runtime log files.
*   `bin/`: Compiled executables.
*   `build/`: Object files.

## Prerequisites

*   GCC compiler
*   `ncurses` library (libncurses)
*   `konsole` (used for spawning separate terminal windows)

## Build Instructions

To build the project, run `make` in the root directory:

```bash
make
```

This will compile all components and generate the executables in the `bin/` directory.

To clean the build artifacts, run:

```bash
make clean
```

## Usage

To start the simulation, run the `main` executable:

```bash
./main
```

This will:
1.  Start the **Watchdog** process in the background.
2.  Open separate `konsole` windows for the **Input Controller** and **Blackboard**.
3.  Initialize the Drone, Obstacles, and Targets processes.

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
*   Repulsive forces from walls and obstacles affect the drone's trajectory.

## Architecture & Data Flow

The system uses a hybrid communication model:
1.  **Pipes**: Used for data flow (Controls, Telemetry, Object positions).
2.  **Signals (SIGUSR1)**: Used for the Watchdog mechanism (Heartbeats).

```mermaid
graph TD
    subgraph Data Flow [Pipes]
    Input[Input Process] -->|KeyMsg| Blackboard[Blackboard Process]
    Blackboard -->|ForceMsg| Drone[Drone Process]
    Drone -->|StateMsg| Blackboard
    Obstacles[Obstacles Process] -->|ObjMsg| Blackboard
    Targets[Targets Process] -->|ObjMsg| Blackboard
    end

    subgraph Monitoring [Signals]
    Watchdog[Watchdog Process] -.->|Monitor| Input
    Watchdog -.->|Monitor| Drone
    Watchdog -.->|Monitor| Blackboard
    Watchdog -.->|Monitor| Obstacles
    Watchdog -.->|Monitor| Targets
    end
```

### Watchdog System
The `watchdog` process verifies that all critical processes are alive and responsive.
*   Each process sends a **Signal** (SIGUSR1) to the watchdog periodically.
*   The signal payload contains the Process ID and the current "Code Area" it is executing.
*   If the watchdog does not receive a signal from a process within `TIMEOUT_THRESHOLD`, it logs an alert and signals `main` to terminate the entire simulation.

## Logging

The system implements a centralized logging mechanism in `src/logger.c`. Logs are stored in the `logs/` directory:

*   **`processes.log`**: Registration of all started processes (Name, PID, Start Time).
*   **`watchdog.log`**: Specific logs from the watchdog process.
*   **`system.log`**: Critical system errors and shutdown events.
*   **`LogFile1`**: Detailed operational logs from the Watchdog (heartbeats, checks).
*   **`LogFile2`**: (Reserved for general process logging).
*   **`drone.log` / `input.log`**: Process-specific log files.

The logging system uses file locking (`flock`) to ensure thread-safe writing from multiple concurrent processes.

## System Evolution & Improvements

This version of the project introduces significant architectural shifts compared to previous iterations, moving from a simple functional simulation to a robust, self-monitored system.

1.  **Architecture: Closed-Loop Monitoring**
    *   **Previous**: The system was an "open loop" pipeline. Processes executed independently without supervision; if a component acted as a zombie or froze, the system continued indefinitely.
    *   **Current**: Use of a **Watchdog** creates a closed control loop. The system actively observes its own health, ensuring that a failure in one component triggers a safe, coordinated shutdown of the entire array.

2.  **Communication: Hybrid Model**
    *   **Previous**: Relied strictly on pipes for all interactions, potentially mixing high-volume data (telemetry) with critical control signals.
    *   **Current**: **Separation of Concerns**.
        *   **Pipes** are dedicated exclusively to data flow (positions, keypresses).
        *   **Signals (SIGUSR1)** are dedicated to the control plane (monitoring), ensuring that health checks can still succeed even if data pipes are saturated.

3.  **Observability & Concurrency**
    *   **Previous**: Ad-hoc logging to individual text files with no synchronization.
    *   **Current**: **Centralized, Thread-Safe Logging**. The new `logger` module uses file locking (`flock`) to prevent race conditions when multiple processes write to the same log file simultaneously. Logs are now structured by purpose (Lifecycle, Health, Errors) rather than just by origin.
  
## Corrections from assignment 1

Following the corrections and feedback provided by the professor on Assignment 1, the wall–repulsion logic has been revised.
In the previous implementation, the repulsive force generated by the walls caused the drone to bounce back upon collision. This behavior has been modified to better reflect a more realistic and controlled interaction with the environment.

In the current version, when the drone reaches a wall, the repulsive logic prevents further penetration by nullifying the velocity component directed toward the wall. As a result, the drone stops against the wall instead of rebounding, ensuring a more stable and predictable behavior consistent with the updated project requirements.
