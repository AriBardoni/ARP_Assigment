#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

// Helper to immediately exit when something goes wrong
static void die(const char *msg){ perror(msg); _exit(1); }

int main() {

    // Absolute path of the project folder (needed so child processes can exec files)
    const char *PATH = "/mnt/c/Users/Ary/Desktop/Robotics/ARP/Assigment1/ARP_Assigment";

    // Pipes:
    // ItoB = input → blackboard
    // BtoD = blackboard → drone
    // DtoB = drone → blackboard
    int ItoB[2], BtoD[2], DtoB[2];

    // Create pipes. Each pipe has a read end [0] and write end [1]
    if(pipe(ItoB) < 0) die("ItoB");
    if(pipe(BtoD) < 0) die("BtoD");
    if(pipe(DtoB) < 0) die("DtoB");

    pid_t p = fork();              // create first child (drone)
    if (p < 0) die("fork drone");

    if (p == 0) {                  // child: DRONE

        chdir(PATH);               // change directory to project folder

        // Close all pipe ends we do NOT use in this process
        close(ItoB[0]);
        close(ItoB[1]);
        close(BtoD[1]);   // drone reads from BtoD[0], so close write end
        close(DtoB[0]);   // drone writes to DtoB[1], so close read end

        // Convert pipe numbers into strings to pass as arguments
        char fdBtoD_r[16], fdDtoB_w[16];
        snprintf(fdBtoD_r,16,"%d",BtoD[0]);  // read end for drone
        snprintf(fdDtoB_w,16,"%d",DtoB[1]);  // write end for drone

        char dronePath[512];
        snprintf(dronePath, sizeof(dronePath), "%s/drone", PATH);

        // Replace current process with the drone executable
        execlp(dronePath, "drone", fdBtoD_r, fdDtoB_w, NULL);

        die("exec drone");    // only executed if exec fails
    }


    pid_t p2 = fork();             // create second child (input)
    if (p2 < 0) die("fork input");

    if (p2 == 0) {                 // child: INPUT

        chdir(PATH);

        // Close pipe ends not used by input
        close(ItoB[0]);    // input writes to ItoB[1]
        close(BtoD[0]);
        close(BtoD[1]);
        close(DtoB[0]);
        close(DtoB[1]);

        // Ensure correct terminal file descriptors
        dup2(STDIN_FILENO, 0);
        dup2(STDOUT_FILENO, 1);
        dup2(STDERR_FILENO, 2);

        // Convert pipe fd to string
        char fdItoB_w[16];
        snprintf(fdItoB_w, 16, "%d", ItoB[1]);

        char inputPath[512];
        snprintf(inputPath, sizeof(inputPath), "%s/input", PATH);

        // Launch input program inside a Konsole window
        char *argsI[] = {
            "konsole",
            "-e",
            inputPath,
            fdItoB_w,
            NULL
        };

        execvp("konsole", argsI);
        die("exec input");
    }

    // Parent closes the ends not used by the blackboard
    close(BtoD[0]); // blackboard writes on BtoD[1]
    close(DtoB[1]); // blackboard reads on DtoB[0]

    pid_t p3 = fork();
    if (p3 < 0) die("fork blackboard");

    if (p3 == 0) {                 // child: BLACKBOARD

        chdir(PATH);

        // Convert pipe fds to strings
        char fdItoB_r[16], fdBtoD_w[16], fdDtoB_r[16];
        snprintf(fdItoB_r,16,"%d",ItoB[0]);  // blackboard reads from here
        snprintf(fdBtoD_w,16,"%d",BtoD[1]);  // blackboard writes to drone
        snprintf(fdDtoB_r,16,"%d",DtoB[0]);  // blackboard reads drone state

        char blackPath[512];
        snprintf(blackPath, sizeof(blackPath), "%s/blackboard", PATH);

        // Launch blackboard inside Konsole
        char *argsB[] = {
            "konsole",
            "-e",
            blackPath,
            fdItoB_r,
            fdBtoD_w,
            fdDtoB_r,
            NULL
        };

        execvp("konsole", argsB);
        die("exec blackboard");
    }

    return 0;   // parent exits
}
