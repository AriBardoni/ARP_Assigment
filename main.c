#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

// Helper to immediately exit when something goes wrong
static void die(const char *msg){ perror(msg); _exit(1); }

int main() {

    // Absolute path of the project folder (needed so child processes can exec files)
    char PATH[1024];
    if (getcwd(PATH, sizeof(PATH)) == NULL) {
        perror("getcwd");
        exit(1);
    }
    // Pipes:
    // ItoB = input → blackboard
    // BtoD = blackboard → drone
    // DtoB = drone → blackboard
    // OtoB = obstacles → blackboard
    // TtoB = targets → blackboard 
    int ItoB[2], BtoD[2], DtoB[2], OtoB[2], TtoB[2];

    // Create pipes. Each pipe has a read end [0] and write end [1]
    if(pipe(ItoB) < 0) die("ItoB");
    if(pipe(BtoD) < 0) die("BtoD");
    if(pipe(DtoB) < 0) die("DtoB");
    if(pipe(OtoB) < 0) die("OtoB");
    if(pipe(TtoB) < 0) die("TtoB");

    //DRONE 
    pid_t p = fork();
    if (p < 0) die("fork drone");

    if (p == 0) { // child: DRONE

        chdir(PATH);

        // Close all pipe ends not used in this process
        close(ItoB[0]); close(ItoB[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(TtoB[0]); close(TtoB[1]);
        close(BtoD[1]); // drone reads from BtoD[0]
        close(DtoB[0]); // drone writes to DtoB[1]

        // Convert pipe numbers into strings to pass as arguments
        char fdBtoD_r[16], fdDtoB_w[16];
        snprintf(fdBtoD_r,16,"%d",BtoD[0]);
        snprintf(fdDtoB_w,16,"%d",DtoB[1]);

        char dronePath[1024];
        snprintf(dronePath, sizeof(dronePath), "%s/drone", PATH);

        execlp(dronePath, "drone", fdBtoD_r, fdDtoB_w, NULL);
        die("exec drone");
    }

    //INPUT 
    pid_t p2 = fork();
    if (p2 < 0) die("fork input");

    if (p2 == 0) { // child: INPUT

        chdir(PATH);

        // Close pipe ends not used by input
        close(ItoB[0]); // input writes to ItoB[1]
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(TtoB[0]); close(TtoB[1]);

        // Ensure correct terminal file descriptors
        dup2(STDIN_FILENO, 0);
        dup2(STDOUT_FILENO, 1);
        dup2(STDERR_FILENO, 2);

        char fdItoB_w[16];
        snprintf(fdItoB_w, 16, "%d", ItoB[1]);

        char inputPath[1024];
        snprintf(inputPath, sizeof(inputPath), "%s/input", PATH);    

        char *argsI[] = { "konsole","--workdir", PATH,"-e", "./input", fdItoB_w, NULL };
        execvp("konsole", argsI);
        die("exec input");
    }

    //OBSTACLES 
    pid_t po = fork();
    if(po < 0) die("fork obstacles");
    if(po == 0){
        chdir(PATH);
        // close unused pipe ends
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(TtoB[0]); close(TtoB[1]);
        close(OtoB[0]); // write only

        char fdOtoB_w[16];
        snprintf(fdOtoB_w, 16, "%d", OtoB[1]);

        char obstaclesPath[1024];
        snprintf(obstaclesPath, sizeof(obstaclesPath), "%s/obstacles", PATH);

        execlp(obstaclesPath, "obstacles", fdOtoB_w, NULL);
        die("exec obstacles");
    }

    //TARGETS 
    pid_t pt = fork();
    if(pt < 0) die("fork targets");
    if(pt == 0){
        chdir(PATH);
        // close unused pipe ends
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(TtoB[0]); // write only

        char fdTtoB_w[16];
        snprintf(fdTtoB_w, 16, "%d", TtoB[1]);

        char targetsPath[1024];
        snprintf(targetsPath, sizeof(targetsPath), "%s/targets", PATH);

        execlp(targetsPath, "targets", fdTtoB_w, NULL);
        die("exec targets");
    }

    //BLACKBOARD 
    // Parent closes ends not used by the blackboard
    close(BtoD[0]); // blackboard writes on BtoD[1]
    close(DtoB[1]); // blackboard reads on DtoB[0]
    close(OtoB[1]); // blackboard reads on OtoB[0]
    close(TtoB[1]); // blackboard reads on TtoB[0]

    pid_t p3 = fork();
    if (p3 < 0) die("fork blackboard");

    if (p3 == 0) { // child: BLACKBOARD

        chdir(PATH);

        char fdItoB_r[16], fdBtoD_w[16], fdDtoB_r[16], fdOtoB_r[16], fdTtoB_r[16];
        snprintf(fdItoB_r,16,"%d",ItoB[0]);
        snprintf(fdBtoD_w,16,"%d",BtoD[1]);
        snprintf(fdDtoB_r,16,"%d",DtoB[0]);
        snprintf(fdOtoB_r,16,"%d",OtoB[0]);
        snprintf(fdTtoB_r,16,"%d",TtoB[0]);

        char blackPath[1024];
        snprintf(blackPath, sizeof(blackPath), "%s/blackboard", PATH);

        char *argsB[] = {
            "konsole",
            "--workdir", PATH,
            "-e", "./blackboard",
            blackPath,
            fdItoB_r,
            fdBtoD_w,
            fdDtoB_r,
            fdOtoB_r,
            fdTtoB_r,
            NULL
        };

        execvp("konsole", argsB);
        die("exec blackboard");
    }

    return 0; // parent exits
}
