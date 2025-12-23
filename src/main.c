#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <signal.h>

// Helper to immediately exit when something goes wrong
static void die(const char *msg){ perror(msg); _exit(1); }

// Global PIDs for signal handler
pid_t p_drone, p_input, p_obstacles, p_targets, p_blackboard, p_watchdog;

void shutdown_handler(int sig) {
    (void)sig;
    printf("Main: Received shutdown signal. Terminating all children...\n");
    if (p_drone > 0) kill(p_drone, SIGKILL);
    if (p_input > 0) kill(p_input, SIGKILL);
    if (p_obstacles > 0) kill(p_obstacles, SIGKILL);
    if (p_targets > 0) kill(p_targets, SIGKILL);
    if (p_blackboard > 0) kill(p_blackboard, SIGKILL);
    if (p_watchdog > 0) kill(p_watchdog, SIGKILL);
    exit(0);
}

int main() {
    signal(SIGUSR1, shutdown_handler);

    // Use current working directory as project path
    char PATH[1024];
    if (getcwd(PATH, sizeof(PATH)) == NULL) {
        perror("getcwd");
        exit(1);
    }

    // aggiunta: usare la cartella bin
    if (strlen(PATH) + 4 >= sizeof(PATH)) {
        fprintf(stderr, "PATH too long\n");
        exit(1);
    }
    strcat(PATH, "/bin");

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

    // Watchdog
    pid_t pw = fork();
    if (pw < 0) die("fork watchdog");
    if (pw == 0) {
        chdir(PATH);
        // Close unused
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(TtoB[0]); close(TtoB[1]);

        char wdPath[1024];
        snprintf(wdPath, sizeof(wdPath), "%s/watchdog", PATH);
        execlp(wdPath, "watchdog", NULL);
        die("exec watchdog");
    }
    p_watchdog = pw;

    // Prepare Watchdog PID string to pass to children
    char wdPidStr[16];
    snprintf(wdPidStr, 16, "%d", p_watchdog);

    pid_t p = fork();
    if (p < 0) die("fork drone");

    if (p == 0) { // child: DRONE
        chdir(PATH);
        // ... (drone setup) ...
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

        execlp(dronePath, "drone", fdBtoD_r, fdDtoB_w, wdPidStr, NULL);
        die("exec drone");
    }
    p_drone = p;

    pid_t p2 = fork();
    if (p2 < 0) die("fork input");

    if (p2 == 0) { // child: INPUT
        chdir(PATH);
        // ... (input setup) ...
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

        char *argsI[] = {
            "konsole",
            "--workdir", PATH,
            "-e", inputPath, fdItoB_w, wdPidStr,
            NULL
        };
        execvp("konsole", argsI);
        die("exec input");
    }
    p_input = p2;

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

        execlp(obstaclesPath, "obstacles", fdOtoB_w, wdPidStr, NULL);
        die("exec obstacles");
    }
    p_obstacles = po;

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

        execlp(targetsPath, "targets", fdTtoB_w, wdPidStr, NULL);
        die("exec targets");
    }
    p_targets = pt;

    // Parent closes ends not used by the blackboard
    close(BtoD[0]); // blackboard writes on BtoD[1]
    close(DtoB[1]); // blackboard reads on DtoB[0]
    close(OtoB[1]); // blackboard reads on OtoB[0]
    close(TtoB[1]); // blackboard reads on TtoB[0]

    pid_t p3 = fork();
    if (p3 < 0) die("fork blackboard");

    if (p3 == 0) { // child:blackboard 
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
            "-e",
            blackPath,
            fdItoB_r,
            fdBtoD_w,
            fdDtoB_r,
            fdOtoB_r,
            fdTtoB_r,
            wdPidStr,
            NULL
        };

        execvp("konsole", argsB);
        die("exec blackboard");
    }
    p_blackboard = p3;

    while(1) {
        pause();
    }
    return 0; 
}
