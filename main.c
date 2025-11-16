/*#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

static void die(const char *msg){ perror(msg); _exit(1); }

int main() {
    int ItoB[2], BtoD[2], DtoB[2];
    if (pipe(ItoB) < 0) die("pipe ItoB");
    if (pipe(BtoD) < 0) die("pipe BtoD");
    if (pipe(DtoB) < 0) die("pipeDtoB");

    // -----------------------------------------
    // PROCESSO INPUT (in una nuova Konsole!)
    // -----------------------------------------
    pid_t p = fork();
    if (p < 0) die("fork input");
    if (p == 0) {
        close(ItoB[0]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);

        char fdItoB_w[16];
        snprintf(fdItoB_w, sizeof(fdItoB_w), "%d", ItoB[1]);

        // APRI input in NUOVA KONSOLE
        char *argsI[] = {"konsole", "-e", "./input", fdItoB_w, NULL};
        execvp("konsole", argsI);

        die("exec input");
    }
    close(ItoB[1]);

    // -----------------------------------------
    // PROCESSO DRONE
    // -----------------------------------------
    p = fork();
    if (p < 0) die("fork drone");
    if (p == 0) {
        close(ItoB[0]);
        close(BtoD[1]);
        close(DtoB[0]);

        char fdBtoD_r[16], fdDtoB_w[16];
        snprintf(fdBtoD_r, sizeof(fdBtoD_r), "%d", BtoD[0]);
        snprintf(fdDtoB_w, sizeof(fdDtoB_w), "%d", DtoB[1]);

        char *argsD[] = {"./drone", fdBtoD_r, fdDtoB_w, NULL};
        execv("./drone", argsD);

        die("exec drone");
    }
    close(BtoD[0]);
    close(DtoB[1]);

    // -----------------------------------------
    // PROCESSO BLACKBOARD (ncurses)
    // -----------------------------------------
    char fdItoB_r[16], fdBtoD_w[16], fdDtoB_r[16];
    snprintf(fdItoB_r, sizeof(fdItoB_r), "%d", ItoB[0]);
    snprintf(fdBtoD_w, sizeof(fdBtoD_w), "%d", BtoD[1]);
    snprintf(fdDtoB_r, sizeof(fdDtoB_r), "%d", DtoB[0]);

    char *argsB[] = {"./blackboard", fdItoB_r, fdBtoD_w, fdDtoB_r, NULL};
    execv("./blackboard", argsB);

    die("exec blackboard");
}*/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

static void die(const char *msg){ perror(msg); _exit(1); }

int main() {

    // Percorso ASSOLUTO del progetto
    const char *PATH = "/mnt/c/Users/Ary/Desktop/Robotics/ARP/Assigment1";

    int ItoB[2], BtoD[2], DtoB[2];
    if(pipe(ItoB) < 0) die("ItoB");
    if(pipe(BtoD) < 0) die("BtoD");
    if(pipe(DtoB) < 0) die("DtoB");

    // ----------------------------------------------------
    // DRONE (senza terminale)
    // ----------------------------------------------------
    pid_t p = fork();
    if (p < 0) die("fork drone");

    if (p == 0) {

        chdir(PATH);

        close(ItoB[0]); 
        close(ItoB[1]);
        close(BtoD[1]);   // drone legge
        close(DtoB[0]);   // drone scrive

        char fdBtoD_r[16], fdDtoB_w[16];
        snprintf(fdBtoD_r,16,"%d",BtoD[0]);
        snprintf(fdDtoB_w,16,"%d",DtoB[1]);

        char dronePath[512];
        snprintf(dronePath, sizeof(dronePath), "%s/drone", PATH);

        execlp(dronePath, "drone", fdBtoD_r, fdDtoB_w, NULL);
        die("exec drone");
    }

    // ----------------------------------------------------
    // PROCESSO INPUT → in una NUOVA KONSOLE
    // ----------------------------------------------------

    close(ItoB[0]); // blackboard leggerà

    char fdItoB_w[16];
    snprintf(fdItoB_w, 16, "%d", ItoB[1]);

    pid_t p2 = fork();
    if (p2 < 0) die("fork input");

    if (p2 == 0) {

        char inputPath[512];
        snprintf(inputPath, sizeof(inputPath), "%s/input", PATH);

        char *argsI[] = {
            "konsole",
            "-e",
            "./input",
            fdItoB_w,
            NULL
        };

        execvp("konsole", argsI);
        die("exec input");
    }

    // ----------------------------------------------------
    // PROCESSO BLACKBOARD → nuova KONSOLE
    // ----------------------------------------------------

    close(BtoD[0]);  // blackboard scrive
    close(DtoB[1]);  // blackboard legge

    char fdBtoD_w[16], fdDtoB_r[16];
    snprintf(fdBtoD_w,16,"%d",BtoD[1]);
    snprintf(fdDtoB_r,16,"%d",DtoB[0]);

    pid_t p3 = fork();
    if (p3 < 0) die("fork blackboard");

    if (p3 == 0) {

        char blackPath[512];
        snprintf(blackPath, sizeof(blackPath), "%s/blackboard", PATH);

        char *argsB[] = {
            "konsole",
            "-e",
            blackPath,
            fdItoB_w, fdBtoD_w, fdDtoB_r,
            NULL
        };

        execvp("konsole", argsB);
        die("exec blackboard");
    }

    return 0;
}






