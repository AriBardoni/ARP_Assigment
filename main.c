#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

static void die(const char *msg){ perror(msg); _exit(1); }

int main() {

    // Path ASSOLUTO al progetto
    const char *PATH = "/mnt/c/Users/Ary/Desktop/Robotics/ARP/Assigment1/ARP_Assigment";

    int ItoB[2], BtoD[2], DtoB[2];
    if(pipe(ItoB) < 0) die("ItoB");
    if(pipe(BtoD) < 0) die("BtoD");
    if(pipe(DtoB) < 0) die("DtoB");

    // =====================================================
    // ============= PROCESSO DRONE =========================
    // =====================================================

    pid_t p = fork();
    if (p < 0) die("fork drone");

    if (p == 0) {

        chdir(PATH);

        // FD usati dal drone
        close(ItoB[0]);
        close(ItoB[1]);
        close(BtoD[1]);   // drone legge
        close(DtoB[0]);   // drone scrive

        // Preparazione argomenti
        char fdBtoD_r[16], fdDtoB_w[16];
        snprintf(fdBtoD_r,16,"%d",BtoD[0]);
        snprintf(fdDtoB_w,16,"%d",DtoB[1]);

        char dronePath[512];
        snprintf(dronePath, sizeof(dronePath), "%s/drone", PATH);

        execlp(dronePath, "drone", fdBtoD_r, fdDtoB_w, NULL);
        die("exec drone");
    }

        // =====================================================
    // ============= PROCESSO INPUT =========================
    // =====================================================

    pid_t p2 = fork();
    if (p2 < 0) die("fork input");

    if (p2 == 0) {

        chdir(PATH);

        // CHIUDI tutte le pipe che NON servono
        close(ItoB[0]);    // input scrive
        // input non deve vedere queste:
        close(BtoD[0]);
        close(BtoD[1]);
        close(DtoB[0]);
        close(DtoB[1]);

        // molto importante: assicura che STDIN,STDOUT,STDERR
        // siano ancora la Konsole e NON pipe
        // (WSL/Konsole lo garantisce di solito, ma non rischiamo)
        dup2(STDIN_FILENO, 0);
        dup2(STDOUT_FILENO, 1);
        dup2(STDERR_FILENO, 2);

        char fdItoB_w[16];
        snprintf(fdItoB_w, 16, "%d", ItoB[1]);

        char inputPath[512];
        snprintf(inputPath, sizeof(inputPath), "%s/input", PATH);

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


    // =====================================================
    // ============ PROCESSO BLACKBOARD =====================
    // =====================================================

    close(BtoD[0]); // blackboard scrive
    close(DtoB[1]); // blackboard legge

    pid_t p3 = fork();
    if (p3 < 0) die("fork blackboard");

    if (p3 == 0) {

        chdir(PATH);

        char fdItoB_r[16], fdBtoD_w[16], fdDtoB_r[16];
        snprintf(fdItoB_r,16,"%d",ItoB[1]); // NOTA
        snprintf(fdBtoD_w,16,"%d",BtoD[1]);
        snprintf(fdDtoB_r,16,"%d",DtoB[0]);

        char blackPath[512];
        snprintf(blackPath, sizeof(blackPath), "%s/blackboard", PATH);

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

    return 0;
}
