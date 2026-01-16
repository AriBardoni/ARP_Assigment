#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include "logger.h"
#include "common.h"

// Helper per uscire in caso di errore critico
static void die(const char *msg){
    perror(msg);
    exit(1);
}

// Global PIDs
pid_t p_drone = 0, p_input = 0, p_obstacles = 0, p_targets = 0, p_blackboard = 0, p_watchdog = 0, p_network = 0;

void shutdown_handler(int sig) {
    (void)sig;
    printf("\nShutting down...\n"); 
    if (p_drone > 0) kill(p_drone, SIGKILL);
    if (p_input > 0) kill(p_input, SIGKILL);
    if (p_obstacles > 0) kill(p_obstacles, SIGKILL);
    if (p_targets > 0) kill(p_targets, SIGKILL);
    if (p_blackboard > 0) kill(p_blackboard, SIGKILL);
    if (p_network > 0) kill(p_network, SIGKILL);
    if (p_watchdog > 0) kill(p_watchdog, SIGKILL);
    exit(0);
}

// --- FUNZIONI DI SUPPORTO ---

// Spawn Blackboard (Adattato per supportare pipe di rete BtoN)
pid_t spawn_blackboard(char *start_path, char *wdPidStr, 
                       int *ItoB, int *BtoD, int *DtoB, int *OtoB, 
                       int *TtoB, int *BtoM, int *BtoN,
                       int is_server, int win_w, int win_h, int spawn_random) {
    pid_t p = fork();
    if (p < 0) die("fork blackboard failed");
    if (p == 0) {
        chdir(start_path);
        
        // Chiudi lati inutilizzati
        close(ItoB[1]);
        close(BtoD[0]);
        close(DtoB[1]); 
        close(OtoB[1]);
        close(TtoB[1]); 
        close(BtoM[0]); 
        
        // BtoN: Blackboard scrive verso Network
        if (BtoN) close(BtoN[0]);

        char fM[16] = "-1";
        if(is_server) snprintf(fM, 16, "%d", BtoM[1]);
        else close(BtoM[1]);
        
        char fR[16];
        snprintf(fR, 16, "%d", spawn_random);

        char fI[16], fB[16], fD[16], fO[16], fT[16], fBN[16];
        snprintf(fI, 16, "%d", ItoB[0]);
        snprintf(fB, 16, "%d", BtoD[1]);
        snprintf(fD, 16, "%d", DtoB[0]); 
        snprintf(fO, 16, "%d", OtoB[0]);
        snprintf(fT, 16, "%d", TtoB[0]);
        
        if(BtoN) snprintf(fBN, 16, "%d", BtoN[1]);
        else strcpy(fBN, "-1");
        
        char blackPath[1024];
        snprintf(blackPath, sizeof(blackPath), "%s/blackboard", start_path);

        char *argsB[20];
        int i = 0;
        argsB[i++] = "konsole";
        
        char geom[32];
        // Geometria fissa solo se siamo sicuri delle dimensioni
        if (win_w > 0 && win_h > 0) {
            snprintf(geom, sizeof(geom), "%dx%d", win_w, win_h);
            argsB[i++] = "--qwindowgeometry";
            argsB[i++] = geom;
        } else {
            // Default Geometry requested by user
            argsB[i++] = "--qwindowgeometry";
            argsB[i++] = "99x24";
        }

        argsB[i++] = "--workdir";
        argsB[i++] = start_path;
        argsB[i++] = "-e";
        argsB[i++] = blackPath;
        argsB[i++] = fI; // Arg 1
        argsB[i++] = fB; // Arg 2
        argsB[i++] = fD; // Arg 3
        argsB[i++] = fO; // Arg 4
        argsB[i++] = fT; // Arg 5
        argsB[i++] = wdPidStr; // Arg 6
        argsB[i++] = fM; // Arg 7
        argsB[i++] = fR; // Arg 8
        argsB[i++] = fBN; // Arg 9: Pipe to Network
        argsB[i++] = NULL;

        execvp("konsole", argsB);
        perror("Exec konsole failed (Blackboard Spawn)");
        _exit(1);
    }
    return p;
}

// --- NETWORK MODE ---
void run_network(char *start_path) {
    int mode;
    printf("\nNetwork Mode selected.\n1. Server\n2. Client\nChoose: ");
    fflush(stdout);
    if(scanf("%d", &mode) != 1) exit(1);

    // --- SETUP PIPES ---
    int ItoB[2], BtoD[2], DtoB[2], TtoB[2], BtoM[2], BtoN[2], NtoB[2];

    if(pipe(ItoB)<0 || pipe(BtoD)<0 || pipe(DtoB)<0 || pipe(TtoB)<0 || pipe(BtoM)<0 || pipe(BtoN)<0 || pipe(NtoB)<0)
        die("pipe failed");

    // Parametri per il processo Network
    char ip_str[64] = "127.0.0.1";
    int port_num = 0;
    int is_server_flag = 0;
    int win_w = 0, win_h = 0;

    if (mode == 1) { // SERVER
        printf("Enter Port to listen on (e.g. 5001): ");
        scanf("%d", &port_num);
        
        is_server_flag = 1;
        win_w = 80; win_h = 24; // Default server size
        
    } else { // CLIENT
        printf("Enter Server IP: ");
        scanf("%63s", ip_str);
        printf("Enter Server Port: ");
        scanf("%d", &port_num);
        
        is_server_flag = 0;
        // Client non sa la dimensione all'avvio, mette 0 e lascia fare a network
        win_w = 0; win_h = 0; 
    }

    // Watchdog disattivato per semplicità in rete
    char wdPidStr[] = "0";

    // --- 1. FORK BLACKBOARD ---
    // Passiamo NtoB al posto di Obstacles o Targets, oppure gestiamo la logica dentro Blackboard.
    // Qui passo NtoB come pipe "Obstacles" (simulati dalla rete) se necessario, 
    // ma la spawn_blackboard sopra ha l'argomento BtoN (Arg 9) specifico.
    // IMPORTANTE: Blackboard deve leggere da NtoB per aggiornare gli ostacoli remoti.
    // Nel tuo codice originale, Blackboard leggeva da OtoB. Qui useremo NtoB al posto di OtoB?
    // Per ora passo NtoB come pipe "Obstacles" nel parametro OtoB della funzione, è il trucco standard.
    
    p_blackboard = spawn_blackboard(start_path, wdPidStr,
                                    ItoB, BtoD, DtoB, NtoB, /* Uso NtoB come input Obstacles! */
                                    TtoB, BtoM, BtoN,
                                    is_server_flag, win_w, win_h, 0); 
    
    // Server chiude lato scrittura di BtoM (usato solo per Server Master logic se presente)
    if(mode == 1) close(BtoM[1]); 

    // --- 2. FORK DRONE ---
    pid_t p = fork();
    if(p < 0) die("fork drone failed");
    if(p==0){
        chdir(start_path);
        // Chiudi pipes inutili
        close(ItoB[0]); close(ItoB[1]);
        close(NtoB[0]); close(NtoB[1]); // Drone non parla col network diretto
        close(TtoB[0]); close(TtoB[1]);
        close(BtoD[1]); 
        close(BtoN[0]); close(BtoN[1]);
        close(BtoM[0]); close(BtoM[1]);
        close(DtoB[0]); 

        char fdBtoD_r[16], fdDtoB_w[16];
        snprintf(fdBtoD_r,16,"%d",BtoD[0]);
        snprintf(fdDtoB_w,16,"%d",DtoB[1]);

        char dronePath[1024];
        snprintf(dronePath,sizeof(dronePath),"%s/drone", start_path);

        // Posizioni diverse per test
        char startX[16], startY[16];
        if (mode == 1) { snprintf(startX, 16, "20.0"); snprintf(startY, 16, "50.0"); } // Server a SX
        else           { snprintf(startX, 16, "10.0"); snprintf(startY, 16, "20.0"); } // Client a DX

        execlp(dronePath, "drone", fdBtoD_r, fdDtoB_w, wdPidStr, startX, startY, NULL);
        die("Exec drone failed");
    }
    p_drone = p;

    // --- 3. FORK NETWORK PROCESS ---
    pid_t pn = fork();
    if (pn < 0) die("fork network failed");
    if (pn == 0) {
        chdir(start_path);
        
        // Chiudi pipe non usate
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(TtoB[0]); close(TtoB[1]);
        close(BtoM[0]); close(BtoM[1]); 
        
        // Configurazione Pipe Network
        // BtoN (Blackboard -> Network): Network legge da [0]
        close(BtoN[1]); 
        // NtoB (Network -> Blackboard): Network scrive su [1]
        close(NtoB[0]); 

        char str_port[16], str_bb2net[16], str_net2bb[16], str_w[16], str_h[16], str_role[16];

        snprintf(str_port, 16, "%d", port_num);
        snprintf(str_bb2net, 16, "%d", BtoN[0]); // Pipe Input (dalla BB)
        snprintf(str_net2bb, 16, "%d", NtoB[1]); // Pipe Output (verso BB)
        snprintf(str_w, 16, "%d", win_w);
        snprintf(str_h, 16, "%d", win_h);
        snprintf(str_role, 16, "%d", is_server_flag); // 1 Server, 0 Client

        char netPath[1024];
        snprintf(netPath, sizeof(netPath), "%s/network", start_path);
        
        // ARGOMENTI PER NETWORK: IP PORT PipeIn PipeOut W H IsServer
        execlp(netPath, "network", ip_str, str_port, str_bb2net, str_net2bb, str_w, str_h, str_role, NULL);
        die("Exec network failed");
    }
    p_network = pn;

    // --- 4. FORK INPUT ---
    pid_t p2 = fork();
    if(p2 < 0) die("fork input failed");
    if(p2==0){
        chdir(start_path);
        // Chiudi tutto tranne ItoB write
        close(ItoB[0]); 
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(NtoB[0]); close(NtoB[1]);
        close(TtoB[0]); close(TtoB[1]);
        close(BtoN[0]); close(BtoN[1]);
        close(BtoM[0]); close(BtoM[1]);
        
        char fdItoB_w[16];
        snprintf(fdItoB_w, 16, "%d", ItoB[1]);
        char inputPath[1024];
        snprintf(inputPath,sizeof(inputPath),"%s/input", start_path);
        
        char *argsI[] = {"konsole", "--workdir", start_path, "-e", inputPath, fdItoB_w, wdPidStr, NULL};
        execvp("konsole", argsI);
        die("Exec konsole failed (Input)");
    }
    p_input = p2;

    // --- PARENT: CHIUDE TUTTE LE PIPE ---
    close(ItoB[0]); close(ItoB[1]);
    close(BtoD[0]); close(BtoD[1]);
    close(DtoB[0]); close(DtoB[1]);
    close(NtoB[0]); close(NtoB[1]);
    close(TtoB[0]); close(TtoB[1]);
    close(BtoN[0]); close(BtoN[1]);
    close(BtoM[0]); close(BtoM[1]); 

    // Attesa infinita (i figli lavorano)
    while(1) {
        pause();
    }
}

// --- ORIGINAL ASSIGNMENT 2 (LOCAL) ---
void run_local(char *start_path) {
    printf("Starting Local Mode (Assignment 2)...\n");

    // Pipes:
    // ItoB = input → blackboard
    // BtoD = blackboard → drone
    // DtoB = drone → blackboard
    // OtoB = obstacles → blackboard
    // TtoB = targets → blackboard
    int ItoB[2], BtoD[2], DtoB[2], OtoB[2], TtoB[2];

    // Create pipes. Each pipe has a read end [0] and write end [1]
    if(pipe(ItoB) < 0) die("pipe ItoB failed");
    if(pipe(BtoD) < 0) die("pipe BtoD failed");
    if(pipe(DtoB) < 0) die("pipe DtoB failed");
    if(pipe(OtoB) < 0) die("pipe OtoB failed");
    if(pipe(TtoB) < 0) die("pipe TtoB failed");

    // Watchdog
    pid_t pw = fork();
    if (pw < 0) die("fork watchdog failed");
    if (pw == 0) {
        chdir(start_path);
        // Close unused
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(TtoB[0]); close(TtoB[1]);

        char wdPath[1024];
        snprintf(wdPath, sizeof(wdPath), "%s/watchdog", start_path);
        execlp(wdPath, "watchdog", NULL);
        _exit(1);
    }
    p_watchdog = pw;

    // Prepare Watchdog PID string to pass to children
    char wdPidStr[16];
    snprintf(wdPidStr, 16, "%d", p_watchdog);

    pid_t p = fork();
    if (p < 0) die("fork drone failed");

    if (p == 0) { // child: DRONE
        chdir(start_path);
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
        snprintf(dronePath, sizeof(dronePath), "%s/drone", start_path);

        execlp(dronePath, "drone", fdBtoD_r, fdDtoB_w, wdPidStr, NULL);
        _exit(1);
    }
    p_drone = p;

    pid_t p2 = fork();
    if (p2 < 0) die("fork input failed");

    if (p2 == 0) { // child: INPUT
        chdir(start_path);
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
        snprintf(inputPath, sizeof(inputPath), "%s/input", start_path);

        char *argsI[] = {
            "konsole",
            "--qwindowgeometry", "99x24", 
            "--workdir", start_path,
            "-e", inputPath, fdItoB_w, wdPidStr,
            NULL
        };
        execvp("konsole", argsI);
        _exit(1);
    }
    p_input = p2;

    pid_t po = fork();
    if(po < 0) die("fork obstacles failed");
    if(po == 0){
        chdir(start_path);
        // close unused pipe ends
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(TtoB[0]); close(TtoB[1]);
        close(OtoB[0]); // write only

        char fdOtoB_w[16];
        snprintf(fdOtoB_w, 16, "%d", OtoB[1]);

        char obstaclesPath[1024];
        snprintf(obstaclesPath, sizeof(obstaclesPath), "%s/obstacles", start_path);

        execlp(obstaclesPath, "obstacles", fdOtoB_w, wdPidStr, NULL);
        _exit(1);
    }
    p_obstacles = po;

    pid_t pt = fork();
    if(pt < 0) die("fork targets failed");
    if(pt == 0){
        chdir(start_path);
        // close unused pipe ends
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoB[0]); close(DtoB[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(TtoB[0]); // write only

        char fdTtoB_w[16];
        snprintf(fdTtoB_w, 16, "%d", TtoB[1]);

        char targetsPath[1024];
        snprintf(targetsPath, sizeof(targetsPath), "%s/targets", start_path);

        execlp(targetsPath, "targets", fdTtoB_w, wdPidStr, NULL);
        _exit(1);
    }
    p_targets = pt;

    // Parent closes ends not used by the blackboard
    close(BtoD[0]); // blackboard writes on BtoD[1]
    close(DtoB[1]); // blackboard reads on DtoB[0]
    close(OtoB[1]); // blackboard reads on OtoB[0]
    close(TtoB[1]); // blackboard reads on TtoB[0]

    pid_t p3 = fork();
    if (p3 < 0) die("fork blackboard failed");

    if (p3 == 0) { // child:blackboard
        chdir(start_path);

        char fdItoB_r[16], fdBtoD_w[16], fdDtoB_r[16], fdOtoB_r[16], fdTtoB_r[16];
        snprintf(fdItoB_r,16,"%d",ItoB[0]);
        snprintf(fdBtoD_w,16,"%d",BtoD[1]);
        snprintf(fdDtoB_r,16,"%d",DtoB[0]);
        snprintf(fdOtoB_r,16,"%d",OtoB[0]);
        snprintf(fdTtoB_r,16,"%d",TtoB[0]);

        char blackPath[1024];
        snprintf(blackPath, sizeof(blackPath), "%s/blackboard", start_path);

        char *argsB[] = {
            "konsole",
            "--qwindowgeometry", "99x24",
            "--workdir", start_path,
            "-e",
            blackPath,
            fdItoB_r,
            fdBtoD_w,
            fdDtoB_r,
            fdOtoB_r,
            fdTtoB_r,
            wdPidStr,
            "-1",
            NULL
        };

        execvp("konsole", argsB);
        _exit(1);
    }
    p_blackboard = p3;

    while(1) {
        pause();
    }
}


int main() {
    signal(SIGUSR1, shutdown_handler);

    // Init Logger
    printf("Main Process PID: %d. Initializing...\n", getpid());
    fflush(stdout);

    // Use current working directory as project path
     char PATH[1024];
    if (getcwd(PATH, sizeof(PATH)) == NULL) die("getcwd failed");
    strcat(PATH, "/bin");
    
    int choice;
    while(1){

        printf("Select Mode:\n1. Standalone\n2. Networked\n3. Quit\n");
        fflush(stdout);

        if(scanf("%d", &choice) != 1){
            while (getchar() != '\n'); // Svuota buffer
            continue;
        }

         if(choice == 1) {
            run_local(PATH);
            break;
        } else if (choice == 2) {
            run_network(PATH); // Assicurati di avere la funzione implementata o copiata dal vecchio codice
            break;
        } else if (choice == 3) {
            return 0;
        }
    }

    return 0;
}