#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/time.h>
#include <signal.h>
#include <errno.h>
#include <sys/select.h>
#include <fcntl.h>
#include "logger.h"
#include "common.h"

// Helper to immediately exit when something goes wrong
static void die(const char *msg){
    perror(msg);
    log_system("MAIN", msg);
    exit(1);
}

// Global PIDs for signal handler
pid_t p_drone = 0, p_input = 0, p_obstacles = 0, p_targets = 0, p_blackboard = 0, p_watchdog = 0, p_network = 0;

void shutdown_handler(int sig) {
    (void)sig;

    log_system("MAIN", "received shutdown signal, terminating children");

    if (p_drone > 0) kill(p_drone, SIGKILL);
    if (p_input > 0) kill(p_input, SIGKILL);
    if (p_obstacles > 0) kill(p_obstacles, SIGKILL);
    if (p_targets > 0) kill(p_targets, SIGKILL);
    if (p_blackboard > 0) kill(p_blackboard, SIGKILL);
    if (p_network > 0) kill(p_network, SIGKILL);
    if (p_watchdog > 0) kill(p_watchdog, SIGKILL);

    log_system("MAIN", "shutdown complete");
    exit(0);
}

// Struct to exchange dimensions
typedef struct {
    int w;
    int h;
} NetInitMsg;

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

// Helper to spawn Blackboard (Deduplicated)
pid_t spawn_blackboard(char *start_path, char *wdPidStr, int socket_fd, 
                       int *ItoB, int *BtoD, int *DtoN, int *OtoB, int *NtoB_Drone, int *BtoM,
                       int is_server, int win_w, int win_h) {
    pid_t p = fork();
    if (p < 0) die("fork blackboard failed");
    if (p == 0) {
        chdir(start_path);
        
        // Close ends
        if (socket_fd >= 0) close(socket_fd);
        close(ItoB[1]);
        close(BtoD[0]);
        close(DtoN[0]); close(DtoN[1]);
        close(OtoB[1]);
        close(NtoB_Drone[1]);
        close(BtoM[0]); // Always close read end
        
        char fM[16] = "-1";
        if(is_server) {
            snprintf(fM, 16, "%d", BtoM[1]);
        } else {
            close(BtoM[1]);
        }

        char fI[16], fB[16], fD[16], fO[16];
        snprintf(fI, 16, "%d", ItoB[0]);
        snprintf(fB, 16, "%d", BtoD[1]);
        snprintf(fD, 16, "%d", NtoB_Drone[0]);
        snprintf(fO, 16, "%d", OtoB[0]);
        
        char blackPath[1024];
        snprintf(blackPath, sizeof(blackPath), "%s/blackboard", start_path);

        char *argsB[16];
        int i = 0;
        argsB[i++] = "konsole";
        
        char geom[32];
        if (!is_server && win_w > 0 && win_h > 0) {
            snprintf(geom, sizeof(geom), "%dx%d", win_w, win_h);
            argsB[i++] = "--qwindowgeometry";
            argsB[i++] = geom;
        }

        argsB[i++] = "--workdir";
        argsB[i++] = start_path;
        argsB[i++] = "-e";
        argsB[i++] = blackPath;
        argsB[i++] = fI;
        argsB[i++] = fB;
        argsB[i++] = fD;
        argsB[i++] = fO;
        argsB[i++] = "-1"; // TtoB unused
        argsB[i++] = wdPidStr;
        argsB[i++] = fM;
        argsB[i++] = NULL;

        execvp("konsole", argsB);
        _exit(1);
    }
    return p;
}

// Helper to spawn Watchdog for Network Mode
pid_t spawn_watchdog_network(char *start_path, int socket_fd, 
                             int *ItoB, int *BtoD, int *DtoN, int *OtoB, int *NtoB_Drone, int *BtoM) {
    pid_t pw = fork();
    if (pw < 0) die("fork watchdog failed");
    if (pw == 0) {
        chdir(start_path);
        // Close unused
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoN[0]); close(DtoN[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(NtoB_Drone[0]); close(NtoB_Drone[1]);
        close(BtoM[0]); close(BtoM[1]);
        if (socket_fd >= 0) close(socket_fd);
        
        char wdPath[1024];
        snprintf(wdPath, sizeof(wdPath), "%s/watchdog", start_path);
        execlp(wdPath, "watchdog", NULL);
        _exit(1);
    }
    return pw;
}

// --- NETWORK MODE (ASSIGNMENT 3) ---

void run_network(char *start_path) {
    int mode;
    printf("Network Mode selected.\n1. Server\n2. Client\nChoose: ");
    if(scanf("%d", &mode) != 1) exit(1);

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0) die("socket failed");
    
    // Enable address reuse
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Performance: Disable Nagle (TCP_NODELAY)
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    // Robustness: Receive Timeout removed for listening socket to avoid accept errors
    // struct timeval tv = {5, 0}; // 5s timeout
    // setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;

    int win_w = 0, win_h = 0;
    int client_or_server_fd = -1;

    // Pipes initialization
    // ItoB = input -> blackboard
    // BtoD = blackboard -> drone
    // DtoN = drone -> network (local state update)
    // OtoB = obstacles -> blackboard 
    // NtoB_Drone = network -> blackboard (remote drone position)
    // BtoM = blackboard -> main (dimensions)
    
    int ItoB[2], BtoD[2], DtoN[2], OtoB[2], TtoB[2]; 
    int NtoB_Drone[2], BtoM[2]; 

    if(pipe(ItoB)<0 || pipe(BtoD)<0 || pipe(DtoN)<0 || pipe(OtoB)<0 || pipe(TtoB)<0 || pipe(NtoB_Drone)<0 || pipe(BtoM)<0)
        die("pipe failed");

    // Targets ununsed in network mode
    close(TtoB[0]); close(TtoB[1]);

    // Watchdog logic moved to Server/Client blocks
    p_watchdog = 0;
    char wdPidStr[16];
    snprintf(wdPidStr, 16, "0"); // Default to 0 (disabled) until started

    if (mode == 1) { // SERVER
        int port;
        printf("Enter Port to listen on (e.g. 5000): ");
        if (scanf("%d", &port) != 1 || port < 1024 || port > 65535) {
             die("Invalid port (must be 1024-65535)");
        }
        // Dimensions NOT asked here anymore

        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if(bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) die("bind failed");
        listen(sockfd, 1);
        printf("Waiting for client on port %d...\n", port);

        int clientfd = accept(sockfd, NULL, NULL);
        if(clientfd < 0) die("accept failed");
        printf("Client connected!\n");
        
        client_or_server_fd = clientfd;
        close(sockfd); 

        // Start Watchdog AFTER connection established
        p_watchdog = spawn_watchdog_network(start_path, client_or_server_fd,
                                            ItoB, BtoD, DtoN, OtoB, NtoB_Drone, BtoM);
        snprintf(wdPidStr, 16, "%d", p_watchdog); 

        // Fork Blackboard NOW to get dimensions
        p_blackboard = spawn_blackboard(start_path, wdPidStr, client_or_server_fd,
                                        ItoB, BtoD, DtoN, OtoB, NtoB_Drone, BtoM,
                                        1, 0, 0);

        // Parent reads dimensions
        close(BtoM[1]); // Close write end
        struct { int w; int h; } dims;
        ssize_t n = read(BtoM[0], &dims, sizeof(dims));
        if (n == sizeof(dims)) {
            win_w = dims.w;
            win_h = dims.h;
            printf("Detected Blackboard Dimensions: %d x %d\n", win_w, win_h);
        } else {
            printf("Failed to read dimensions from blackboard. Using defaults.\n");
            win_w = 80; win_h = 24;
        }
        close(BtoM[0]);

        // Send Dimensions
        NetInitMsg init = { win_w, win_h };
        send(clientfd, &init, sizeof(init), 0);
        
    } else { // CLIENT
        char ip[64];
        int port;
        printf("Enter Server IP: ");
        scanf("%63s", ip);
        printf("Enter Server Port: ");
        if (scanf("%d", &port) != 1 || port < 1024 || port > 65535) {
             die("Invalid port");
        }

        if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) die("Invalid address");
        addr.sin_port = htons(port);

        if(connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) die("connect failed");
        printf("Connected to Server!\n");

        // Receive Dimensions
        NetInitMsg init;
        recv(sockfd, &init, sizeof(init), 0);
        printf("Received Dimensions: %d x %d\n", init.w, init.h);
        
        win_w = init.w;
        client_or_server_fd = sockfd;

        // Watchdog DISABLED in Client mode (p_watchdog = 0)
        p_watchdog = 0;
        snprintf(wdPidStr, 16, "0");

        // Fork Blackboard
        p_blackboard = spawn_blackboard(start_path, wdPidStr, client_or_server_fd,
                                        ItoB, BtoD, DtoN, OtoB, NtoB_Drone, BtoM,
                                        0, win_w, win_h);
    }

    // Drone
    pid_t p = fork();
    if(p < 0) die("fork drone failed");
    if(p==0){
        chdir(start_path);
        
        close(ItoB[0]); close(ItoB[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(BtoD[1]); 
        close(NtoB_Drone[0]); close(NtoB_Drone[1]);
        close(DtoN[0]); 
        close(client_or_server_fd);
        close(BtoM[0]); close(BtoM[1]);

        char fdBtoD_r[16], fdDtoN_w[16];
        snprintf(fdBtoD_r,16,"%d",BtoD[0]);
        snprintf(fdDtoN_w,16,"%d",DtoN[1]);

        char dronePath[1024];
        snprintf(dronePath,sizeof(dronePath),"%s/drone", start_path);
        execlp(dronePath, "drone", fdBtoD_r, fdDtoN_w, wdPidStr, NULL);
        _exit(1);
    }
    p_drone = p;

    // Network Process
    pid_t pn = fork();
    if (pn < 0) die("fork network failed");
    if (pn == 0) {
        chdir(start_path);
                
        close(ItoB[0]); close(ItoB[1]);
        close(BtoD[0]); close(BtoD[1]);
        close(DtoN[1]); // Write side
        close(NtoB_Drone[0]); // Read side
        close(OtoB[0]); // Read side
        close(BtoM[0]); close(BtoM[1]);
        
        char sfd[16], fdD_r[16], fdO_w[16], fdSelf_w[16], winW_str[16], winH_str[16], role_str[16];
        snprintf(sfd, 16, "%d", client_or_server_fd);
        snprintf(fdD_r, 16, "%d", DtoN[0]);
        snprintf(fdO_w, 16, "%d", OtoB[1]);
        snprintf(fdSelf_w, 16, "%d", NtoB_Drone[1]);
        snprintf(winW_str, 16, "%d", win_w);
        snprintf(winH_str, 16, "%d", win_h);
        snprintf(role_str, 16, "%d", (mode == 1));

        char netPath[1024];
        snprintf(netPath, sizeof(netPath), "%s/network", start_path);
        
        execlp(netPath, "network", sfd, fdD_r, fdO_w, wdPidStr, fdSelf_w, winW_str, winH_str, role_str, NULL);
        _exit(1);
    }
    p_network = pn;

    // Input
    pid_t p2 = fork();
    if(p2 < 0) die("fork input failed");
    if(p2==0){
        chdir(start_path);
        close(ItoB[0]); 
        close(BtoD[0]); close(BtoD[1]);
        close(DtoN[0]); close(DtoN[1]);
        close(OtoB[0]); close(OtoB[1]);
        close(NtoB_Drone[0]); close(NtoB_Drone[1]);
        close(client_or_server_fd);
        close(BtoM[0]); close(BtoM[1]);
        
        char fdItoB_w[16];
        snprintf(fdItoB_w, 16, "%d", ItoB[1]);
        char inputPath[1024];
        snprintf(inputPath,sizeof(inputPath),"%s/input", start_path);
        
        char *argsI[] = {"konsole", "--workdir", start_path, "-e", inputPath, fdItoB_w, wdPidStr, NULL};
        execvp("konsole", argsI);
        _exit(1);
    }
    p_input = p2;

    // Parent closes all
    close(ItoB[0]); close(ItoB[1]);
    close(BtoD[0]); close(BtoD[1]);
    close(DtoN[0]); close(DtoN[1]);
    close(OtoB[0]); close(OtoB[1]);
    close(NtoB_Drone[0]); close(NtoB_Drone[1]);
    close(client_or_server_fd);
    close(BtoM[0]); close(BtoM[1]); // Close BtoM

    while(1) {
        pause();
    }
}

int main() {
    signal(SIGUSR1, shutdown_handler);

    logger_init("logs");
    log_process_register("MAIN", getpid());
    log_system("MAIN", "started");

    // Use current working directory as project path
    char PATH[1024];
    if (getcwd(PATH, sizeof(PATH)) == NULL) {
        log_system("MAIN", "getcwd failed");
        exit(1);
    }

    // aggiunta: usare la cartella bin
    if (strlen(PATH) + 4 >= sizeof(PATH)) {
        log_system("MAIN", "PATH too long");
        exit(1);
    }
    strcat(PATH, "/bin");
    
    int choice;
    printf("Select Mode:\n1. Standalone\n2. Networked\n3. Quit\n");
    if(scanf("%d", &choice) != 1){
        printf("Invalid choice. Shutting down\n");
        return 1;
    }

    while(choice != 3) {
        if(choice == 1) {
            run_local(PATH);
        } else if (choice == 2) {
            run_network(PATH);
        }
        printf("Invalid choice. Please try again.\n");
        printf("Select Mode:\n1. Standalone\n2. Networked\n3. Quit: q\n");
        if(scanf("%d", &choice) != 1){
            printf("Invalid choice. Shutting down\n");
            return 1;
        }
    }

    return 0;
}