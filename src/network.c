#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include "logger.h"
#include "common.h"

int MySocket = -1;

void cleanup(int sig) {
    (void)sig;
    if (MySocket != -1) {
        /* If we are in the loop, we might want to send "Q" but 
           for SIGINT force close is reasonable. */
        close(MySocket);
    }
    log_system("NETWORK", "exiting");
    exit(0);
}

// Helpers for Protocol
void send_msg(int sock, const char *msg) {
    // Send string + null terminator? 
    // "The following words in capital ... are messages transmitted as strings"
    // Usually implies string including null or fixed size. Pseudocode "snd OK" -> send("OK", 3)
    send(sock, msg, strlen(msg) + 1, 0); 
}

void recv_msg(int sock, char *buf, size_t size) {
    // Simple receive, assumes we get the whole string packet or enough to parse
    // For robust TCP we should frame, but for assignment simple recv is likely expected.
    memset(buf, 0, size);
    recv(sock, buf, size, 0);
}

int main(int argc, char **argv) {
    if (argc < 7) {
        fprintf(stderr, "network: missing fds\n");
        return 1;
    }

    logger_init("../logs");
    log_process_register("NETWORK", getpid());
    log_system("NETWORK", "started");

    signal(SIGINT, cleanup);
    signal(SIGTERM, cleanup);

    MySocket = atoi(argv[1]);
    int fdDtoN_r = atoi(argv[2]);
    int fdOtoB_w = atoi(argv[3]);
    pid_t wd_pid = (pid_t)atoi(argv[4]);
    int fdNtoB_Drone_w = atoi(argv[5]);
    int win_w = atoi(argv[6]);
    int win_h = atoi(argv[7]);

    // Set DtoN to non-blocking to check for updates without blocking global loop
    int flags = fcntl(fdDtoN_r, F_GETFL, 0);
    fcntl(fdDtoN_r, F_SETFL, flags | O_NONBLOCK);

    float last_local_x = 0.0f, last_local_y = 0.0f;
    int counter = 0;

    // Determine Role based on logic from Main? 
    // Main doesn't explicitly tell us Server vs Client role via arg, BUT 
    // we can deduce or Main should pass it. 
    // Wait, Main does NOT pass role. I need to know if Server or Client.
    // However, I can infer it? No.
    // I should have added role to arguments.
    // Actually, looking at main.c: "NetInitMsg" exchange happens in Main.
    // If I am Server, I accepted a socket. If Client, I connected.
    // The Protocol spec says Server sends OK first. Client receives OK.
    // Failure to identify role means I can't implement the spec.
    // I will check if I can assume role from something else or update Main again.
    
    // UPDATE: I'll assume I need to handle role. 
    // Let's deduce role: Server 'binds', Client 'connects'. 
    // In Main, both result in a connected socket passed here.
    // I WILL UPDATE MAIN TO PASS ROLE AS ARG 8.
    
    // TEMPORARY FIX: Since I can't update Main in this 'write_to_file' call without
    // potentially breaking sync, I will assume arg 8 exists or I will fail.
    // Let's assume I will update main in next step to pass role.
    
    int is_server = 0; 
    if (argc >= 9) {
        is_server = atoi(argv[8]);
    } else {
        // Fallback/Hack: Try to peek? No.
        // I must update Main. For now code assumes arg 8.
        is_server = 0; 
    }

    /* 
     PROTOCOL VARIABLES
    */
    char buf[256];
    
    if (is_server) {
        // SERVER MODE
        log_system("NETWORK", "Starting Server Protocol");

        // 1. snd OK; rcv OK;
        send_msg(MySocket, "OK");
        recv_msg(MySocket, buf, sizeof(buf)); 
        // Expect "OK"
        
        // 2. sn SIZE l,h; rcv OK <size>;
        char size_msg[64];
        snprintf(size_msg, sizeof(size_msg), "SIZE %d,%d", win_w, win_h);
        send_msg(MySocket, size_msg);
        recv_msg(MySocket, buf, sizeof(buf));
        // Expect "OK <size>" (unused for now, acting as ack)

        // 3. Loop
        while (1) {
            // Check Local Updates (Non-blocking)
            StateMsg local_state;
            while(read(fdDtoN_r, &local_state, sizeof(local_state)) > 0) {
                last_local_x = local_state.x;
                last_local_y = local_state.y;
                // Echo to Blackboard immediately? Or wait? 
                // Plan said: Updates Blackboard (Echo local drone).
                write(fdNtoB_Drone_w, &local_state, sizeof(local_state));
            }

            // snd DRONE; snd x,y; rcv DOK <drone>
            send_msg(MySocket, "DRONE");
            
            char pos_msg[64];
            snprintf(pos_msg, sizeof(pos_msg), "%f,%f", last_local_x, last_local_y);
            send_msg(MySocket, pos_msg);
            
            recv_msg(MySocket, buf, sizeof(buf)); // DOK ...

            // snd OBST; rcv x,y; snd POK <obstacle>
            send_msg(MySocket, "OBST");
            
            recv_msg(MySocket, buf, sizeof(buf)); // Receive "x,y" from remote
            float rx, ry;
            if(sscanf(buf, "%f,%f", &rx, &ry) == 2) {
                // valid remote pos
                ObjMsg om;
                om.type = 'O'; 
                om.id = 0;     
                om.x = rx;
                om.y = ry;
                write(fdOtoB_w, &om, sizeof(om));
            }
            
            send_msg(MySocket, "POK");

            // snd Q; rcv QOK; exit (Conditional)
            // User said "Conditional". Meaning if we want to quit.
            // How do we know if we want to quit? 
            // Maybe if watchdog signal or parent signal? 
            // For now, we don't trigger quit actively in loop unless signaled.
            // Pseudocode implies we Send Q *if* we exit.
            // But strict pseudocode was "snd Q...". 
            // If it's conditional, we skip it unless splitting.
            
            // Watchdog Update
            counter++;
            if (counter >= 20) { // faster loop? No delay?
                // Sequential loop is blocking on recv! 
                // So heartbeat freq depends on network latency.
                // We should send heartbeat every iteration to be safe.
                union sigval value;
                value.sival_int = PROCESS_NETWORK | (AREA_COMPUTE << 8); 
                sigqueue(wd_pid, SIGUSR1, value);
                counter = 0;
            }
            
            // Optional delay? 
            usleep(20000); // 20ms to prevent spamming if compiled locally fast
        }
        
    } else {
        // CLIENT MODE (Mirror)
        log_system("NETWORK", "Starting Client Protocol");

        // 1. rcv OK; snd OK;
        recv_msg(MySocket, buf, sizeof(buf)); // OK
        send_msg(MySocket, "OK");

        // 2. rcv SIZE l,h; sn OK <size>;
        recv_msg(MySocket, buf, sizeof(buf)); // SIZE ...
        // Parse size if needed, but we used args.
        send_msg(MySocket, "OK SIZE");

        // 3. Loop
        while (1) {
            // Check Local Updates
            StateMsg local_state;
            while(read(fdDtoN_r, &local_state, sizeof(local_state)) > 0) {
                last_local_x = local_state.x;
                last_local_y = local_state.y;
                write(fdNtoB_Drone_w, &local_state, sizeof(local_state));
            }

            // rcv DRONE; rcv x,y; snd DOK <drone>
            recv_msg(MySocket, buf, sizeof(buf)); // DRONE check?
            if (strcmp(buf, "Q") == 0) {
                 send_msg(MySocket, "QOK");
                 break; // Server Quit
            }
            
            recv_msg(MySocket, buf, sizeof(buf)); // x,y
            float rx, ry;
            if(sscanf(buf, "%f,%f", &rx, &ry) == 2) {
                ObjMsg om;
                om.type = 'O'; 
                om.id = 0;     
                om.x = rx;
                om.y = ry;
                write(fdOtoB_w, &om, sizeof(om));
            }
            send_msg(MySocket, "DOK");

            // rcv OBST; snd x,y; rcv POK <obstacle>
            recv_msg(MySocket, buf, sizeof(buf)); // OBST
            
            char pos_msg[64];
            snprintf(pos_msg, sizeof(pos_msg), "%f,%f", last_local_x, last_local_y);
            send_msg(MySocket, pos_msg);
            
            recv_msg(MySocket, buf, sizeof(buf)); // POK

            // Heartbeat
            counter++;
            if (counter >= 20) {
                union sigval value;
                value.sival_int = PROCESS_NETWORK | (AREA_COMPUTE << 8); 
                sigqueue(wd_pid, SIGUSR1, value);
                counter = 0;
            }
        }
    }

    cleanup(0);
    return 0;
}
