/*#include <stdio.h>
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
        close(MySocket);
    }
    log_system("NETWORK", "exiting");
    exit(0);
}

// Helpers for Protocol
void send_msg(int sock, const char *msg) {
    char logbuf[128];
    snprintf(logbuf, sizeof(logbuf), "sending '%s'", msg);
    log_system("NETWORK", logbuf);
    ssize_t n = send(sock, msg, strlen(msg) + 1, 0); 
    if (n < 0) log_system("NETWORK", "send failed");
}

void recv_msg(int sock, char *buf, size_t size) {
    memset(buf, 0, size);
    ssize_t n = recv(sock, buf, size, 0);
    if (n < 0) {
        log_system("NETWORK", "recv failed");
        // perror("recv");
    } else if (n == 0) {
        log_system("NETWORK", "recv returned 0 (connection closed)");
    } else {
        char msg[128];
        snprintf(msg, sizeof(msg), "recv got %ld bytes: '%s'", n, buf);
        log_system("NETWORK", msg);
    }
}

int main(int argc, char **argv) {
    if (argc < 8) {
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
    
    int is_server = 0; 
    if (argc >= 9) {
        is_server = atoi(argv[8]);
    } else {
        is_server = 0; 
    }

    
     PROTOCOL VARIABLES
    
    int fdTtoN_r = -1;
    int fdTtoB_w = -1;
    if (argc >= 11) {
        fdTtoN_r = atoi(argv[9]);
        fdTtoB_w = atoi(argv[10]);
    }
    
    // Non-blocking read for Targets (Server only)
    if (is_server && fdTtoN_r != -1) {
         int fl = fcntl(fdTtoN_r, F_GETFL, 0);
         fcntl(fdTtoN_r, F_SETFL, fl | O_NONBLOCK);
    }
    
    
     PROTOCOL VARIABLES
    
    char buf[256];
    
    if (is_server) {
        // SERVER MODE
        log_system("NETWORK", "Starting Server Protocol");

        // 1. snd ok; rcv ook;
        send_msg(MySocket, "ok");
        recv_msg(MySocket, buf, sizeof(buf)); // ook

        if(strcmp(buf, "ook") == 0){
            log_system("NETWORK", "OK");
        }

        // 2. snd size l,h; rcv sok <size>
        char size_msg[64];
        snprintf(size_msg, sizeof(size_msg), "size %d %d", win_w, win_h);
        send_msg(MySocket, size_msg);
        recv_msg(MySocket, buf, sizeof(buf)); // sok <size>
        
        // 3. Loop
        while (1) {
            // Check Local Updates
            StateMsg local_state;
            while(read(fdDtoN_r, &local_state, sizeof(local_state)) > 0) {
                last_local_x = local_state.x;
                last_local_y = local_state.y;
                write(fdNtoB_Drone_w, &local_state, sizeof(local_state));
            }
            
            // Check Local Targets (Server -> Client)
            if (fdTtoN_r != -1) {
                ObjMsg target_msg;
                // Read all pending targets
                while(read(fdTtoN_r, &target_msg, sizeof(target_msg)) > 0) {
                     // 1. Write to local blackboard
                     write(fdTtoB_w, &target_msg, sizeof(target_msg));
                     
                     // 2. Send to Client: "target id,x,y"
                     send_msg(MySocket, "target");
                     
                     char t_str[64];
                     snprintf(t_str, sizeof(t_str), "%d %f %f", target_msg.id, target_msg.x, target_msg.y);
                     send_msg(MySocket, t_str);
                     
                     recv_msg(MySocket, buf, sizeof(buf)); // tok
                }
            }

            // LOOP: snd drone; snd x,y; rcv dok <drone>
            send_msg(MySocket, "drone");

            log_system("NETWORK", "CI SIAMO 1");
            
            char pos_msg[64];
            snprintf(pos_msg, sizeof(pos_msg), "%f %f", last_local_x, last_local_y);
            send_msg(MySocket, pos_msg);
            
            log_system("NETWORK", "CI SIAMO 2");
            recv_msg(MySocket, buf, sizeof(buf)); 
            // Expect "dok ...". 

            log_system("NETWORK", "CI SIAMO 3");
            
            // LOOP: snd obst; rcv x,y; snd pok <obstacle>
            send_msg(MySocket, "obst");
            
            log_system("NETWORK", "CI SIAMO 4");
            recv_msg(MySocket, buf, sizeof(buf)); // Receive "x,y" from remote

            log_system("NETWORK", "CI SIAMO 5");

            float rx, ry;
            if(sscanf(buf, "%f %f", &rx, &ry) == 2) {
                ObjMsg om;
                om.type = 'D'; 
                om.id = 0;     
                om.x = rx;
                om.y = ry;
                write(fdOtoB_w, &om, sizeof(om));
            }
            
            // Send Acknowledgement "pok <obstacle>"
            char pok_msg[64];
            snprintf(pok_msg, sizeof(pok_msg), "pok %f %f", rx, ry);
            send_msg(MySocket, pok_msg);

            log_system("NETWORK", "CI SIAMO 6");
            
            // Watchdog Update
            counter++;
            if (counter >= 20) {
                union sigval value;
                value.sival_int = PROCESS_NETWORK | (AREA_COMPUTE << 8); 
                if (wd_pid > 0) {
                    sigqueue(wd_pid, SIGUSR1, value);
                }
                counter = 0;
            }
            
            usleep(20000); 
        }
        
    } else {
        // CLIENT MODE
        log_system("NETWORK", "Starting Client Protocol");

        // 1. rcv ok; snd ook
        recv_msg(MySocket, buf, sizeof(buf)); // ok

        log_system("NETWORK", "CI SIAMO 1");
        send_msg(MySocket, "ook");

        log_system("NETWORK", "CI SIAMO 2");
        // 2. rcv size l,h; snd sok <size>
        recv_msg(MySocket, buf, sizeof(buf)); // size 80,24

        log_system("NETWORK", "CI SIAMO 3");
        
        // We can parse dimensions if needed, but we act as ACK.
        char sok_msg[64];
        snprintf(sok_msg, sizeof(sok_msg), "sok %s", buf + 5); // echo back size or part of it
        send_msg(MySocket, sok_msg);

        log_system("NETWORK", "CI SIAMO 4");

        // 3. Loop with Switch
        while (1) {
            // Check Local Updates
            StateMsg local_state;
            while(read(fdDtoN_r, &local_state, sizeof(local_state)) > 0) {
                last_local_x = local_state.x;
                last_local_y = local_state.y;
                write(fdNtoB_Drone_w, &local_state, sizeof(local_state));
            }
            
            // rcv x (command)
            recv_msg(MySocket, buf, sizeof(buf));
            
            if (strcmp(buf, "q") == 0) {
                 send_msg(MySocket, "qok");
                 break; 
            }
            else if (strcmp(buf, "target") == 0) {
                 // rcv id,x,y; snd tok
                 recv_msg(MySocket, buf, sizeof(buf));
                 
                 int tid; 
                 float tx, ty;
                 if (sscanf(buf, "%d %f %f", &tid, &tx, &ty) == 3) {
                      ObjMsg om;
                      om.type = 'T';
                      om.id = tid;
                      om.x = tx;
                      om.y = ty;
                      write(fdTtoB_w, &om, sizeof(om));
                 }
                 
                 send_msg(MySocket, "tok");
            }
            else if (strcmp(buf, "drone") == 0) {
                // rcv x, y; snd dok <drone>
                recv_msg(MySocket, buf, sizeof(buf)); // x,y
                float rx, ry;
                // Parse remote drone pos (to show as obstacle)
                if(sscanf(buf, "%f %f", &rx, &ry) == 2) {
                    ObjMsg om;
                    om.type = 'D'; 
                    om.id = 0;     
                    om.x = rx;
                    om.y = ry;
                    write(fdOtoB_w, &om, sizeof(om));
                }
                
                char dok_msg[300];
                snprintf(dok_msg, sizeof(dok_msg), "dok %s", buf);
                send_msg(MySocket, dok_msg);
            }
            else if (strcmp(buf, "obst") == 0) {
                // snd x, y; rcv pok <obstacle>
                char pos_msg[64];
                snprintf(pos_msg, sizeof(pos_msg), "%f %f", last_local_x, last_local_y);
                send_msg(MySocket, pos_msg);
                
                recv_msg(MySocket, buf, sizeof(buf)); // pok ...
            }

            // Heartbeat
            counter++;
            if (counter >= 20) {
                union sigval value;
                value.sival_int = PROCESS_NETWORK | (AREA_COMPUTE << 8); 
                if (wd_pid > 0) {
                    sigqueue(wd_pid, SIGUSR1, value);
                }
                counter = 0;
            }
        }
    }

    cleanup(0);
    return 0;
}*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "logger.h"
#include "common.h"

int MySocket = -1;

void cleanup(int sig) {
    (void)sig;
    if (MySocket != -1) close(MySocket);
    log_system("NETWORK", "exiting");
    exit(0);
}

// --- Helpers ---

// Send null-terminated message
void send_msg(int sock, const char *msg) {
    ssize_t n = send(sock, msg, strlen(msg) + 1, 0); // include \0
    char logbuf[256];
    snprintf(logbuf, sizeof(logbuf), "sending '%s' (%ld bytes)", msg, n);
    log_system("NETWORK", logbuf);
    if (n < 0) log_system("NETWORK", "send failed");
}

// Receive full null-terminated message
int recv_full_msg(int sock, char *buf, size_t size) {
    size_t pos = 0;
    while (pos < size - 1) {
        ssize_t n = recv(sock, buf + pos, 1, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            log_system("NETWORK", "recv failed");
            return -1;
        } else if (n == 0) {
            log_system("NETWORK", "connection closed");
            return 0;
        } else {
            if (buf[pos] == '\0') break;
            pos++;
        }
    }
    buf[pos] = '\0';
    char logbuf[256];
    snprintf(logbuf, sizeof(logbuf), "recv got '%s'", buf);
    log_system("NETWORK", logbuf);
    return 1;
}

// --- Main ---

int main(int argc, char **argv) {
    if (argc < 8) {
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
    int is_server = (argc >= 9) ? atoi(argv[8]) : 0;

    int fdTtoN_r = -1;
    int fdTtoB_w = -1;
    if (argc >= 11) {
        fdTtoN_r = atoi(argv[9]);
        fdTtoB_w = atoi(argv[10]);
    }

    // Non-blocking pipes
    fcntl(fdDtoN_r, F_SETFL, fcntl(fdDtoN_r, F_GETFL, 0) | O_NONBLOCK);
    fcntl(fdOtoB_w, F_SETFL, fcntl(fdOtoB_w, F_GETFL, 0) | O_NONBLOCK);
    if (fdTtoN_r != -1)
        fcntl(fdTtoN_r, F_SETFL, fcntl(fdTtoN_r, F_GETFL, 0) | O_NONBLOCK);

    float last_local_x = 0.0f, last_local_y = 0.0f;
    char buf[256];

    if (is_server) {
        log_system("NETWORK", "Starting Server Protocol");

        // --- HANDSHAKE ---
        send_msg(MySocket, "ok");
        if (recv_full_msg(MySocket, buf, sizeof(buf)) <= 0 || strcmp(buf, "ook") != 0) {
            log_system("NETWORK", "Handshake failed");
            cleanup(0);
        }
        log_system("NETWORK", "Handshake OOK received");

        char size_msg[64];
        snprintf(size_msg, sizeof(size_msg), "size %d %d", win_w, win_h);
        send_msg(MySocket, size_msg);
        recv_full_msg(MySocket, buf, sizeof(buf));
        log_system("NETWORK", "Handshake size confirmed");

        // --- MAIN LOOP ---
        while (1) {
            // Forward local drone state
            StateMsg local_state;
            while (read(fdDtoN_r, &local_state, sizeof(local_state)) > 0) {
                last_local_x = local_state.x;
                last_local_y = local_state.y;
                write(fdNtoB_Drone_w, &local_state, sizeof(local_state));
                log_system("NETWORK", "Forwarded drone state to blackboard");
            }

            // Forward targets
            if (fdTtoN_r != -1) {
                ObjMsg target_msg;
                while (read(fdTtoN_r, &target_msg, sizeof(target_msg)) > 0) {
                    write(fdTtoB_w, &target_msg, sizeof(target_msg));
                    send_msg(MySocket, "target");
                    char t_str[64];
                    snprintf(t_str, sizeof(t_str), "%d %f %f", target_msg.id, target_msg.x, target_msg.y);
                    send_msg(MySocket, t_str);
                    recv_full_msg(MySocket, buf, sizeof(buf)); // expect tok
                    log_system("NETWORK", "Target sent and ACK received");
                }
            }

            // Send drone position
            send_msg(MySocket, "drone");
            char pos_msg[64];
            snprintf(pos_msg, sizeof(pos_msg), "%f %f", last_local_x, last_local_y);
            send_msg(MySocket, pos_msg);
            recv_full_msg(MySocket, buf, sizeof(buf)); // expect dok
            log_system("NETWORK", "Drone sent, dok received");

            // Request obstacles
            send_msg(MySocket, "obst");
            if (recv_full_msg(MySocket, buf, sizeof(buf)) > 0) {
                float rx=0, ry=0;
                if (sscanf(buf, "%f %f", &rx, &ry) == 2) {
                    ObjMsg om = {'D', 0, rx, ry};
                    write(fdOtoB_w, &om, sizeof(om));
                    log_system("NETWORK", "Obstacle received and forwarded");
                }
                char pok_msg[64];
                snprintf(pok_msg, sizeof(pok_msg), "pok %f %f", rx, ry);
                send_msg(MySocket, pok_msg);
            }

            usleep(20000);
        }

    } else {
        // CLIENT
        log_system("NETWORK", "Starting Client Protocol");

        // --- HANDSHAKE ---
        if (recv_full_msg(MySocket, buf, sizeof(buf)) <= 0) cleanup(0); // ok
        log_system("NETWORK", "Handshake OK received");
        send_msg(MySocket, "ook");

        if (recv_full_msg(MySocket, buf, sizeof(buf)) <= 0) cleanup(0); // size
        char sok_msg[64];
        snprintf(sok_msg, sizeof(sok_msg), "sok %s", buf + 5);
        send_msg(MySocket, sok_msg);
        log_system("NETWORK", "Handshake size ACK sent");

        // --- MAIN LOOP ---
        while (1) {
            // Forward local drone state
            StateMsg local_state;
            while (read(fdDtoN_r, &local_state, sizeof(local_state)) > 0) {
                last_local_x = local_state.x;
                last_local_y = local_state.y;
                write(fdNtoB_Drone_w, &local_state, sizeof(local_state));
                log_system("NETWORK", "Forwarded drone state to blackboard");
            }

            // Receive server command
            if (recv_full_msg(MySocket, buf, sizeof(buf)) <= 0) continue;

            if (strcmp(buf, "drone") == 0) {
                if (recv_full_msg(MySocket, buf, sizeof(buf)) <= 0) continue;
                float rx, ry;
                if (sscanf(buf, "%f %f", &rx, &ry) == 2) {
                    ObjMsg om = {'D', 0, rx, ry};
                    write(fdOtoB_w, &om, sizeof(om));
                    log_system("NETWORK", "Drone coords received and forwarded");
                    char dok_msg[64];
                    snprintf(dok_msg, sizeof(dok_msg), "dok %f %f", rx, ry);
                    send_msg(MySocket, dok_msg);
                }
            } else if (strcmp(buf, "obst") == 0) {
                char pos_msg[64];
                snprintf(pos_msg, sizeof(pos_msg), "%f %f", last_local_x, last_local_y);
                send_msg(MySocket, pos_msg);
                recv_full_msg(MySocket, buf, sizeof(buf)); // pok
                log_system("NETWORK", "Obstacle sent, pok received");
            } else if (strcmp(buf, "target") == 0) {
                if (recv_full_msg(MySocket, buf, sizeof(buf)) <= 0) continue;
                int tid; float tx, ty;
                if (sscanf(buf, "%d %f %f", &tid, &tx, &ty) == 3) {
                    ObjMsg om = {'T', tid, tx, ty};
                    write(fdTtoB_w, &om, sizeof(om));
                    log_system("NETWORK", "Target received and forwarded");
                }
                send_msg(MySocket, "tok");
            }

            usleep(20000);
        }
    }

    cleanup(0);
    return 0;
}




