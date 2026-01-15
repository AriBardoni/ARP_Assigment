/* ======================================================================================
 * FILE: network.c
 * Protocollo: Request -> Datum -> Ack
 * CORREZIONE: StateMsg allineato con Blackboard (x,y,vx,vy)
 * ====================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdarg.h>

// --- MOCKUP STRUTTURE (Allineate con common.h della Blackboard) ---
typedef struct { 
    float x, y; 
    float vx, vy; // <--- CAMPI AGGIUNTI PER MANTENERE L'ALLINEAMENTO (16 byte)
} StateMsg;

typedef struct { char type; int id; float x, y; } ObjMsg;
// -------------------------------------------------------------

#define BUFSZ 1024
#define LOG_PATH_SC "network.log"

// Stati del Protocollo
typedef enum {
    SV_SEND_CMD_DRONE,
    SV_SEND_DATA_DRONE,
    SV_WAIT_DOK,
    SV_SEND_CMD_OBST,
    SV_WAIT_DATA_OBST,
    CL_WAIT_COMMAND,
    CL_WAIT_DRONE_DATA,
    CL_SEND_OBST_DATA,
    CL_WAIT_POK
} NetState;

static NetState net_state;
static int MySocket = -1;

// Buffer di ricezione persistente
typedef struct {
    char data[BUFSZ];
    int len;
} SocketBuffer;

static SocketBuffer sock_buf = { .len = 0 };
static float my_last_x = 0.0f;
static float my_last_y = 0.0f;

// --- LOGGING ---
void log_sc(const char *fmt, ...) {
    FILE *fp = fopen(LOG_PATH_SC, "a");
    if (!fp) return;
    va_list args;
    va_start(args, fmt);
    fprintf(fp, "[%d] ", getpid());
    vfprintf(fp, fmt, args);
    fprintf(fp, "\n");
    va_end(args);
    fclose(fp);
}

// --- NETWORK UTILS ---
void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void send_msg_strict(int sock, const char *fmt, ...) {
    char buf[BUFSZ];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf) - 2, fmt, args); 
    va_end(args);
    int len = strlen(buf);
    // log_sc("[OUT] Raw: '%s'", buf); // Commentato per ridurre spam
    if (len == 0 || buf[len-1] != '\n') {
        buf[len] = '\n';
        buf[len+1] = '\0';
        len++;
    }
    ssize_t n = write(sock, buf, len);
    if (n < 0) log_sc("[ERR] Write failed: %s", strerror(errno));
}

int init_server(int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;
    int opt = 1; 
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons(port);
    if (bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        log_sc("[NET-ERR] Bind failed: %s", strerror(errno));
        close(s);
        return -1;
    }
    listen(s, 1);
    log_sc("[NET-SRV] Listening on port %d...", port);
    struct sockaddr_in cli;
    socklen_t len = sizeof(cli);
    int client_fd = accept(s, (struct sockaddr*)&cli, &len);
    if (client_fd >= 0) {
        log_sc("[NET-SRV] Client connected from %s", inet_ntoa(cli.sin_addr));
        close(s); 
    }
    return client_fd;
}

int init_client(const char *addr, int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; 
    a.sin_port = htons(port);
    inet_pton(AF_INET, addr, &a.sin_addr);
    log_sc("[NET-CLI] Connecting to %s:%d ...", addr, port);
    while (connect(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        log_sc("[NET-CLI] Retrying connection...");
        sleep(1);
    }
    log_sc("[NET-CLI] Connected!");
    return s;
}

int read_chunk(int sock) {
    if (sock_buf.len >= BUFSZ - 1) return 0;
    ssize_t n = read(sock, sock_buf.data + sock_buf.len, BUFSZ - 1 - sock_buf.len);
    if (n > 0) {
        sock_buf.len += n;
        sock_buf.data[sock_buf.len] = '\0'; 
        return 1;
    }
    if (n == 0) return -1;
    return 0;
}

int pop_line(char *out, int max) {
    char *p = strchr(sock_buf.data, '\n');
    if (p) {
        int len = p - sock_buf.data;
        if (len >= max) len = max - 1;
        memcpy(out, sock_buf.data, len);
        out[len] = '\0'; 
        // log_sc("[IN] Parsed: '%s'", out); // Commentato per ridurre spam
        int rem = sock_buf.len - (len + 1);
        memmove(sock_buf.data, p + 1, rem);
        sock_buf.len = rem;
        sock_buf.data[sock_buf.len] = '\0';
        return 1;
    }
    return 0;
}

int read_line_blocking(int fd, char *buf, int sz) {
    int i = 0; char c;
    while (i < sz - 1) {
        if (read(fd, &c, 1) <= 0) return -1;
        if (c == '\n') break;
        buf[i++] = c;
    }
    buf[i] = '\0';
    log_sc("[HANDSHAKE] Read: '%s'", buf);
    return i;
}

void cleanup(int sig) {
    (void)sig;
    if (MySocket != -1) close(MySocket);
    log_sc("--- EXITING ---");
    exit(0);
}

// ======================================================================================
//                                      MAIN
// ======================================================================================
int main(int argc, char **argv) {
    if (argc < 8) {
        fprintf(stderr, "Usage: %s IP Port fdBtoN fdNtoB w h is_server\n", argv[0]);
        return 1;
    }

    signal(SIGINT, cleanup);
    signal(SIGTERM, cleanup);
    signal(SIGPIPE, SIG_IGN); 

    const char *ip_addr = argv[1];
    int port = atoi(argv[2]);
    int fdBtoN = atoi(argv[3]); 
    int fdNtoB = atoi(argv[4]); 
    int w = atoi(argv[5]);
    int h = atoi(argv[6]);
    int is_server = atoi(argv[7]);

    log_sc("--- STARTING NETWORK (Role: %s) ---", is_server ? "SERVER" : "CLIENT");

    if (is_server) MySocket = init_server(port);
    else MySocket = init_client(ip_addr, port);

    if (MySocket < 0) {
        log_sc("[FATAL] Connection setup failed.");
        return 1;
    }

    // HANDSHAKE
    char buf[256];
    if (is_server) {
        send_msg_strict(MySocket, "ok");
        if (read_line_blocking(MySocket, buf, sizeof(buf)) <= 0 || strcmp(buf, "ook") != 0) cleanup(0);
        send_msg_strict(MySocket, "size %d %d", w, h);
        if (read_line_blocking(MySocket, buf, sizeof(buf)) <= 0) cleanup(0); 
        net_state = SV_SEND_CMD_DRONE;
    } else {
        if (read_line_blocking(MySocket, buf, sizeof(buf)) <= 0 || strcmp(buf, "ok") != 0) cleanup(0);
        send_msg_strict(MySocket, "ook");
        if (read_line_blocking(MySocket, buf, sizeof(buf)) <= 0) cleanup(0);
        send_msg_strict(MySocket, "sok %d %d", w, h);
        net_state = CL_WAIT_COMMAND;
    }

    log_sc("[INIT] Handshake Complete. Entering Loop.");

    set_nonblocking(MySocket);
    set_nonblocking(fdBtoN);

    fd_set rset;
    struct timeval tv;
    float rx, ry;

    while (1) {
        FD_ZERO(&rset);
        FD_SET(MySocket, &rset);
        FD_SET(fdBtoN, &rset);
        int maxfd = (MySocket > fdBtoN) ? MySocket : fdBtoN;
        
        tv.tv_sec = 0; 
        tv.tv_usec = 5000; 

        if (select(maxfd + 1, &rset, NULL, NULL, &tv) < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // [IPC] Leggi input locale (Blackboard -> Network)
        if (FD_ISSET(fdBtoN, &rset)) {
            StateMsg msg;
            // Leggi e aggiorna. Ora che la dimensione è corretta (16 byte), 
            // la lettura sarà allineata e my_last_x/y saranno corretti.
            while (read(fdBtoN, &msg, sizeof(msg)) > 0) {
                my_last_x = msg.x;
                my_last_y = msg.y;
                // log_sc("[DEBUG] Got local state: %.2f %.2f", my_last_x, my_last_y);
            }
        }

        // [NET] Leggi dati remoti nel buffer
        if (FD_ISSET(MySocket, &rset)) {
            if (read_chunk(MySocket) == -1) cleanup(0);
        }

        // [LOGIC] Macchina a Stati
        int changed;
        do {
            changed = 0;
            if (is_server) {
                switch(net_state) {
                    case SV_SEND_CMD_DRONE:
                        send_msg_strict(MySocket, "drone");
                        net_state = SV_SEND_DATA_DRONE;
                        changed = 1;
                        break;
                    case SV_SEND_DATA_DRONE:
                        send_msg_strict(MySocket, "%f %f", my_last_x, my_last_y);
                        net_state = SV_WAIT_DOK;
                        break;
                    case SV_WAIT_DOK:
                        if (pop_line(buf, sizeof(buf))) {
                            if (sscanf(buf, "dok %f %f", &rx, &ry) == 2) {
                                net_state = SV_SEND_CMD_OBST;
                                changed = 1;
                            }
                        }
                        break;
                    case SV_SEND_CMD_OBST:
                        send_msg_strict(MySocket, "obst");
                        net_state = SV_WAIT_DATA_OBST;
                        break;
                    case SV_WAIT_DATA_OBST:
                        if (pop_line(buf, sizeof(buf))) {
                            if (sscanf(buf, "%f %f", &rx, &ry) == 2) {
                                ObjMsg om = { .type='D', .id=0, .x=rx, .y=ry };
                                write(fdNtoB, &om, sizeof(om));
                                send_msg_strict(MySocket, "pok %f %f", rx, ry);
                                net_state = SV_SEND_CMD_DRONE;
                                changed = 1;
                            }
                        }
                        break;
                }
            } else {
                switch(net_state) {
                    case CL_WAIT_COMMAND:
                        if (pop_line(buf, sizeof(buf))) {
                            if (strcmp(buf, "drone") == 0) {
                                net_state = CL_WAIT_DRONE_DATA;
                                changed = 1;
                            } else if (strcmp(buf, "obst") == 0) {
                                net_state = CL_SEND_OBST_DATA;
                                changed = 1;
                            } else if (strcmp(buf, "q") == 0) {
                                cleanup(0);
                            }
                        }
                        break;
                    case CL_WAIT_DRONE_DATA:
                        if (pop_line(buf, sizeof(buf))) {
                            if (sscanf(buf, "%f %f", &rx, &ry) == 2) {
                                ObjMsg om = { .type='D', .id=0, .x=rx, .y=ry };
                                write(fdNtoB, &om, sizeof(om));
                                send_msg_strict(MySocket, "dok %f %f", rx, ry);
                                net_state = CL_WAIT_COMMAND;
                            }
                        }
                        break;
                    case CL_SEND_OBST_DATA:
                        send_msg_strict(MySocket, "%f %f", my_last_x, my_last_y);
                        net_state = CL_WAIT_POK;
                        break;
                    case CL_WAIT_POK:
                        if (pop_line(buf, sizeof(buf))) {
                            if (sscanf(buf, "pok %f %f", &rx, &ry) == 2) {
                                net_state = CL_WAIT_COMMAND;
                                changed = 1;
                            }
                        }
                        break;
                }
            }
        } while(changed);
    }

    cleanup(0);
    return 0;
}