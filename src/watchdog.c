#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <time.h>
#include <errno.h>
#include "common.h"
#include <signal.h>

// Check interval in seconds
#define CHECK_INTERVAL 1
// Timeout threshold (if no signal for this long, consider dead)
#define TIMEOUT_THRESHOLD 3

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "watchdog: missing pipe fd\n");
        return 1;
    }

    int pipe_fd = atoi(argv[1]);
    
    // Array to store the last time a signal was received from each process
    time_t last_seen[PROCESS_COUNT];
    time_t now = time(NULL);
    
    for (int i = 0; i < PROCESS_COUNT; i++) {
        last_seen[i] = now;
    }

    printf("Watchdog started. Monitoring %d processes.\n", PROCESS_COUNT);

    while (1) {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(pipe_fd, &set);

        struct timeval timeout;
        timeout.tv_sec = CHECK_INTERVAL;
        timeout.tv_usec = 0;

        int ret = select(pipe_fd + 1, &set, NULL, NULL, &timeout);

        if (ret > 0) {
            if (FD_ISSET(pipe_fd, &set)) {
                int proc_id;
                ssize_t n = read(pipe_fd, &proc_id, sizeof(int));
                if (n == sizeof(int)) {
                    if (proc_id >= 0 && proc_id < PROCESS_COUNT) {
                        last_seen[proc_id] = time(NULL);
                        // printf("Watchdog: Received signal from process %d\n", proc_id);
                    } else {
                        fprintf(stderr, "Watchdog: Received invalid process ID %d\n", proc_id);
                    }
                } else if (n == 0) {
                    fprintf(stderr, "Watchdog: Pipe closed. Exiting.\n");
                    return 0;
                }
            }
        } else if (ret == -1) {
            if (errno != EINTR) {
                perror("watchdog select");
                return 1;
            }
        }

        // Check for timeouts
        now = time(NULL);
        printf("Watchdog: Waiting... (%ld seconds elapsed)\n", (long)CHECK_INTERVAL); 
        
        for (int i = 0; i < PROCESS_COUNT; i++) {
            if (now - last_seen[i] > TIMEOUT_THRESHOLD) {
                fprintf(stderr, "Watchdog ALERT: Process %d is unresponsive! (Last seen %ld seconds ago)\n", 
                        i, now - last_seen[i]);
                
                fprintf(stderr, "Watchdog: Signaling main to shutdown...\n");
                fflush(stderr);
                kill(getppid(), SIGUSR1);
                // Wait to be killed by main
                while(1) sleep(1);
            }
        }
    }

    return 0;
}
