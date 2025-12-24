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
#define TIMEOUT_THRESHOLD 10

int main(int argc, char **argv) {
    (void)argc; (void)argv; // Unused

    // Block SIGUSR1 so we can wait for it synchronously
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    if (sigprocmask(SIG_BLOCK, &set, NULL) == -1) {
        perror("sigprocmask");
        return 1;
    }

    // Array to store the last time a signal was received from each process
    time_t last_seen[PROCESS_COUNT];
    int last_area[PROCESS_COUNT]; // Store last known code area
    time_t now = time(NULL);
    
    for (int i = 0; i < PROCESS_COUNT; i++) {
        last_seen[i] = now;
        last_area[i] = AREA_INIT;
    }

    const char *proc_names[PROCESS_COUNT] = {
        "Drone", "Input", "Blackboard", "Obstacles", "Targets"
    };

    printf("Watchdog started. Monitoring %d processes via signals.\n", PROCESS_COUNT);
    log_message("LogFile1", "Watchdog", "Watchdog started");

    while (1) {
        struct timespec timeout;
        timeout.tv_sec = CHECK_INTERVAL;
        timeout.tv_nsec = 0;

        siginfo_t info;
        int sig = sigtimedwait(&set, &info, &timeout);

        if (sig > 0) {
            // Signal received
            if (sig == SIGUSR1) {
                int payload = info.si_value.sival_int;
                int proc_id = payload & 0xFF; // Lower 8 bits for Process ID
                int area_id = (payload >> 8) & 0xFF; // Next 8 bits for Code Area

                if (proc_id >= 0 && proc_id < PROCESS_COUNT) {
                    last_seen[proc_id] = time(NULL);
                    last_area[proc_id] = area_id;
                    
                    // Log to file
                    char msg[256];
                    snprintf(msg, sizeof(msg), "Process %s executing Area %d", proc_names[proc_id], area_id);
                    log_message("LogFile1", "Watchdog", msg);

                    // Print to terminal
                    printf("Watchdog: Received signal from %s (ID %d) | Area: %d\n", proc_names[proc_id], proc_id, area_id);
                    fflush(stdout);
                } else {
                    fprintf(stderr, "Watchdog: Received invalid process ID %d from PID %d\n", proc_id, info.si_pid);
                }
            }
        } else {
            if (errno != EAGAIN) {
                perror("sigtimedwait");
                // If interrupted or other error, continue?
                // return 1; 
            }
        }

        // Check for timeouts
        now = time(NULL);
        // printf("Watchdog: Checking timeouts... (%ld seconds elapsed)\n", (long)CHECK_INTERVAL); 
        
        for (int i = 0; i < PROCESS_COUNT; i++) {
            if (now - last_seen[i] > TIMEOUT_THRESHOLD) {
                char msg[256];
                snprintf(msg, sizeof(msg), "ALERT: Process %d is unresponsive! (Last Area: %d)", i, last_area[i]);
                log_message("LogFile1", "Watchdog", msg);
                
                fprintf(stderr, "Watchdog ALERT: Process %d is unresponsive! (Last seen %ld seconds ago, Last Area: %d)\n", 
                        i, now - last_seen[i], last_area[i]);
                
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
