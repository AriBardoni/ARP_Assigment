#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include "logger.h"
#include "common.h"

// Check interval in seconds
#define CHECK_INTERVAL 1
// Timeout threshold (if no signal for this long, consider dead)
#define TIMEOUT_THRESHOLD 10

int main(int argc, char **argv) {
    (void)argc; (void)argv; // Unused

    logger_init("../logs");
    log_process_register("WATCHDOG", getpid());
    log_system("WATCHDOG", "started");

    // Block SIGUSR1 so we can wait for it synchronously
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    if (sigprocmask(SIG_BLOCK, &set, NULL) == -1) {
        log_system("WATCHDOG", "sigprocmask error");
        return 1;
    }

    // Array to store the last time a signal was received from each process
    time_t last_seen[PROCESS_COUNT];
    int last_area[PROCESS_COUNT];
    time_t now = time(NULL);

    for (int i = 0; i < PROCESS_COUNT; i++) {
        last_seen[i] = now;
        last_area[i] = AREA_INIT;
    }

    const char *proc_names[PROCESS_COUNT] = {
        "Drone", "Input", "Blackboard", "Obstacles", "Targets", "Network"
    };

    printf("Watchdog started. Monitoring %d processes via signals.\n", PROCESS_COUNT);
    log_message("LogFile1", "Watchdog", "Watchdog started");

    while (1) {
        struct timespec timeout;
        timeout.tv_sec = CHECK_INTERVAL;
        timeout.tv_nsec = 0;

        siginfo_t info;
        int sig = sigtimedwait(&set, &info, &timeout);

        if (sig > 0 && sig == SIGUSR1) {
            int payload = info.si_value.sival_int;
            int proc_id = payload & 0xFF;          // process ID
            int area_id = (payload >> 8) & 0xFF;   // code area

            if (proc_id >= 0 && proc_id < PROCESS_COUNT) {
                last_seen[proc_id] = time(NULL);
                last_area[proc_id] = area_id;
                
                // Log to file
                char msg[256];
                snprintf(msg, sizeof(msg), "Process %s executing Area %d", proc_names[proc_id], area_id);
                log_message("LogFile1", "Watchdog", msg);
                printf("Watchdog: %s\n", msg);
            } else {
                log_system("WATCHDOG", "received invalid process id");
            }
        } else {
            if (errno != EAGAIN && errno != EINTR) {
                log_system("WATCHDOG", "sigtimedwait error");
            }
        }

        // Check for timeouts
        now = time(NULL);

        for (int i = 0; i < PROCESS_COUNT; i++) {
            if (now - last_seen[i] > TIMEOUT_THRESHOLD) {
                char msg[256];
                snprintf(msg, sizeof(msg), "ALERT: Process %d is unresponsive! (Last Area: %d)", i, last_area[i]);
                log_message("LogFile1", "Watchdog", msg);
                
                fprintf(stderr, "Watchdog ALERT: Process %d is unresponsive! (Last seen %ld seconds ago, Last Area: %d)\n", 
                        i, now - last_seen[i], last_area[i]);
                
                fprintf(stderr, "Watchdog: Signaling main to shutdown...\n");
                fflush(stderr);

                log_system("WATCHDOG", "ALERT: process unresponsive");
                log_system("WATCHDOG", "signaling main for shutdown");

                kill(getppid(), SIGUSR1);

                // Wait to be terminated by main
                while (1)
                    sleep(1);
            }
        }
    }

    return 0;
}
