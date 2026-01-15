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
// Timeout threshold
#define TIMEOUT_THRESHOLD 10

int main(int argc, char **argv) {
    (void)argc; (void)argv; 

    logger_init("../logs");
    log_process_register("WATCHDOG", getpid());
    log_system("WATCHDOG", "started");

    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    if (sigprocmask(SIG_BLOCK, &set, NULL) == -1) {
        log_system("WATCHDOG", "sigprocmask error");
        return 1;
    }

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

    printf("Watchdog started. Monitoring processes...\n");

    while (1) {
        struct timespec timeout;
        timeout.tv_sec = CHECK_INTERVAL;
        timeout.tv_nsec = 0;

        siginfo_t info;
        int sig = sigtimedwait(&set, &info, &timeout);

        if (sig > 0 && sig == SIGUSR1) {
            int payload = info.si_value.sival_int;
            int proc_id = payload & 0xFF;          
            int area_id = (payload >> 8) & 0xFF;   

            if (proc_id >= 0 && proc_id < PROCESS_COUNT) {
                last_seen[proc_id] = time(NULL);
                last_area[proc_id] = area_id;
            }
        }

        // Check for timeouts
        now = time(NULL);

        for (int i = 0; i < PROCESS_COUNT; i++) {
            // --- FIX: IGNORA IL PROCESSO NETWORK ---
            // In modalità Locale, il Network non esiste.
            // In modalità Network, il Watchdog è disabilitato.
            // Quindi non dobbiamo mai controllare l'indice 5 (Network).
            if (i == 5) continue; 
            // ---------------------------------------

            if (now - last_seen[i] > TIMEOUT_THRESHOLD) {
                
                fprintf(stderr, "Watchdog ALERT: Process %s (%d) is unresponsive! (Last seen %ld seconds ago)\n", 
                        proc_names[i], i, now - last_seen[i]);
                
                fprintf(stderr, "Watchdog: Signaling main to shutdown...\n");
                
                kill(getppid(), SIGUSR1);

                while (1) sleep(1);
            }
        }
    }

    return 0;
}