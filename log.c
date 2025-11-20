#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <linux/time.h>

// This file implements a very simple logging module.
// It lets us write timestamped messages to a file,
// useful for debugging force updates and drone physics.

static FILE *logf = NULL;   // pointer to opened log file

// Get current time in milliseconds
long log_now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

// Open the log file. Returns 1 on success.
int log_init(const char *filename) {
    logf = fopen(filename, "w");
    if (!logf) return 0;    // could not open file

    fprintf(logf, "==== LOG START ====\n");
    fflush(logf);
    return 1;
}

// Write a message with timestamp
void log_write(const char *msg) {
    if (!logf) return;   // safety: log not opened

    fprintf(logf, "[%ld ms] %s\n", log_now_ms(), msg);
    fflush(logf);
}

// Write a message plus two float values (for forces or velocities)
void log_write2(const char *msg, float a, float b) {
    if (!logf) return;

    fprintf(logf, "[%ld ms] %s %.3f %.3f\n", log_now_ms(), msg, a, b);
    fflush(logf);
}

// Close the logfile
void log_close() {
    if (!logf) return;

    fprintf(logf, "==== LOG END ====\n");
    fclose(logf);
    logf = NULL;
}
