#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static FILE *logf = NULL;

// Timestamp in millisecondi
long log_now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

// Apre il logfile
int log_init(const char *filename) {
    logf = fopen(filename, "w");
    if (!logf) return 0;
    fprintf(logf, "==== LOG START ====
");
    fflush(logf);
    return 1;
}

// Scrive una riga nel log
void log_write(const char *msg) {
    if (!logf) return;
    fprintf(logf, "[%ld ms] %s
", log_now_ms(), msg);
    fflush(logf);
}

// Formato avanzato: msg + due numeri
void log_write2(const char *msg, float a, float b) {
    if (!logf) return;
    fprintf(logf, "[%ld ms] %s %.3f %.3f
", log_now_ms(), msg, a, b);
    fflush(logf);
}

// Chiude il log
void log_close() {
    if (!logf) return;
    fprintf(logf, "==== LOG END ====
");
    fclose(logf);
    logf = NULL;
}
