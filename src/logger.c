#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/file.h>
#include <unistd.h>
#include <stdarg.h>

#define LOG_DIR "logs"
#define SYSTEM_LOG   LOG_DIR "/system.log"
#define WATCHDOG_LOG LOG_DIR "/watchdog.log"
#define PROCESS_LOG  LOG_DIR "/processes.log"

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void locked_write(FILE *f, const char *fmt, ...) {
    va_list ap;
    flock(fileno(f), LOCK_EX);
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);
    fflush(f);
    flock(fileno(f), LOCK_UN);
}

/* -------- public API -------- */

void logger_init(void) {
    /* nothing to do for now, files opened on demand */
}

void log_process_register(const char *proc, pid_t pid) {
    FILE *f = fopen(PROCESS_LOG, "a");
    if (!f) return;

    locked_write(f, "%s %d\n", proc, pid);
    fclose(f);
}

void log_system(const char *proc, const char *fmt, ...) {
    FILE *f = fopen(SYSTEM_LOG, "a");
    if (!f) return;

    flock(fileno(f), LOCK_EX);
    fprintf(f, "[%ld] %s ", now_ms(), proc);

    va_list ap;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);

    fprintf(f, "\n");
    fflush(f);
    flock(fileno(f), LOCK_UN);
    fclose(f);
}

void log_watchdog(const char *fmt, ...) {
    FILE *f = fopen(WATCHDOG_LOG, "a");
    if (!f) return;

    flock(fileno(f), LOCK_EX);
    fprintf(f, "[%ld] WATCHDOG ", now_ms());

    va_list ap;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);

    fprintf(f, "\n");
    fflush(f);
    flock(fileno(f), LOCK_UN);
    fclose(f);
}
