#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/file.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>

static char LOG_DIR[512] = "logs";

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

static void get_log_path(char *buf, size_t size, const char *filename) {
    snprintf(buf, size, "%s/%s", LOG_DIR, filename);
}

/* -------- public API -------- */

void logger_init(const char *path) {
    if (path) {
        snprintf(LOG_DIR, sizeof(LOG_DIR), "%s", path);
    }
    struct stat st = {0};
    if (stat(LOG_DIR, &st) == -1) {
        mkdir(LOG_DIR, 0700);
    }
}

void log_process_register(const char *proc, pid_t pid) {
    char path[512];
    get_log_path(path, sizeof(path), "processes.log");
    
    FILE *f = fopen(path, "a");
    if (!f) return;

    locked_write(f, "[%ld] %s %d\n", now_ms(), proc, pid);
    fclose(f);
}

void log_system(const char *proc, const char *fmt, ...) {
    char path[512];
    get_log_path(path, sizeof(path), "system.log");
    
    FILE *f = fopen(path, "a");
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
    char path[512];
    get_log_path(path, sizeof(path), "watchdog.log");

    FILE *f = fopen(path, "a");
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

void log_message(const char *logfile, const char *proc_name, const char *msg) {
    char path[512];
    get_log_path(path, sizeof(path), logfile);

    FILE *f = fopen(path, "a");
    if (!f) return;

    locked_write(f, "[%ld] %s %s\n", now_ms(), proc_name, msg);
    fclose(f);
}
