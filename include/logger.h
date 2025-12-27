#pragma once

#include <sys/types.h>

void logger_init(void);
void logger_close(void);
void log_process_register(const char *process_name, pid_t pid);
void log_system(const char *process_name, const char *fmt, ...);
void log_watchdog(const char *fmt, ...);
