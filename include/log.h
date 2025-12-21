#pragma once

int  log_init(const char *filename);
void log_write(const char *msg);
void log_write2(const char *msg, float a, float b);
void log_close(void);
