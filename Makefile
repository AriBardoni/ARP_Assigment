CC      = gcc
CFLAGS  = -Wall -Wextra -std=c11 -g -D_GNU_SOURCE
LDLIBS_COMMON   = -lm
LDLIBS_NCURSES  = -lncursesw
TARGETS = main drone input blackboard obstacles targets

all: $(TARGETS)

# ---- Regola generica per i .o ----
%.o: %.c
	$(CC) $(CFLAGS) -c $<

# ---- Eseguibili ----

main: main.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS_COMMON)

drone: drone.o log.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS_COMMON)

input: input.o log.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS_COMMON) $(LDLIBS_NCURSES)

blackboard: blackboard.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS_COMMON) $(LDLIBS_NCURSES)

obstacles: obstacles.o log.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS_COMMON)

targets: targets.o log.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS_COMMON)


# ---- Utility ----

clean:
	rm -f *.o $(TARGETS) *.log
