CC      = gcc
CFLAGS  = -Wall -Wextra -std=c11 -g -D_GNU_SOURCE -Iinclude

LDLIBS_COMMON   = -lm
LDLIBS_NCURSES  = -lncursesw

TARGETS = main drone input blackboard obstacles targets watchdog network

BIN_DIR   = bin
SRC_DIR   = src
OBJ_DIR   = build
LOG_DIR   = logs

# Regola di default
all: $(TARGETS)

# Regola generica .o
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Eseguibili
main: $(OBJ_DIR)/main.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

drone: $(OBJ_DIR)/drone.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

input: $(OBJ_DIR)/input.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON) $(LDLIBS_NCURSES)

blackboard: $(OBJ_DIR)/blackboard.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON) $(LDLIBS_NCURSES)

obstacles: $(OBJ_DIR)/obstacles.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

targets: $(OBJ_DIR)/targets.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

watchdog: $(OBJ_DIR)/watchdog.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

network: $(OBJ_DIR)/network.o $(OBJ_DIR)/logger.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

# --- PULIZIA ---

# Pulisce solo i file compilati e i log
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR) $(LOG_DIR)/*

# Pulisce TUTTO: file, processi appesi e SEMAFORI BLOCCATI (Usa questo se si impalla)
flush: clean
	@echo "--- KILLING ZOMBIE PROCESSES ---"
	-killall -9 main drone input blackboard obstacles targets watchdog network 2>/dev/null || true
	@echo "--- CLEANING SEMAPHORES & SHM ---"
	-ipcs -s | awk -v user=$(USER) '$$3 == user {print $$2}' | xargs -r ipcrm sem
	-ipcs -m | awk -v user=$(USER) '$$3 == user {print $$2}' | xargs -r ipcrm shm
	@echo "--- SYSTEM CLEANED ---"