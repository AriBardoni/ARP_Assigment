CC      = gcc
CFLAGS  = -Wall -Wextra -std=c11 -g -D_GNU_SOURCE -Iinclude

LDLIBS_COMMON   = -lm
LDLIBS_NCURSES  = -lncursesw

TARGETS = main drone input blackboard obstacles targets watchdog

BIN_DIR   = bin
SRC_DIR   = src
OBJ_DIR   = build

# Regola di default

all: $(TARGETS)

# Regola generica .o

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Eseguibili

main: $(OBJ_DIR)/main.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

drone: $(OBJ_DIR)/drone.o $(OBJ_DIR)/log.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

input: $(OBJ_DIR)/input.o $(OBJ_DIR)/log.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON) $(LDLIBS_NCURSES)

blackboard: $(OBJ_DIR)/blackboard.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON) $(LDLIBS_NCURSES)

obstacles: $(OBJ_DIR)/obstacles.o $(OBJ_DIR)/log.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

targets: $(OBJ_DIR)/targets.o $(OBJ_DIR)/log.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

watchdog: $(OBJ_DIR)/watchdog.o
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^ $(LDLIBS_COMMON)

# Utility

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR) *.log
