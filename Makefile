CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -pedantic
SRC      = firmware/core/crank_decoder.c firmware/core/timing_map.c
INC      = -Ifirmware/core
BIN      = build/test_bench

.PHONY: all test plots clean

all: test

$(BIN): firmware/tests/test_bench.c $(SRC) firmware/core/crank_decoder.h firmware/core/timing_map.h
	@mkdir -p build results/data results/plots
	$(CC) $(CFLAGS) $(INC) firmware/tests/test_bench.c $(SRC) -lm -o $(BIN)

test: $(BIN)
	./$(BIN) | tee results/test_report.txt

plots: test
	python3 simulation/make_plots.py

clean:
	rm -rf build
