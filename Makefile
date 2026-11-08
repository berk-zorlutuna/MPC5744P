# Host build of the portable firmware parts + tests.
# The target (MPC5744P) build is done in S32 Design Studio, see README.md.

CC      ?= gcc
BUILD   := build
INC     := -Iconfig -Isrc/bsp -Isrc/control -Isrc/app -Isrc/safety -Isrc/comm -Itests
CFLAGS  := -std=c99 -O2 -g -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
           -Wdouble-promotion -Wno-error=double-promotion -DHOST_BUILD $(INC)
LDLIBS  := -lm

SRC := src/control/pi.c src/control/svm.c src/control/pll.c src/control/foc.c \
       src/app/pedal.c src/app/torque_mgr.c src/app/vehicle_sm.c src/app/app.c \
       src/safety/diag.c src/comm/can_msgs.c \
       tests/bsp_stub.c tests/test_main.c

.PHONY: all test clean

all: test

$(BUILD)/tests: $(SRC) $(wildcard config/*.h src/*/*.h tests/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDLIBS)

test: $(BUILD)/tests
	./$(BUILD)/tests

clean:
	rm -rf $(BUILD)
