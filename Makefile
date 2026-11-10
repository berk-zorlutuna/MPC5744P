# Host build of the portable firmware parts + tests.
# The target (MPC5744P) build is done in S32 Design Studio, see README.md.

CC      ?= gcc
BUILD   := build
INC     := -Iconfig -Isrc/bsp -Isrc/control -Isrc/app -Isrc/safety -Isrc/comm -Itests
CFLAGS  := -std=c99 -O2 -g -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
           -Wdouble-promotion -DHOST_BUILD $(INC)
LDLIBS  := -lm

SRC := src/control/pi.c src/control/svm.c src/control/pll.c src/control/foc.c \
       src/app/pedal.c src/app/torque_mgr.c src/app/vehicle_sm.c src/app/app.c \
       src/safety/diag.c src/comm/can_msgs.c \
       tests/bsp_stub.c tests/harness.c tests/sim_motor.c tests/vtest.c \
       tests/test_control.c tests/test_vehicle.c tests/test_integration.c

.PHONY: all test report hw-report schematics trace clean

all: test

$(BUILD)/tests: $(SRC) $(wildcard config/*.h src/*/*.h tests/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDLIBS)

test: $(BUILD)/tests
	./$(BUILD)/tests

# Software verification results report (docs/04_software_verification_results.md)
REV := $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)$(shell git diff --quiet HEAD -- src config tests 2>/dev/null || echo -modified)
report: $(BUILD)/tests
	./$(BUILD)/tests --report docs/04_software_verification_results.md --rev "$(REV)"

# Hardware design analysis + results report (docs/08_hardware_verification_results.md)
HWREV := $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)$(shell git diff --quiet HEAD -- config tools src 2>/dev/null || echo -modified)
hw-report:
	python3 tools/hw_analysis.py --report docs/08_hardware_verification_results.md --rev "$(HWREV)"

# Concept schematics (docs/hw/*.svg) and BOM; needs: pip install schemdraw==0.19
schematics:
	python3 tools/draw_schematics.py

# Traceability consistency check across the requirement and verification documents
trace:
	sh tools/check_trace.sh

clean:
	rm -rf $(BUILD)
