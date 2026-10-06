# Host-side tests for the PureThermal firmware. See README.md.
#
#   make            build and run all tests
#   make run T=wedge   run only tests whose name contains "wedge"
#   make mutants       check the tests still catch known past bugs
#   make clean

ROOT     := ..
BUILD    := build
# Not CC: the firmware Makefile exports CC as the ARM cross compiler.
HOST_CC  ?= gcc

# Fakes first so they shadow the STM32 HAL; real firmware headers after.
INCLUDES := -Ifakes \
            -I$(ROOT)/Inc \
            -I$(ROOT)/Middlewares/lepton_sdk/Inc \
            -I$(ROOT)/Middlewares/ST/STM32_USB_Device_Library/Class/video/Inc

SANITIZE := -fsanitize=address,undefined -fno-sanitize-recover=undefined
CFLAGS   := -g -O0 -Wall -fno-common -MMD -MP $(SANITIZE) $(INCLUDES)

# Firmware sources under test, compiled unmodified. Same language standard as
# the target build; warnings are reported but are not ours to fail on.
FW_SRCS  := $(ROOT)/Src/lepton_task.c
FW_OBJS  := $(patsubst $(ROOT)/Src/%.c,$(BUILD)/fw_%.o,$(FW_SRCS))

TEST_SRCS := sim.c test_main.c $(wildcard test_*.c)
TEST_SRCS := $(filter-out test_main.c,$(TEST_SRCS)) test_main.c
TEST_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(TEST_SRCS))

BIN := $(BUILD)/run_tests

.PHONY: all run mutants clean
all: run

run: $(BIN)
	@./$(BIN) $(T)

mutants: $(BIN)
	@python3 mutants.py $(CFLAGS)

$(BIN): $(FW_OBJS) $(TEST_OBJS)
	$(HOST_CC) $(CFLAGS) -o $@ $^ -lm

$(BUILD)/fw_%.o: $(ROOT)/Src/%.c | $(BUILD)
	$(HOST_CC) $(CFLAGS) -std=c99 -c -o $@ $<

$(BUILD)/%.o: %.c | $(BUILD)
	$(HOST_CC) $(CFLAGS) -std=gnu99 -Wextra -Wno-unused-parameter -Werror -c -o $@ $<

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)
