# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 S2Arm contributors
NATIVE_TEST_OWN += joycon1_spi
ifeq ($(S2A_UNAME),Darwin)
.PHONY: native-test-joycon1_spi
native-test-joycon1_spi:
	@mkdir -p $(BUILD)/tests
	$(CC) $(CFLAGS) $(NATIVE_INC) -Iruntime/include -fobjc-arc runtime/tests/t_joycon1_spi.m runtime/src/pad.c -framework Foundation -framework GameController -framework IOKit -o $(BUILD)/tests/t_joycon1_spi
	$(BUILD)/tests/t_joycon1_spi
TEST_TARGETS += native-test-joycon1_spi
endif
