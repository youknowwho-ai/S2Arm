# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 S2Arm contributors
NATIVE_TEST_OWN += joycon1_calibration
.PHONY: native-test-joycon1_calibration
native-test-joycon1_calibration:
	@mkdir -p $(BUILD)/tests
	$(CC) $(CFLAGS) $(NATIVE_INC) -Iruntime/include runtime/tests/t_joycon1_calibration.c runtime/src/pad.c -lm -lpthread -o $(BUILD)/tests/t_joycon1_calibration
	$(BUILD)/tests/t_joycon1_calibration
TEST_TARGETS += native-test-joycon1_calibration
