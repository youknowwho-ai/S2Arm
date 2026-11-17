// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 S2Arm contributors
#include "s2a_pad.h"
#include <stdio.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); } } while (0)

static void pack(uint8_t *p, unsigned x, unsigned y) {
    p[0] = x; p[1] = (x >> 8) | (y << 4); p[2] = y >> 4;
}
static void record(uint8_t *p, int right, const s2a_joycon1_calibration *cal) {
    pack(p + (right ? 0 : 3), cal->center[0], cal->center[1]);
    pack(p + (right ? 6 : 0), cal->above[0], cal->above[1]);
    pack(p + (right ? 3 : 6), cal->below[0], cal->below[1]);
}

int main(void) {
    // Original synthetic records, independent of any player's hardware values.
    const s2a_joycon1_calibration cal[2] = {
        {{1900, 2300}, {1000, 1100}, {900, 1000}},
        {{2200, 1800}, {1200, 1000}, {1100, 900}}
    };
    for (int right = 0; right < 2; right++) {
        int pid = right ? 0x2007 : 0x2006;
        uint8_t bytes[9]; s2a_joycon1_calibration parsed = {0};
        record(bytes, right, &cal[right]);
        CHECK(s2a_joycon1_calibration_parse(pid, bytes, &parsed));
        CHECK(!memcmp(&parsed, &cal[right], sizeof parsed));
        for (unsigned axis = 0; axis < 2; axis++) {
            int32_t value, prior = -32768;
            for (unsigned raw = 0; raw < 4096; raw++) {
                CHECK(s2a_joycon1_calibration_axis(&parsed, axis, raw, &value));
                CHECK(value >= prior && value >= -32767 && value <= 32767);
                prior = value;
            }
            CHECK(s2a_joycon1_calibration_axis(&parsed, axis, parsed.center[axis], &value) && value == 0);
            CHECK(s2a_joycon1_calibration_axis(&parsed, axis, parsed.center[axis] - parsed.below[axis], &value) && value == -32767);
            CHECK(s2a_joycon1_calibration_axis(&parsed, axis, parsed.center[axis] + parsed.above[axis], &value) && value == 32767);
        }
        // Every calibrated cardinal endpoint must exceed the unchanged guest
        // direction threshold after the existing 15% linear deadzone.
        for (int axis = 0; axis < 2; axis++) for (int positive = 0; positive < 2; positive++) {
            uint8_t full[49] = {0x30};
            unsigned xy[2] = {parsed.center[0], parsed.center[1]};
            xy[axis] += positive ? parsed.above[axis] : -(int)parsed.below[axis];
            pack(full + (right ? 9 : 6), xy[0], xy[1]);
            uint32_t b; int32_t sticks[4]; int side;
#ifdef JC1_TEST_LEGACY
            CHECK(s2a_joycon1_decode(pid, full, sizeof full, &b, sticks, &side));
#else
            CHECK(s2a_joycon1_decode_calibrated(pid, full, sizeof full, &parsed, &b, sticks, &side));
#endif
            int32_t v = sticks[right * 2 + axis];
            int mapped = s2a_pad_axis_deadzone(v / 32767.0f, .15f) * 32767;
            CHECK(positive ? mapped > 16383 : mapped < -16383);
            CHECK(sticks[right * 2 + (1 - axis)] == 0);
            uint8_t *raw = full + 1;
            s2a_joycon1_pair_decode_calibrated(right ? NULL : raw, right ? raw : NULL, &parsed, &parsed, &b, sticks);
            CHECK(sticks[right * 2 + axis] == (positive ? 32767 : -32767));
            CHECK(sticks[(1 - right) * 2] == 0 && sticks[(1 - right) * 2 + 1] == 0);
        }
        s2a_joycon1_calibration unchanged = parsed;
        memset(bytes, 0xff, sizeof bytes);
        CHECK(!s2a_joycon1_calibration_parse(pid, bytes, &parsed));
        CHECK(!memcmp(&parsed, &unchanged, sizeof parsed));
        memset(bytes, 0, sizeof bytes);
        CHECK(!s2a_joycon1_calibration_parse(pid, bytes, &parsed));
    }
    int32_t value;
    CHECK(!s2a_joycon1_calibration_axis(&cal[0], 2, 2000, &value));
    CHECK(!s2a_joycon1_calibration_axis(&cal[0], 0, 4096, &value));
    CHECK(!s2a_joycon1_calibration_axis(NULL, 0, 2000, &value));
    CHECK(!s2a_joycon1_calibration_parse(0x2009, (const uint8_t[9]){0}, NULL));
    // Pro and the legacy API remain byte-for-byte equivalent with a cal argument.
    uint8_t full[49] = {0x30}; pack(full + 6, 1111, 3333); pack(full + 9, 2222, 444);
    uint32_t b1, b2; int32_t s1[4], s2[4]; int side1, side2;
    CHECK(s2a_joycon1_decode(0x2009, full, sizeof full, &b1, s1, &side1));
    CHECK(s2a_joycon1_decode_calibrated(0x2009, full, sizeof full, &cal[0], &b2, s2, &side2));
    CHECK(b1 == b2 && side1 == side2 && !memcmp(s1, s2, sizeof s1));
    printf("%s t_joycon1_calibration %u/%u\n", failures ? "FAIL" : "✓", checks - failures, checks);
    return failures != 0;
}
