// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 S2Arm contributors
// Real production parser/state machine with only the IOHID/dispatch boundary
// replaced. This test never opens or enumerates a physical device.
#import <Foundation/Foundation.h>
#import <IOKit/hid/IOHIDManager.h>
#import <GameController/GameController.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

static dispatch_block_t jobs[128], timers[128];
static unsigned njobs, ntimers, writes, spi_writes, bad_command, locked_io;
static uint8_t last_write[64];
static void queued_async(dispatch_queue_t q, dispatch_block_t b) { (void)q; jobs[njobs++] = [b copy]; }
static void queued_after(dispatch_time_t t, dispatch_queue_t q, dispatch_block_t b) { (void)t; (void)q; timers[ntimers++] = [b copy]; }
static IOReturn fake_set_report(IOHIDDeviceRef d, IOHIDReportType type, CFIndex reportID, const uint8_t *p, CFIndex n);
static void fake_register(IOHIDDeviceRef d, uint8_t *buf, CFIndex n, IOHIDReportCallback cb, void *context) {
    (void)d; (void)buf; (void)n; (void)cb; (void)context;
}
static IOReturn fake_close(IOHIDDeviceRef d, IOOptionBits options) { (void)d; (void)options; return kIOReturnSuccess; }
#define dispatch_async queued_async
#define dispatch_after queued_after
#define IOHIDDeviceSetReport fake_set_report
#define IOHIDDeviceRegisterInputReportCallback fake_register
#define IOHIDDeviceClose fake_close
#include "../../macos/joycon_imu.m"
#undef dispatch_async
#undef dispatch_after

static unsigned checks, failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); } } while (0)
static IOReturn fake_set_report(IOHIDDeviceRef d, IOHIDReportType type, CFIndex reportID, const uint8_t *p, CFIndex n) {
    (void)d; (void)type; (void)reportID;
    writes++;
    if (p[10] == 0x11) bad_command++;
    if (p[10] == 0x10) {
        spi_writes++;
        if (pthread_mutex_trylock(&g_mu)) locked_io++;
        else pthread_mutex_unlock(&g_mu);
        if (n == sizeof last_write) memcpy(last_write, p, n);
    }
    return kIOReturnSuccess;
}
static void run_jobs(void) {
    while (njobs) {
        dispatch_block_t b = jobs[0];
        for (unsigned i = 1; i < njobs; i++) jobs[i - 1] = jobs[i];
        jobs[--njobs] = nil; b();
    }
}
static void run_timer(void) {
    if (!ntimers) return;
    dispatch_block_t b = timers[0];
    for (unsigned i = 1; i < ntimers; i++) timers[i - 1] = timers[i];
    timers[--ntimers] = nil; b(); run_jobs();
}
static void drain(void) { run_jobs(); while (ntimers) run_timer(); }
static void pack(uint8_t *p, unsigned x, unsigned y) {
    p[0] = x; p[1] = (x >> 8) | (y << 4); p[2] = y >> 4;
}
static const s2a_joycon1_calibration factory = {{1900, 2300}, {1000, 1100}, {900, 1000}};
static const s2a_joycon1_calibration user = {{2100, 1800}, {1200, 1000}, {1100, 900}};
static void make_record(uint8_t *p, int right, const s2a_joycon1_calibration *cal) {
    pack(p + (right ? 0 : 3), cal->center[0], cal->center[1]);
    pack(p + (right ? 6 : 0), cal->above[0], cal->above[1]);
    pack(p + (right ? 3 : 6), cal->below[0], cal->below[1]);
}
static void reply(uint8_t p[64], jc_dev *d, const s2a_joycon1_calibration *cal) {
    memset(p, 0, 64); p[0] = 0x21; p[13] = 0x90; p[14] = 0x10;
    for (int i = 0; i < 4; i++) p[15 + i] = d->cal_address >> (8 * i);
    p[19] = d->cal_length;
    int skip = d->cal_phase == 1 ? 2 : 0;
    if (skip) { p[20] = 0xb2; p[21] = 0xa1; }
    if (cal) make_record(p + 20 + skip, d->product_id == JC_PID_R, cal);
}
static jc_dev *fresh(IOHIDDeviceRef device, int pid) {
    drain(); memset(g_dev, 0, sizeof g_dev);
    jc_dev *d = &g_dev[0]; d->device = device; d->generation = ++g_attachment_generation;
    d->product_id = pid; strcpy(d->id, "synthetic-device");
    atomic_store(&g_pad_want, 1); atomic_store(&g_want, 0); g_started = 1;
    return d;
}
static void rejects(jc_dev *d, uint8_t *p, CFIndex n) {
    apply_report(d, d->device, 0x21, p, n);
    CHECK(d->cal_pending && !d->calibration_known);
}
int main(void) { @autoreleasepool {
    // A CF object is retained by asynchronous production commands; no fake
    // pointer is passed to CF or any real IOHID device operation.
    CFDataRef token = CFDataCreate(kCFAllocatorDefault, NULL, 0);
    IOHIDDeviceRef device = (IOHIDDeviceRef)token;
    uint8_t p[64], valid[64], raw[S2A_JC1_RAW];
    jc_dev *d = fresh(device, JC_PID_L);
    request_calibration(d, device, d->generation); run_jobs();
    CHECK(last_write[10] == 0x10 && last_write[11] == 0x10 && last_write[12] == 0x80 && last_write[15] == 11);
    CHECK(!memcmp(last_write + 2, (uint8_t[]){0,1,0,0x40,0,1,0,0x40}, 8));
    reply(valid, d, &user);
    memcpy(p, valid, 64); p[13] = 0x80; rejects(d, p, 64);
    memcpy(p, valid, 64); p[14] = 0x11; rejects(d, p, 64);
    memcpy(p, valid, 64); p[18] = 1; rejects(d, p, 64); // same low 16 bits, different 32-bit address
    memcpy(p, valid, 64); p[19] = 9; rejects(d, p, 64);
    rejects(d, valid, 30); // 11 bytes promised, only 10 present
    d->cal_generation--; rejects(d, valid, 64); d->cal_generation++;
    apply_report(d, device, 0x21, valid, 64);
    CHECK(d->calibration_known && d->cal_phase == 3 && !memcmp(&d->calibration, &user, sizeof user));
    unsigned before = spi_writes; drain(); CHECK(spi_writes == before); // stale timeout cannot retry

    // Invalid user magic falls back to this side's factory record.
    d = fresh(device, JC_PID_R); request_calibration(d, device, d->generation); run_jobs();
    CHECK(last_write[11] == 0x1b && last_write[12] == 0x80);
    reply(p, d, &user); p[20] = 0;
    apply_report(d, device, 0x21, p, 64); run_jobs();
    CHECK(d->cal_phase == 2 && last_write[11] == 0x46 && last_write[12] == 0x60 && last_write[15] == 9);
    reply(p, d, &factory);
    // Omitted report ID, including a timer equal to the report ID.
    memmove(p, p + 1, 63); p[0] = 0x21;
    apply_report(d, device, 0x21, p, 63);
    CHECK(d->calibration_known && !memcmp(&d->calibration, &factory, sizeof factory));

    // Ordinary cached read and atomic raw+cal snapshot agree after calibration.
    uint8_t full[49] = {0x30}; pack(full + 9, factory.center[0], factory.center[1] + factory.above[1]);
    apply_report(d, device, 0x30, full, sizeof full);
    uint32_t buttons; int32_t sticks[4]; int side; s2a_joycon1_calibration snapshot;
    CHECK(s2a_joycon_pad_read_id(d->id, &buttons, sticks, &side, NULL) && sticks[2] == 0 && sticks[3] == 32767);
    CHECK(s2a_joycon_pad_snapshot_id(d->id, raw, &snapshot) && !memcmp(raw, full + 1, sizeof raw) && !memcmp(&snapshot, &factory, sizeof snapshot));
    s2a_joycon1_pair_decode_calibrated(NULL, raw, NULL, &snapshot, &buttons, sticks);
    CHECK(sticks[0] == 0 && sticks[1] == 0 && sticks[2] == 0 && sticks[3] == 32767);

    // IOHID's reportID is authoritative. The timer in an omitted-ID full
    // report can equal either known ID; decode, raw cache and IMU must all use
    // the same unambiguous payload offset instead of guessing from that timer.
    for (int timer = 0; timer < 2; timer++) for (int include_id = 0; include_id < 2; include_id++) {
        memset(full, 0, sizeof full); full[0] = 0x30; full[1] = timer ? 0x3f : 0x30;
        pack(full + 9, factory.center[0], factory.center[1] + factory.above[1]);
        full[37] = 0x00; full[38] = 0x10; // raw accel x = 4096
        full[39] = 0x00; full[40] = 0xf8; // raw accel y = -2048
        full[41] = 0x00; full[42] = 0x04; // raw accel z = 1024
        full[43] = 100; full[45] = 0x38; full[46] = 0xff; // raw gyro x=100, y=-200
        full[47] = 0x2c; full[48] = 0x01; // raw gyro z = 300
        apply_report(d, device, 0x30, full + (include_id ? 0 : 1), include_id ? 49 : 48);
        CHECK(s2a_joycon_pad_read_id(d->id, &buttons, sticks, &side, NULL) && sticks[2] == 0 && sticks[3] == 32767);
        CHECK(s2a_joycon_pad_snapshot_id(d->id, raw, &snapshot) && !memcmp(raw, full + 1, sizeof raw));
        CHECK(d->known && fabsf(d->sample.acceleration_x - 2048 * .000244f) < .0001f &&
              fabsf(d->sample.acceleration_y - 4096 * .000244f) < .0001f &&
              fabsf(d->sample.gyro_z - 300 * .06103f) < .0001f);
    }
    d->pad_known = d->pad_raw_known = d->known = 0;
    apply_report(d, device, 0x30, full + 1, 47);
    CHECK(!d->pad_known && !d->pad_raw_known && !d->known);

    // USB padding does not change the payload offset, with or without the ID.
    memset(p, 0xa5, sizeof p); memcpy(p, full, sizeof full);
    for (int include_id = 0; include_id < 2; include_id++) {
        apply_report(d, device, 0x30, p + (include_id ? 0 : 1), include_id ? 64 : 63);
        CHECK(s2a_joycon_pad_snapshot_id(d->id, raw, &snapshot) && !memcmp(raw, full + 1, sizeof raw));
    }
    // Simple packets have an 11-byte payload. Their first button byte can
    // also equal an ID; do not strip a pressed button from omitted-ID input.
    uint8_t simple[12] = {0x3f};
    for (int collision = 0; collision < 2; collision++) for (int include_id = 0; include_id < 2; include_id++) {
        simple[1] = collision ? 0x3f : 0x30;
        d->pad_known = d->pad_raw_known = d->known = 0;
        apply_report(d, device, 0x3f, simple + (include_id ? 0 : 1), include_id ? 12 : 11);
        // 0x30 = SL+SR; 0x3f adds all four face buttons (bits 0..3).
        CHECK(s2a_joycon_pad_read_id(d->id, &buttons, sticks, &side, NULL) && buttons == (collision ? 0xcfu : 0xc0u));
        CHECK(!d->pad_raw_known && !d->known);
    }
    d->pad_known = 0;
    apply_report(d, device, 0x3f, simple + 1, 10);
    CHECK(!d->pad_known);

    // Two simultaneous devices retain their own raw samples and calibration;
    // a composite GameController pair is never used as an axis substitute.
    d = fresh(device, JC_PID_L);
    CFDataRef token2 = CFDataCreate(kCFAllocatorDefault, (const uint8_t *)"R", 1);
    jc_dev *r = &g_dev[1]; r->device = (IOHIDDeviceRef)token2;
    r->generation = ++g_attachment_generation; r->product_id = JC_PID_R; strcpy(r->id, "synthetic-right");
    request_calibration(d, device, d->generation); request_calibration(r, r->device, r->generation); run_jobs();
    reply(p, d, &user); apply_report(d, device, 0x21, p, 64);
    reply(p, r, &factory); apply_report(r, r->device, 0x21, p, 64);
    CHECK(!memcmp(&d->calibration, &user, sizeof user) && !memcmp(&r->calibration, &factory, sizeof factory));
    memset(full, 0, sizeof full); full[0] = 0x30; pack(full + 6, user.center[0] + user.above[0], user.center[1]);
    apply_report(d, device, 0x30, full, sizeof full);
    memset(full, 0, sizeof full); full[0] = 0x30; pack(full + 9, factory.center[0], factory.center[1] - factory.below[1]);
    apply_report(r, r->device, 0x30, full, sizeof full);
    uint8_t lraw[S2A_JC1_RAW], rraw[S2A_JC1_RAW]; s2a_joycon1_calibration lc, rc;
    CHECK(s2a_joycon_pad_snapshot_id(d->id, lraw, &lc));
    CHECK(s2a_joycon_pad_snapshot_id(r->id, rraw, &rc));
    s2a_joycon1_pair_decode_calibrated(lraw, rraw, &lc, &rc, &buttons, sticks);
    CHECK(sticks[0] == 32767 && sticks[1] == 0 && sticks[2] == 0 && sticks[3] == -32767);
    drain(); CFRelease(token2);

    // Malformed user record (with valid magic) also falls back; an invalid
    // factory record ends the bounded attempt with no calibration.
    d = fresh(device, JC_PID_L); request_calibration(d, device, d->generation); run_jobs();
    reply(p, d, NULL); apply_report(d, device, 0x21, p, 64); run_jobs();
    CHECK(d->cal_phase == 2 && d->cal_pending);
    reply(p, d, NULL); apply_report(d, device, 0x21, p, 64); drain();
    CHECK(d->cal_phase == 3 && !d->calibration_known);

    // No response: exactly four requests, no retry loop after both timeouts.
    d = fresh(device, JC_PID_L); before = spi_writes;
    request_calibration(d, device, d->generation); drain();
    CHECK(spi_writes - before == 4 && d->cal_phase == 3 && !d->cal_pending);

    // Reused device pointer/slot: an old send and timeout cannot affect it.
    d = fresh(device, JC_PID_L); request_calibration(d, device, d->generation);
    uint64_t old = d->generation; d->generation = ++g_attachment_generation;
    before = spi_writes; drain(); CHECK(spi_writes == before && d->generation != old);
    d->cal_generation = old; reply(p, d, &user); rejects(d, p, 64);
    detach_device(device); CHECK(!d->device && !d->cal_pending && !d->calibration_known);

    // Initial enumeration can precede pad demand: enable must start the read
    // for already attached devices rather than depending solely on attach.
    d = fresh(device, JC_PID_L); atomic_store(&g_pad_want, 0);
    request_calibration(d, device, d->generation); CHECK(!d->cal_pending);
    s2a_joycon_pad_set_enabled(1); run_jobs(); CHECK(d->cal_phase == 1 && d->cal_pending);
    drain();
    d = fresh(device, JC_PID_PRO); before = spi_writes;
    request_calibration(d, device, d->generation); drain(); CHECK(before == spi_writes && !d->cal_pending);
    CHECK(!bad_command && !locked_io);
    CFRelease(token);
    printf("%s t_joycon1_spi %u/%u\n", failures ? "FAIL" : "✓", checks - failures, checks);
    return failures != 0;
} }
