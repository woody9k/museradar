/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Host Wi-Fi radar harness: synthetic CSI frames in, presence.read result out.
// The runner extracts the production estimator into presence_motion.inc.
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cJSON.h"

#include "presence_motion.inc"

#define S(x) ((int64_t)(x) * 1000000LL)
#define FRAME_US 100000            // 10 frames a second, like beacons
#define N_SUB 64

static uint32_t s_seed = 12345;

// Deterministic noise in [-1, 1).
static float noise(void) {
    s_seed = s_seed * 1664525u + 1013904223u;
    return (float)(s_seed >> 8) / (float)(1u << 24) * 2.0f - 1.0f;
}

// A plausible quiet-room channel: guard bands and DC at zero, the rest a
// smooth curve of amplitudes in the 8-bit range.
static void base_profile(float *amp) {
    for (int k = 0; k < N_SUB; k++) {
        if (k < 4 || k >= N_SUB - 3 || k == N_SUB / 2) {
            amp[k] = 0;
        } else {
            amp[k] = 50.0f + 25.0f * sinf(k / 6.0f);
        }
    }
}

// Turn amplitudes into I/Q at a fixed phase, with +/- wobble on each one.
static void frame_from(const float *amp, float wobble, int8_t *iq) {
    for (int k = 0; k < N_SUB; k++) {
        float a = amp[k];
        if (a > 0) a *= 1.0f + wobble * noise();
        float phase = k * 0.3f;
        int i = (int)lrintf(a * cosf(phase));
        int q = (int)lrintf(a * sinf(phase));
        if (i > 127) i = 127;
        if (i < -127) i = -127;
        if (q > 127) q = 127;
        if (q < -127) q = -127;
        iq[2 * k] = (int8_t)i;
        iq[2 * k + 1] = (int8_t)q;
    }
}

// Feed `count` frames with the given wobble, FRAME_US apart, from *now_us.
static void feed(motion_est_t *m, int count, float wobble, int64_t *now_us) {
    float amp[N_SUB];
    base_profile(amp);
    int8_t iq[N_SUB * 2];
    for (int i = 0; i < count; i++) {
        frame_from(amp, wobble, iq);
        bool ok = motion_est_push(m, iq, sizeof(iq), false, *now_us);
        assert(ok);
        *now_us += FRAME_US;
    }
}

static cJSON *read_result(const motion_est_t *m, int64_t now_us) {
    presence_state_t st;
    motion_est_state(m, &st);
    cJSON *r = presence_result(&st, now_us, -55, 10.0f);
    assert(r);
    return r;
}

static const char *error_code(cJSON *r) {
    assert(!cJSON_IsTrue(cJSON_GetObjectItem(r, "ok")));
    assert(!cJSON_GetObjectItem(r, "payload"));
    return cJSON_GetStringValue(cJSON_GetObjectItem(cJSON_GetObjectItem(r, "error"), "code"));
}

static cJSON *payload(cJSON *r) {
    assert(cJSON_IsTrue(cJSON_GetObjectItem(r, "ok")));
    assert(cJSON_GetArraySize(r) == 2);   // exactly ok and payload
    cJSON *p = cJSON_GetObjectItem(r, "payload");
    assert(p);
    return p;
}

static double number(cJSON *p, const char *key) {
    cJSON *v = cJSON_GetObjectItem(p, key);
    assert(cJSON_IsNumber(v));
    return v->valuedouble;
}

int main(void) {
    static motion_est_t m;
    motion_est_init(&m);
    int64_t now = S(100);

    // Nothing yet: not an answer, an error.
    cJSON *r = read_result(&m, now);
    assert(strcmp(error_code(r), "no_csi") == 0);
    cJSON_Delete(r);

    // The first frames fill the window; until then it's warming up.
    feed(&m, PRESENCE_WINDOW - 1, 0.02f, &now);
    r = read_result(&m, now);
    assert(strcmp(error_code(r), "warming_up") == 0);
    cJSON_Delete(r);

    // A quiet room: low motion, nobody present, the floor settles.
    feed(&m, 60, 0.02f, &now);
    r = read_result(&m, now);
    cJSON *p = payload(r);
    assert(cJSON_IsFalse(cJSON_GetObjectItem(p, "presence")));
    assert(number(p, "motion") < PRESENCE_TRIGGER);
    assert(cJSON_IsNull(cJSON_GetObjectItem(p, "last_motion_s")));
    assert(number(p, "floor") > 0 && number(p, "floor") < 0.05);
    assert(number(p, "frames_per_s") == 10.0);
    assert(number(p, "rssi_dbm") == -55);
    assert(number(p, "hold_s") == PRESENCE_HOLD_S);
    assert(number(p, "samples") == (double)m.frames);
    cJSON_Delete(r);
    float quiet_floor = m.floor;

    // Someone walks through: the profile wobbles, motion jumps, presence.
    feed(&m, 20, 0.3f, &now);
    r = read_result(&m, now);
    p = payload(r);
    assert(cJSON_IsTrue(cJSON_GetObjectItem(p, "presence")));
    assert(number(p, "motion") > PRESENCE_TRIGGER);
    assert(number(p, "last_motion_s") == 0);
    assert(number(p, "activity") > number(p, "floor") * 3);
    cJSON_Delete(r);
    // Twenty busy frames must not have dragged the floor up to meet them.
    assert(m.floor < quiet_floor * 1.5f);

    // Quiet again: the busy frames take the window length to drain and the
    // level a few more frames to settle, so motion stops a couple of seconds
    // after the room does, and no later. Presence holds.
    feed(&m, 50, 0.02f, &now);
    r = read_result(&m, now);
    p = payload(r);
    assert(cJSON_IsTrue(cJSON_GetObjectItem(p, "presence")));
    assert(number(p, "motion") < PRESENCE_TRIGGER);
    assert(number(p, "last_motion_s") >= 1 && number(p, "last_motion_s") <= 4);
    cJSON_Delete(r);

    // ...until the hold time runs out.
    now += S(PRESENCE_HOLD_S);
    feed(&m, 1, 0.02f, &now);
    r = read_result(&m, now);
    p = payload(r);
    assert(cJSON_IsFalse(cJSON_GetObjectItem(p, "presence")));
    assert(number(p, "last_motion_s") >= PRESENCE_HOLD_S);
    cJSON_Delete(r);

    // A frame in another layout is dropped, not mixed into the window.
    int8_t other[32] = {10, 10, 20, 20, 30, 30, 40, 40};
    uint32_t dropped = m.dropped;
    assert(!motion_est_push(&m, other, sizeof(other), false, now));
    assert(m.dropped == dropped + 1);
    // So is a frame with no signal in it.
    int8_t empty[N_SUB * 2] = {0};
    assert(!motion_est_push(&m, empty, sizeof(empty), false, now));
    assert(m.dropped == dropped + 2);

    // The hardware's invalid first word is skipped, and the frame still fits.
    float amp[N_SUB];
    base_profile(amp);
    int8_t iq[N_SUB * 2 + 4] = {0};
    frame_from(amp, 0.02f, iq + 4);
    assert(motion_est_push(&m, iq, sizeof(iq), true, now));

    // No frames for a while: stale, and says so.
    r = read_result(&m, now + PRESENCE_STALE_US + 1);
    assert(strcmp(error_code(r), "no_csi") == 0);
    cJSON_Delete(r);

    // A long run keeps the window sums exact.
    feed(&m, PRESENCE_RESUM_EVERY * 2, 0.02f, &now);
    float sum0 = m.sum[10];
    motion_est_resum(&m);
    assert(fabsf(m.sum[10] - sum0) < 1e-3f);

    puts("ok");
    return 0;
}
