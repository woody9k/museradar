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

#include "presence_csi.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/ip_addr.h"
#include "ping/ping_sock.h"
#include "stack_monitor.h"
#include "wifi_mgr.h"

#ifdef CONFIG_HOMEHUB_PRESENCE_CSI_HOLD_S
#define PRESENCE_HOLD_S CONFIG_HOMEHUB_PRESENCE_CSI_HOLD_S
#endif
#ifdef CONFIG_HOMEHUB_PRESENCE_CSI_SCALE_MILLI
#define PRESENCE_SCALE_MILLI CONFIG_HOMEHUB_PRESENCE_CSI_SCALE_MILLI
#endif

// ---- Motion estimator (host-tested) -----------------------------------------
//
// Each CSI frame carries the channel estimate for the ~64 OFDM subcarriers
// between the access point and this board, as 8-bit I/Q pairs. Someone moving
// in the room changes the multipath, so the subcarrier amplitudes wobble from
// frame to frame; an empty room gives a nearly constant profile. The activity
// is the standard deviation of each subcarrier's normalised amplitude over a
// short window, averaged across subcarriers. A slowly tracked floor removes
// the quiet-room noise, and what's left is scaled to a 0..1 motion level.

#ifndef PRESENCE_WINDOW
#define PRESENCE_WINDOW 16             // frames in the sliding window
#endif
#define PRESENCE_MAX_SUB 64            // L-LTF: 64 subcarriers of 8-bit I/Q
#ifndef PRESENCE_HOLD_S
#define PRESENCE_HOLD_S 30             // presence stays true this long after motion
#endif
#ifndef PRESENCE_SCALE_MILLI
#define PRESENCE_SCALE_MILLI 80        // activity above the floor that means motion = 1.0
#endif
#define PRESENCE_TRIGGER 0.25f         // motion level that counts as "someone moved"
#define PRESENCE_EMA 0.3f              // smoothing of the motion level
#define PRESENCE_FLOOR_FALL 0.1f       // how fast the floor follows a quieter room
#define PRESENCE_FLOOR_RISE 0.0002f    // how slowly it creeps up (~5 min at 20 fps)
#define PRESENCE_RESUM_EVERY 1024      // frames between exact re-sums of the window
#define PRESENCE_STALE_US (10LL * 1000 * 1000)

typedef struct {
    int n_sub;                        // subcarriers per frame, fixed by the first frame
    float ring[PRESENCE_WINDOW][PRESENCE_MAX_SUB];
    float sum[PRESENCE_MAX_SUB];
    float sumsq[PRESENCE_MAX_SUB];
    int head;
    int filled;
    uint32_t frames;
    uint32_t dropped;                 // frames with another layout or no signal
    float activity;                   // mean per-subcarrier std over the window
    float floor;                      // tracked quiet-room activity
    bool floor_valid;
    float motion;                     // 0..1, smoothed
    int64_t last_frame_us;
    int64_t last_motion_us;           // 0 = never
} motion_est_t;

// What presence.read and the heartbeat need: small enough to snapshot.
typedef struct {
    bool ready;                       // the window is full and the floor is set
    int64_t last_frame_us;
    int64_t last_motion_us;
    float motion;
    float activity;
    float floor;
    uint32_t frames;
    uint32_t dropped;
} presence_state_t;

static void motion_est_init(motion_est_t *m) {
    memset(m, 0, sizeof(*m));
}

// Recompute the window sums from the ring, so float drift can't build up.
static void motion_est_resum(motion_est_t *m) {
    memset(m->sum, 0, sizeof(m->sum));
    memset(m->sumsq, 0, sizeof(m->sumsq));
    for (int i = 0; i < m->filled; i++) {
        for (int k = 0; k < m->n_sub; k++) {
            float a = m->ring[i][k];
            m->sum[k] += a;
            m->sumsq[k] += a * a;
        }
    }
}

// Feed one CSI frame. Returns false for a frame that was dropped.
static bool motion_est_push(motion_est_t *m, const int8_t *iq, size_t len,
                            bool first_word_invalid, int64_t now_us) {
    size_t skip = first_word_invalid ? 4 : 0;
    if (len <= skip) return false;
    int n = (int)((len - skip) / 2);
    if (n > PRESENCE_MAX_SUB) n = PRESENCE_MAX_SUB;
    if (m->n_sub == 0) m->n_sub = n;
    if (n != m->n_sub) {
        m->dropped++;
        return false;
    }

    // Amplitude per subcarrier, normalised by the frame's mean so gain
    // changes in the receiver don't look like motion.
    float amp[PRESENCE_MAX_SUB];
    float mean = 0;
    int used = 0;
    for (int k = 0; k < n; k++) {
        float i = iq[skip + 2 * k];
        float q = iq[skip + 2 * k + 1];
        amp[k] = sqrtf(i * i + q * q);
        if (amp[k] > 0) {
            mean += amp[k];
            used++;
        }
    }
    if (used < n / 2 || mean <= 0) {
        m->dropped++;
        return false;
    }
    mean /= used;

    float *slot = m->ring[m->head];
    bool full = m->filled == PRESENCE_WINDOW;
    for (int k = 0; k < n; k++) {
        float a = amp[k] / mean;
        if (full) {
            m->sum[k] -= slot[k];
            m->sumsq[k] -= slot[k] * slot[k];
        }
        slot[k] = a;
        m->sum[k] += a;
        m->sumsq[k] += a * a;
    }
    m->head = (m->head + 1) % PRESENCE_WINDOW;
    if (!full) m->filled++;
    m->frames++;
    m->last_frame_us = now_us;
    if (m->frames % PRESENCE_RESUM_EVERY == 0) motion_est_resum(m);
    if (m->filled < PRESENCE_WINDOW) return true;

    // Spread of each subcarrier over the window. Guard bands and DC are
    // always ~0, so leave them out rather than let them dilute the mean.
    float w = (float)PRESENCE_WINDOW;
    float total = 0;
    int counted = 0;
    for (int k = 0; k < n; k++) {
        float mu = m->sum[k] / w;
        if (mu < 0.05f) continue;
        float var = m->sumsq[k] / w - mu * mu;
        if (var < 0) var = 0;
        total += sqrtf(var);
        counted++;
    }
    if (!counted) return true;
    m->activity = total / counted;

    if (!m->floor_valid) {
        m->floor = m->activity;
        m->floor_valid = true;
    } else if (m->activity < m->floor) {
        m->floor += (m->activity - m->floor) * PRESENCE_FLOOR_FALL;
    } else {
        m->floor += (m->activity - m->floor) * PRESENCE_FLOOR_RISE;
    }

    float raw = (m->activity - m->floor) / (PRESENCE_SCALE_MILLI / 1000.0f);
    if (raw < 0) raw = 0;
    if (raw > 1) raw = 1;
    m->motion += (raw - m->motion) * PRESENCE_EMA;
    if (m->motion > PRESENCE_TRIGGER) m->last_motion_us = now_us;
    return true;
}

static void motion_est_state(const motion_est_t *m, presence_state_t *s) {
    s->ready = m->filled == PRESENCE_WINDOW && m->floor_valid;
    s->last_frame_us = m->last_frame_us;
    s->last_motion_us = m->last_motion_us;
    s->motion = m->motion;
    s->activity = m->activity;
    s->floor = m->floor;
    s->frames = m->frames;
    s->dropped = m->dropped;
}

static cJSON *presence_error(cJSON *result, const char *code, const char *message) {
    cJSON_AddBoolToObject(result, "ok", false);
    cJSON *error = cJSON_AddObjectToObject(result, "error");
    cJSON_AddStringToObject(error, "code", code);
    cJSON_AddStringToObject(error, "message", message);
    return result;
}

static double rounded(float value, int decimals) {
    double scale = pow(10, decimals);
    return round(value * scale) / scale;
}

static cJSON *presence_result(const presence_state_t *s, int64_t now_us,
                              int rssi_dbm, float frames_per_s) {
    cJSON *result = cJSON_CreateObject();
    if (!result) return NULL;
    if (s->last_frame_us == 0 || now_us - s->last_frame_us > PRESENCE_STALE_US) {
        return presence_error(result, "no_csi",
                              "no Wi-Fi samples in the last 10 seconds; the "
                              "gadget has to be on Wi-Fi to sense motion");
    }
    if (!s->ready) {
        return presence_error(result, "warming_up",
                              "collecting the first seconds of Wi-Fi samples; "
                              "try again shortly");
    }
    bool presence = s->last_motion_us != 0 &&
                    now_us - s->last_motion_us < PRESENCE_HOLD_S * 1000000LL;
    cJSON_AddBoolToObject(result, "ok", true);
    cJSON *payload = cJSON_AddObjectToObject(result, "payload");
    cJSON_AddBoolToObject(payload, "presence", presence);
    cJSON_AddNumberToObject(payload, "motion", rounded(s->motion, 2));
    if (s->last_motion_us == 0) {
        cJSON_AddNullToObject(payload, "last_motion_s");
    } else {
        cJSON_AddNumberToObject(payload, "last_motion_s",
                                (double)((now_us - s->last_motion_us) / 1000000));
    }
    cJSON_AddNumberToObject(payload, "hold_s", PRESENCE_HOLD_S);
    cJSON_AddNumberToObject(payload, "activity", rounded(s->activity, 4));
    cJSON_AddNumberToObject(payload, "floor", rounded(s->floor, 4));
    cJSON_AddNumberToObject(payload, "frames_per_s", rounded(frames_per_s, 1));
    cJSON_AddNumberToObject(payload, "rssi_dbm", rssi_dbm);
    cJSON_AddNumberToObject(payload, "samples", s->frames);
    return result;
}

// ---- ESP32 CSI capture ------------------------------------------------------

static const char *TAG = "link.presence";

#ifndef CONFIG_HOMEHUB_PRESENCE_CSI_PING_MS
#define CONFIG_HOMEHUB_PRESENCE_CSI_PING_MS 100
#endif

// One frame as the Wi-Fi task hands it over. Kept small: the callback runs on
// the Wi-Fi task, so it only copies and queues.
typedef struct {
    int64_t us;
    int8_t rssi;
    bool first_word_invalid;
    uint16_t len;
    int8_t iq[PRESENCE_MAX_SUB * 2];
} csi_frame_t;

static QueueHandle_t s_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static presence_state_t s_state;
static float s_fps;
static int s_rssi;
static volatile uint32_t s_cb_frames;
static volatile uint32_t s_cb_dropped;
static uint8_t s_bssid[6];
static bool s_running;
static esp_ping_handle_t s_ping;

static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    (void)ctx;
    if (!info || !info->buf || !s_queue) return;
    // Only the access point's own frames: beacons and the ping replies.
    if (memcmp(info->mac, s_bssid, sizeof(s_bssid)) != 0) return;
    csi_frame_t f = {
        .us = esp_timer_get_time(),
        .rssi = (int8_t)info->rx_ctrl.rssi,
        .first_word_invalid = info->first_word_invalid,
        .len = info->len > sizeof(f.iq) ? sizeof(f.iq) : info->len,
    };
    memcpy(f.iq, info->buf, f.len);
    s_cb_frames++;
    if (xQueueSend(s_queue, &f, 0) != pdTRUE) s_cb_dropped++;
}

static void capture_start(void) {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return;
    memcpy(s_bssid, ap.bssid, sizeof(s_bssid));

    // Take the 20 MHz frame types the AP is likely to send: 11g (beacons on
    // many APs), 11n and Wi-Fi 6 single-user (data and ping replies). Each
    // has its own CSI layout, so the task picks the most common one and
    // feeds only that to the estimator.
    wifi_csi_config_t cfg = {0};
    cfg.enable = 1;
    cfg.acquire_csi_legacy = 1;
    cfg.acquire_csi_ht20 = 1;
    cfg.acquire_csi_su = 1;
    esp_err_t err = esp_wifi_set_csi_config(&cfg);
    if (err == ESP_OK) err = esp_wifi_set_csi_rx_cb(csi_cb, NULL);
    if (err == ESP_OK) err = esp_wifi_set_csi(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CSI setup failed: %s", esp_err_to_name(err));
        return;
    }

    // Ping the gateway so the AP keeps talking to us between beacons.
    esp_netif_ip_info_t ip = {0};
    esp_netif_t *netif = wifi_mgr_get_netif();
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.gw.addr) {
        esp_ping_config_t pc = ESP_PING_DEFAULT_CONFIG();
        ip_addr_set_ip4_u32(&pc.target_addr, ip.gw.addr);
        pc.interval_ms = CONFIG_HOMEHUB_PRESENCE_CSI_PING_MS;
        pc.count = ESP_PING_COUNT_INFINITE;
        pc.data_size = 8;
        esp_ping_callbacks_t cbs = {0};
        if (esp_ping_new_session(&pc, &cbs, &s_ping) == ESP_OK) {
            esp_ping_start(s_ping);
        } else {
            s_ping = NULL;
            ESP_LOGW(TAG, "gateway ping failed to start; relying on beacons");
        }
    }
    s_running = true;
    ESP_LOGI(TAG, "Wi-Fi radar on: CSI from %02x:%02x:%02x:%02x:%02x:%02x, "
             "pinging " IPSTR " every %d ms",
             s_bssid[0], s_bssid[1], s_bssid[2], s_bssid[3], s_bssid[4], s_bssid[5],
             IP2STR(&ip.gw), CONFIG_HOMEHUB_PRESENCE_CSI_PING_MS);
}

static void capture_stop(void) {
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(NULL, NULL);
    if (s_ping) {
        esp_ping_stop(s_ping);
        esp_ping_delete_session(s_ping);
        s_ping = NULL;
    }
    s_running = false;
    ESP_LOGI(TAG, "Wi-Fi radar off: Wi-Fi down");
}

// The first frames after Wi-Fi comes up vote on a CSI layout (by length);
// the winner is the only one the estimator sees from then on.
#define LAYOUT_VOTES 40
#define LAYOUT_KINDS 4

typedef struct {
    bool locked;
    uint16_t len;
    uint16_t lens[LAYOUT_KINDS];
    uint16_t votes[LAYOUT_KINDS];
    uint16_t total;
    int64_t first_us;
} layout_t;

// Returns true once a layout is chosen (this call or earlier).
static bool layout_vote(layout_t *l, uint16_t len, int64_t now_us) {
    if (l->locked) return true;
    if (!l->total) l->first_us = now_us;
    for (int i = 0; i < LAYOUT_KINDS; i++) {
        if (l->votes[i] && l->lens[i] != len) continue;
        l->lens[i] = len;
        l->votes[i]++;
        break;
    }
    l->total++;
    if (l->total < LAYOUT_VOTES && now_us - l->first_us < 3000000) return false;
    int best = 0;
    for (int i = 1; i < LAYOUT_KINDS; i++) {
        if (l->votes[i] > l->votes[best]) best = i;
    }
    l->len = l->lens[best];
    l->locked = true;
    ESP_LOGI(TAG, "CSI layout: %u-byte frames (%u of the first %u)",
             l->len, l->votes[best], l->total);
    return true;
}

static void presence_task(void *arg) {
    (void)arg;
    stack_monitor_t stack = STACK_MONITOR_INIT;
    static motion_est_t est;   // 4 KB: static, not on the stack
    motion_est_init(&est);
    layout_t layout = {0};
    csi_frame_t f;
    int64_t rate_t0 = esp_timer_get_time();
    uint32_t rate_n = 0;
    float fps = 0;
    int rssi = 0;
    bool was_ready = false;
    for (;;) {
        if (xQueueReceive(s_queue, &f, pdMS_TO_TICKS(1000)) == pdTRUE) {
            rssi = f.rssi;
            if (layout_vote(&layout, f.len, f.us) && f.len == layout.len &&
                motion_est_push(&est, f.iq, f.len, f.first_word_invalid, f.us)) {
                rate_n++;
            }
        }
        int64_t now = esp_timer_get_time();
        if (now - rate_t0 >= 1000000) {
            fps = rate_n * 1000000.0f / (float)(now - rate_t0);
            rate_n = 0;
            rate_t0 = now;
        }

        presence_state_t st;
        motion_est_state(&est, &st);
        taskENTER_CRITICAL(&s_lock);
        s_state = st;
        s_fps = fps;
        s_rssi = rssi;
        taskEXIT_CRITICAL(&s_lock);
        if (st.ready && !was_ready) {
            ESP_LOGI(TAG, "radar ready: %d subcarriers, quiet floor %.4f", est.n_sub, est.floor);
        }
        was_ready = st.ready;

        bool up = wifi_mgr_is_connected();
        if (up && !s_running) {
            motion_est_init(&est);   // a fresh baseline for each connection
            memset(&layout, 0, sizeof(layout));
            capture_start();
        } else if (!up && s_running) {
            capture_stop();
        }
        stack_monitor_poll(&stack);
    }
}

void presence_csi_init(void) {
    s_queue = xQueueCreate(8, sizeof(csi_frame_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "no memory for the CSI queue");
        return;
    }
    if (xTaskCreate(presence_task, "presence", 4096, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to start the presence task");
        return;
    }
    ESP_LOGI(TAG, "Wi-Fi radar waiting for Wi-Fi");
}

cJSON *presence_csi_command(void) {
    presence_state_t st;
    float fps;
    int rssi;
    taskENTER_CRITICAL(&s_lock);
    st = s_state;
    fps = s_fps;
    rssi = s_rssi;
    taskEXIT_CRITICAL(&s_lock);
    return presence_result(&st, esp_timer_get_time(), rssi, fps);
}

void presence_csi_status(char *buf, size_t size) {
    presence_state_t st;
    float fps;
    taskENTER_CRITICAL(&s_lock);
    st = s_state;
    fps = s_fps;
    taskEXIT_CRITICAL(&s_lock);
    snprintf(buf, size, "radar %s motion=%.2f act=%.4f floor=%.4f csi=%.1f/s cb=%lu drop=%lu+%lu",
             !s_running ? "off" : st.ready ? "on" : "warming",
             st.motion, st.activity, st.floor, fps,
             (unsigned long)s_cb_frames, (unsigned long)s_cb_dropped,
             (unsigned long)st.dropped);
}
