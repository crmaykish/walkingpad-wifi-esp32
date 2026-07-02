/*
 * walkingpad-bridge — taps the walking pad's console->drive UART (pin 5, 2400 8N1),
 * decodes speed, integrates distance, logs speed-change events to flash, and serves
 * it all as JSON + a web page over WiFi.
 *
 * Frame: AA <cmd> <HH> <LL> <CK> 55     (CK = (cmd + HH + LL) & 0xFF)
 *   cmd 0x42 = run-at-speed, value V = HH*256 + LL, mph = (V - 105) / 160
 *   cmd 0x40 / 0xA2 / 0x50 = idle/stopped -> 0 mph
 *
 * Wiring (recommended): pin5 --(4.7k series + 10k to GND, ~3.4V)--> RX2/GPIO16 (UART2 RX);
 *         pin1 -> GND; pad 5V rail -> ESP32 5V/VIN. RX only. PASSIVE divider, no active shifter.
 * AS BUILT: pin5 (5V) wired DIRECTLY to RX2/GPIO16 (no divider) — relies on the ESP32's
 *         unofficial 5V tolerance; OUT OF SPEC (GPIO abs-max ~3.6V; UART idles high so 5V is
 *         continuous on the ESD clamp). Works so far; suspect FIRST if RX ever gets flaky.
 *
 * Logging: 8-byte records {epoch, 0.1mph, flags, checksum} appended to a SEGMENTED
 * CIRCULAR LOG on LittleFS (LOG_SEGS files of LOG_SEG_BYTES each). When the active
 * segment fills, we advance to the next and ERASE it — so the newest ~(LOG_SEGS-1)
 * segments are always kept and the oldest drops off. Bounded forever, never fails.
 * Reconstruct hours/miles from the timestamped records (see /stats and /log.csv).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_sntp.h"
#include "esp_littlefs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "mdns.h"

/* ------------------- user config ------------------- */
#if defined(__has_include)
#  if __has_include("wifi_secrets.h")
#    include "wifi_secrets.h"       /* gitignored: real WIFI_SSID / WIFI_PASS */
#  endif
#endif
#ifndef WIFI_SSID
#define WIFI_SSID   "YOUR_SSID"     /* fallback — or set at runtime via the setup AP */
#endif
#ifndef WIFI_PASS
#define WIFI_PASS   "YOUR_PASSWORD"
#endif
#define HOSTNAME    "walkingpad"       /* -> http://walkingpad.local/ */
#define STRIDE_M    0.624              /* metres/step; calibrated to the pad's own footfall
                                          counter (est 4424 vs real 4752 on 2026-07-02:
                                          0.67 * 4424/4752). Re-tune if the estimate drifts. */
#define NTP_SERVER  "pool.ntp.org"
#define LOCAL_TZ    "EST5EDT,M3.2.0/2,M11.1.0/2"  /* US Eastern, DST-aware: day/week/month
                                                     buckets use this so they match the wall
                                                     calendar (records still store UTC epoch) */
/* --------------------------------------------------- */

#define PAD_UART        UART_NUM_2
#define PAD_RX_GPIO     16             /* = the board's RX2 pin */
#define PAD_BAUD        2400
#define STALE_US        2000000LL      /* no valid frame for 2s -> speed 0 */
#define AP_FALLBACK_US  30000000LL     /* no STA connect in 30s -> open setup AP */
#define ODO_SAVE_US     30000000LL     /* persist odometer at most every 30s */
#define LOG_DEBOUNCE_US 400000LL       /* speed must settle 0.4s before logging */
#define LOG_HB_US       240000000LL    /* heartbeat record every 4 min while moving */
#define LOG_GAP_CAP_S   600            /* moving segment > 10 min => treat as power gap */
#define EPOCH_VALID     1600000000UL   /* time() beyond this => NTP-synced */
#define LOG_SEGS        6              /* circular log: this many segment files ... */
#define LOG_SEG_BYTES   (100 * 1024)   /* ... of this size each (6*100K = 600K < 960K part) */
/* --- gamification --- */
#define GOAL_MI_DEFAULT 2.0            /* daily distance goal (mi); runtime-settable via /goal */
#define STAT_DAYS       370            /* per-day window for records/streak (JSON days[] still 60) */
#define MILESTONE_MI    25.0           /* lifetime-distance marker spacing (mi) */
#define ACTIVE_MI       0.2            /* a day counts as "active" (filled dot) above this (mi) */

static const char *TAG = "pad";
#define MINI(a,b) ((a) < (b) ? (a) : (b))

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static float    g_speed_mph = 0.0f;
static double   g_dist_mi   = 0.0;    /* persisted odometer (survives reboots, resettable) */
static double   g_sess_mi   = 0.0;    /* live session distance since power-on (starts at 0) */
static double   g_sess_s    = 0.0;    /* live session MOVING seconds since power-on (speed>0) */
static double   g_life_mi   = 0.0;    /* monotonic lifetime miles for milestones (NOT reset by /reset) */
static uint16_t g_rawV = 0;
static uint8_t  g_cmd  = 0;
static uint32_t g_frames = 0;
static int64_t  g_last_frame_us = 0;
static int64_t  g_boot_us = 0;
static volatile bool g_connected = false;
static volatile bool g_ap_active = false;
static volatile bool g_time_synced = false;
static char     g_ip[16] = "0.0.0.0";
static esp_timer_handle_t g_fallback_timer;

/* segmented circular log */
static SemaphoreHandle_t g_log_mux;
static FILE    *g_logf = NULL;       /* append handle for the active segment */
static int      g_seg  = 0;          /* active segment index */
static long     g_seg_bytes = 0;     /* bytes written to the active segment */
static uint32_t g_log_records = 0;   /* total records currently stored (all segments) */
static int      g_logged_dmph = -1;  /* last logged speed, 0.1 mph units */

typedef struct __attribute__((packed)) {
    uint32_t ts;      /* epoch seconds, 0 if not yet NTP-synced */
    uint16_t dmph;    /* speed in 0.1 mph */
    uint8_t  flags;   /* bit0: time valid */
    uint8_t  ck;      /* sum(bytes[0..6]) & 0xFF */
} logrec_t;

static inline float mph_from_V(uint16_t V) {
    float m = ((float)V - 105.0f) / 160.0f;
    return m < 0.0f ? 0.0f : m;
}
static void seg_path(int i, char *out, size_t n) { snprintf(out, n, "/lfs/walk.%d", i); }

/* ------------------- NVS: creds + odometer + log-segment ------------------- */
static bool nvs_load_creds(char *ssid, size_t ss, char *pass, size_t ps) {
    nvs_handle_t h; bool ok = false;
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) {
        size_t a = ss, b = ps;
        if (nvs_get_str(h, "ssid", ssid, &a) == ESP_OK &&
            nvs_get_str(h, "pass", pass, &b) == ESP_OK && a > 1) ok = true;
        nvs_close(h);
    }
    return ok;
}
static void nvs_save_creds(const char *ssid, const char *pass) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "ssid", ssid); nvs_set_str(h, "pass", pass);
        nvs_commit(h); nvs_close(h);
    }
}
static double nvs_load_odo(void) {
    double mi = 0; nvs_handle_t h; size_t sz = sizeof(mi);
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) { nvs_get_blob(h, "odo", &mi, &sz); nvs_close(h); }
    return mi;
}
static void nvs_save_odo(double mi) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "odo", &mi, sizeof(mi)); nvs_commit(h); nvs_close(h);
    }
}
static int nvs_load_seg(void) {
    nvs_handle_t h; uint8_t v = 0;
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) { nvs_get_u8(h, "logseg", &v); nvs_close(h); }
    return v % LOG_SEGS;
}
static void nvs_save_seg(int s) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) == ESP_OK) { nvs_set_u8(h, "logseg", (uint8_t)s); nvs_commit(h); nvs_close(h); }
}
/* generic small-value persistence for gamification state (namespace "cfg") */
static double nvs_load_d(const char *key, double def) {
    double v = def; nvs_handle_t h; size_t sz = sizeof(v);
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) { nvs_get_blob(h, key, &v, &sz); nvs_close(h); }
    return v;
}
static void nvs_save_d(const char *key, double v) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) == ESP_OK) { nvs_set_blob(h, key, &v, sizeof(v)); nvs_commit(h); nvs_close(h); }
}
static uint32_t nvs_load_u32(const char *key) {
    uint32_t v = 0; nvs_handle_t h;
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) { nvs_get_u32(h, key, &v); nvs_close(h); }
    return v;
}
static void nvs_save_u32(const char *key, uint32_t v) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) == ESP_OK) { nvs_set_u32(h, key, v); nvs_commit(h); nvs_close(h); }
}

static void ota_mark_valid(void) {
    static bool done = false;
    if (done) return;
    done = true;
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "OTA image marked valid");
    }
}

/* ------------------- segmented circular log ------------------- */
static void log_fs_init(void) {
    esp_vfs_littlefs_conf_t c = {
        .base_path = "/lfs", .partition_label = "storage",
        .format_if_mount_failed = true, .dont_mount = false,
    };
    esp_err_t e = esp_vfs_littlefs_register(&c);
    if (e != ESP_OK) { ESP_LOGE(TAG, "littlefs mount failed: %s", esp_err_to_name(e)); return; }
    size_t total = 0, used = 0; esp_littlefs_info("storage", &total, &used);
    g_seg = nvs_load_seg();
    g_log_records = 0;
    char p[24]; struct stat stt;
    for (int i = 0; i < LOG_SEGS; i++) { seg_path(i, p, sizeof(p)); if (stat(p, &stt) == 0) g_log_records += (uint32_t)(stt.st_size / sizeof(logrec_t)); }
    seg_path(g_seg, p, sizeof(p));
    g_logf = fopen(p, "ab");
    g_seg_bytes = (stat(p, &stt) == 0) ? (long)stt.st_size : 0;
    ESP_LOGI(TAG, "littlefs %uKB/%uKB used; log seg %d, %lu records total",
             (unsigned)(used / 1024), (unsigned)(total / 1024), g_seg, (unsigned long)g_log_records);
}
/* advance to the next segment, erasing it (drops the oldest data). Call with mutex held. */
static void log_advance(void) {
    int next = (g_seg + 1) % LOG_SEGS;
    char p[24]; seg_path(next, p, sizeof(p));
    struct stat stt;
    if (stat(p, &stt) == 0 && g_log_records >= stt.st_size / sizeof(logrec_t))
        g_log_records -= (uint32_t)(stt.st_size / sizeof(logrec_t));
    if (g_logf) { fclose(g_logf); g_logf = NULL; }
    remove(p);
    g_logf = fopen(p, "ab");
    g_seg = next; g_seg_bytes = 0;
    nvs_save_seg(next);
    ESP_LOGW(TAG, "log rollover -> segment %d (oldest dropped)", next);
}
static void log_append(int dmph) {
    xSemaphoreTake(g_log_mux, portMAX_DELAY);
    if (g_logf && g_seg_bytes + (long)sizeof(logrec_t) > LOG_SEG_BYTES) log_advance();
    if (g_logf) {
        time_t now = time(NULL);
        logrec_t r;
        r.ts = (now > (time_t)EPOCH_VALID) ? (uint32_t)now : 0;
        r.dmph = (uint16_t)(dmph < 0 ? 0 : dmph);
        r.flags = r.ts ? 1 : 0;
        uint8_t *b = (uint8_t *)&r, s = 0;
        for (int i = 0; i < 7; i++) s += b[i];
        r.ck = s;
        if (fwrite(&r, 1, sizeof(r), g_logf) == sizeof(r)) {
            fflush(g_logf); fsync(fileno(g_logf));
            g_seg_bytes += sizeof(r); g_log_records++;
        }
    }
    xSemaphoreGive(g_log_mux);
}

/* ------------------- UART decode + distance + logging ------------------- */
static void uart_task(void *arg) {
    uint8_t rx[128];
    uint8_t acc[64];
    int alen = 0;
    int64_t last_int = esp_timer_get_time();
    int64_t last_save = last_int, last_log = last_int, change_us = 0;
    double  last_saved = g_dist_mi;
    for (;;) {
        int n = uart_read_bytes(PAD_UART, rx, sizeof(rx), pdMS_TO_TICKS(50));
        int64_t now = esp_timer_get_time();
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (alen < (int)sizeof(acc)) acc[alen++] = rx[i];
                else { memmove(acc, acc + 1, sizeof(acc) - 1); acc[sizeof(acc) - 1] = rx[i]; }
            }
            int p = 0;
            while (alen - p >= 6) {
                if (acc[p] != 0xAA) { p++; continue; }
                uint8_t *f = &acc[p];
                if (f[5] == 0x55 && (uint8_t)(f[1] + f[2] + f[3]) == f[4]) {
                    uint8_t cmd = f[1];
                    uint16_t V = ((uint16_t)f[2] << 8) | f[3];
                    float sp = (cmd == 0x42) ? mph_from_V(V) : 0.0f;
                    taskENTER_CRITICAL(&mux);
                    g_cmd = cmd; g_rawV = V; g_speed_mph = sp;
                    g_frames++; g_last_frame_us = now;
                    taskEXIT_CRITICAL(&mux);
                    p += 6;
                } else {
                    p++;
                }
            }
            if (p > 0) { memmove(acc, acc + p, alen - p); alen -= p; }
        }
        int64_t dt = now - last_int; last_int = now;
        taskENTER_CRITICAL(&mux);
        if (g_last_frame_us && (now - g_last_frame_us) > STALE_US) g_speed_mph = 0.0f;
        double inc = (double)g_speed_mph * ((double)dt / 1e6) / 3600.0;
        g_dist_mi += inc;
        g_sess_mi += inc;
        g_life_mi += inc;                                        /* never reset */
        if (g_speed_mph > 0.0f) g_sess_s += (double)dt / 1e6;   /* moving time only */
        float cur = g_speed_mph;
        double snapshot = g_dist_mi;
        double lifesnap = g_life_mi;
        taskEXIT_CRITICAL(&mux);

        if (now - last_save > ODO_SAVE_US && snapshot - last_saved > 0.005) {
            nvs_save_odo(snapshot); nvs_save_d("lifemi", lifesnap);
            last_saved = snapshot; last_save = now;
        }

        int cur_dmph = (int)(cur * 10.0f + 0.5f);
        if (cur_dmph != g_logged_dmph) {
            if (change_us == 0) change_us = now;                 /* start debounce */
            if (now - change_us > LOG_DEBOUNCE_US) {
                log_append(cur_dmph); g_logged_dmph = cur_dmph;
                last_log = now; change_us = 0;
            }
        } else {
            change_us = 0;
            if (cur_dmph > 0 && now - last_log > LOG_HB_US) {    /* heartbeat while moving */
                log_append(cur_dmph); last_log = now;
            }
        }
    }
}

/* ------------------- HTTP ------------------- */
static esp_err_t status_get(httpd_req_t *req) {
    float sp; double mi, smi, ss; uint16_t V; uint8_t cmd; uint32_t fr; int64_t lf;
    taskENTER_CRITICAL(&mux);
    sp = g_speed_mph; mi = g_dist_mi; smi = g_sess_mi; ss = g_sess_s; V = g_rawV; cmd = g_cmd; fr = g_frames; lf = g_last_frame_us;
    taskEXIT_CRITICAL(&mux);
    double m = mi * 1609.34, sm = smi * 1609.34;
    int link = (lf && (esp_timer_get_time() - lf) < STALE_US) ? 1 : 0;
    wifi_ap_record_t ap; int rssi = 0;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    char buf[640];
    int len = snprintf(buf, sizeof(buf),
        "{\"speed_mph\":%.1f,\"speed_kmh\":%.2f,\"distance_mi\":%.3f,\"distance_m\":%.1f,"
        "\"steps_est\":%.0f,\"session_mi\":%.3f,\"session_steps\":%.0f,\"session_s\":%.0f,"
        "\"uptime_s\":%lld,\"raw_V\":%u,\"cmd\":\"0x%02X\",\"frames\":%lu,"
        "\"link\":\"%s\",\"rssi\":%d,\"ip\":\"%s\",\"ap_mode\":%s,\"heap\":%lu,"
        "\"log_records\":%lu,\"log_seg\":%d,\"time_synced\":%s,\"epoch\":%lld}",
        sp, sp * 1.60934, mi, m, m / STRIDE_M, smi, sm / STRIDE_M, ss,
        (long long)((esp_timer_get_time() - g_boot_us) / 1000000),
        V, cmd, (unsigned long)fr, link ? "ok" : "stale", rssi, g_ip,
        g_ap_active ? "true" : "false", (unsigned long)esp_get_free_heap_size(),
        (unsigned long)g_log_records, g_seg, g_time_synced ? "true" : "false", (long long)time(NULL));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t reset_get(httpd_req_t *req) {
    taskENTER_CRITICAL(&mux);
    g_dist_mi = 0.0;
    taskEXIT_CRITICAL(&mux);
    nvs_save_odo(0.0);
    httpd_resp_sendstr(req, "odometer reset");
    return ESP_OK;
}

/* stream the whole circular log as CSV (oldest->newest): epoch,utc,mph */
static esp_err_t logcsv_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=walk.csv");
    httpd_resp_sendstr_chunk(req, "epoch,utc,mph\n");
    xSemaphoreTake(g_log_mux, portMAX_DELAY);
    if (g_logf) fflush(g_logf);
    char p[24], line[64]; struct tm tmv; logrec_t r;
    for (int k = 1; k <= LOG_SEGS; k++) {           /* (g_seg+1) = oldest ... g_seg = newest */
        seg_path((g_seg + k) % LOG_SEGS, p, sizeof(p));
        FILE *rf = fopen(p, "rb");
        if (!rf) continue;
        while (fread(&r, 1, sizeof(r), rf) == sizeof(r)) {
            uint8_t *b = (uint8_t *)&r, s = 0; for (int i = 0; i < 7; i++) s += b[i];
            if (s != r.ck) continue;
            char iso[24] = "";
            if (r.ts) { time_t t = r.ts; gmtime_r(&t, &tmv); strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &tmv); }
            int nn = snprintf(line, sizeof(line), "%lu,%s,%.1f\n", (unsigned long)r.ts, iso, r.dmph / 10.0);
            httpd_resp_send_chunk(req, line, nn);
        }
        fclose(rf);
    }
    xSemaphoreGive(g_log_mux);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* days since the Unix epoch for a civil (Y,M,D) date — Howard Hinnant's algorithm.
   Lets us bucket by LOCAL calendar day/week/month without epoch/86400 rounding to UTC. */
static long days_from_civil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    long era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (long)doe - 719468;
}

/* reconstruct hours+miles+steps from the log: today / this week / this month / lifetime,
   bucketed by the LOCAL (Eastern) calendar; also computes the gamification "game" object
   (goal, streak, personal-best records, week momentum, activity dots, milestone) and a
   per-day array for the last 60 days. All-time records + longest streak persist in NVS. */
static esp_err_t stats_get(httpd_req_t *req) {
    time_t now = time(NULL);
    bool synced = now > (time_t)EPOCH_VALID;
    long ldn_now = 0, wk_now = 0, ym_now = 0;
    if (synced) {
        struct tm lt; localtime_r(&now, &lt);
        ldn_now = days_from_civil(lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);
        wk_now  = (ldn_now + 4) / 7;                 /* weeks start Sunday (epoch day 0 = Thu) */
        ym_now  = lt.tm_year * 12 + lt.tm_mon;
    }
    double tot_mi = 0, d_mi = 0, w_mi = 0, m_mi = 0;
    long   tot_s = 0,  d_s = 0,  w_s = 0,  m_s = 0, records = 0;
    static double day_mi[STAT_DAYS]; static long day_s[STAT_DAYS];
    memset(day_mi, 0, sizeof(day_mi)); memset(day_s, 0, sizeof(day_s));
    const int today_idx = STAT_DAYS - 1;             /* today is the last per-day bucket */

    xSemaphoreTake(g_log_mux, portMAX_DELAY);
    if (g_logf) fflush(g_logf);
    char p[24]; logrec_t r, prev; bool have = false;
    for (int k = 1; k <= LOG_SEGS; k++) {
        seg_path((g_seg + k) % LOG_SEGS, p, sizeof(p));
        FILE *rf = fopen(p, "rb");
        if (!rf) continue;
        while (fread(&r, 1, sizeof(r), rf) == sizeof(r)) {
            uint8_t *b = (uint8_t *)&r, s = 0; for (int i = 0; i < 7; i++) s += b[i];
            if (s != r.ck) continue;
            records++;
            if (have && prev.ts >= EPOCH_VALID && r.ts >= prev.ts) {
                long dt = (long)r.ts - (long)prev.ts;
                if (prev.dmph > 0 && dt > 0 && dt <= LOG_GAP_CAP_S) {   /* ignore power gaps */
                    double miles = (prev.dmph / 10.0) * dt / 3600.0;
                    tot_mi += miles; tot_s += dt;
                    if (synced) {
                        time_t pts = prev.ts; struct tm pt; localtime_r(&pts, &pt);
                        long ldn = days_from_civil(pt.tm_year + 1900, pt.tm_mon + 1, pt.tm_mday);
                        if (ldn == ldn_now)                        { d_mi += miles; d_s += dt; }
                        if ((ldn + 4) / 7 == wk_now)               { w_mi += miles; w_s += dt; }
                        if (pt.tm_year * 12 + pt.tm_mon == ym_now) { m_mi += miles; m_s += dt; }
                        long idx = ldn - (ldn_now - today_idx);
                        if (idx >= 0 && idx < STAT_DAYS) { day_mi[idx] += miles; day_s[idx] += dt; }
                    }
                }
            }
            prev = r; have = true;
        }
        fclose(rf);
    }
    xSemaphoreGive(g_log_mux);

    double spm = 1609.34 / STRIDE_M;   /* steps per mile */

    /* ---------- gamification ---------- */
    double goal_mi = nvs_load_d("goalmi", GOAL_MI_DEFAULT);
    double b7_mi = 0, b30_mi = 0, bw_mi = 0;   /* best-day distance: last 7d / 30d / whole window */
    long   b7_s = 0,  b30_s = 0,  bw_s = 0;    /* best-day time */
    long   bw_mi_idx = today_idx, bw_s_idx = today_idx;
    double lw_mi = 0;                          /* last calendar week miles */
    for (int i = 0; i < STAT_DAYS; i++) {
        int age = today_idx - i;               /* 0 = today */
        if (day_mi[i] > bw_mi) { bw_mi = day_mi[i]; bw_mi_idx = i; }
        if (day_s[i]  > bw_s)  { bw_s  = day_s[i];  bw_s_idx  = i; }
        if (age < 7)  { if (day_mi[i] > b7_mi)  b7_mi  = day_mi[i]; if (day_s[i] > b7_s)  b7_s  = day_s[i]; }
        if (age < 30) { if (day_mi[i] > b30_mi) b30_mi = day_mi[i]; if (day_s[i] > b30_s) b30_s = day_s[i]; }
        if (synced && (ldn_now - age + 4) / 7 == wk_now - 1) lw_mi += day_mi[i];
    }
    /* streaks vs the daily goal — computed fresh from the window so they stay consistent when the
       goal changes (window > any realistic streak; goal-relative, so NOT persisted like records):
       streak_best = longest goal-met run; streak = current run ending today (today pending is ok) */
    int streak = 0, streak_best = 0;
    if (synced) {
        int run = 0;
        for (int i = 0; i < STAT_DAYS; i++) {
            if (day_mi[i] >= goal_mi) { if (++run > streak_best) streak_best = run; }
            else run = 0;
        }
        int i = today_idx;
        if (day_mi[i] < goal_mi) i--;          /* today not met yet -> count back from yesterday */
        for (; i >= 0 && day_mi[i] >= goal_mi; i--) streak++;
        if (streak > streak_best) streak_best = streak;
    }
    /* 7-day activity dots (oldest -> newest) */
    int dots[7];
    for (int j = 0; j < 7; j++) { int i = today_idx - 6 + j; dots[j] = (i >= 0 && day_mi[i] >= ACTIVE_MI) ? 1 : 0; }
    /* lifetime miles + next round-number milestone */
    double life_mi = g_life_mi;
    double ms_next = ((double)((long)(life_mi / MILESTONE_MI)) + 1.0) * MILESTONE_MI;

    /* all-time records + longest streak: load, lazily bump, persist (rare writes) */
    double allrec_mi = nvs_load_d("recdmi", 0.0);
    long   allrec_mid = (long)nvs_load_u32("recdmid");
    long   allrec_s   = (long)nvs_load_u32("recds");
    long   allrec_sd  = (long)nvs_load_u32("recdsd");
    bool today_record = synced && day_mi[today_idx] > 0 &&
                        day_mi[today_idx] >= bw_mi && day_mi[today_idx] >= allrec_mi;
    if (synced) {
        if (bw_mi > allrec_mi + 1e-9) {
            allrec_mi = bw_mi; allrec_mid = ldn_now - (today_idx - bw_mi_idx);
            nvs_save_d("recdmi", allrec_mi); nvs_save_u32("recdmid", (uint32_t)allrec_mid);
        }
        if (bw_s > allrec_s) {
            allrec_s = bw_s; allrec_sd = ldn_now - (today_idx - bw_s_idx);
            nvs_save_u32("recds", (uint32_t)allrec_s); nvs_save_u32("recdsd", (uint32_t)allrec_sd);
        }
    }
    char amid[11] = "", asd[11] = "";
    if (allrec_mid) { time_t t = (time_t)(allrec_mid * 86400); struct tm tv; gmtime_r(&t, &tv); strftime(amid, sizeof(amid), "%Y-%m-%d", &tv); }
    if (allrec_sd)  { time_t t = (time_t)(allrec_sd  * 86400); struct tm tv; gmtime_r(&t, &tv); strftime(asd,  sizeof(asd),  "%Y-%m-%d", &tv); }

    char *out = malloc(8192);
    if (!out) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_FAIL; }
    int o = snprintf(out, 8192,
        "{\"today\":{\"hours\":%.3f,\"miles\":%.3f,\"steps\":%.0f},"
        "\"week\":{\"hours\":%.3f,\"miles\":%.3f,\"steps\":%.0f},"
        "\"month\":{\"hours\":%.3f,\"miles\":%.3f,\"steps\":%.0f},"
        "\"total_hours\":%.3f,\"total_miles\":%.3f,\"total_steps\":%.0f,"
        "\"records\":%ld,\"synced\":%s,"
        "\"game\":{\"goal_mi\":%.2f,\"streak\":%d,\"streak_best\":%d,\"today_record\":%s,"
        "\"best\":{\"d7\":{\"mi\":%.3f,\"s\":%ld},\"d30\":{\"mi\":%.3f,\"s\":%ld},"
        "\"all\":{\"mi\":%.3f,\"mi_date\":\"%s\",\"s\":%ld,\"s_date\":\"%s\"}},"
        "\"week_mi\":%.3f,\"lastweek_mi\":%.3f,"
        "\"life_mi\":%.3f,\"milestone_step\":%.0f,\"milestone_next\":%.0f,"
        "\"dots\":[%d,%d,%d,%d,%d,%d,%d]},\"days\":[",
        d_s / 3600.0, d_mi, d_mi * spm,
        w_s / 3600.0, w_mi, w_mi * spm,
        m_s / 3600.0, m_mi, m_mi * spm,
        tot_s / 3600.0, tot_mi, tot_mi * spm,
        records, synced ? "true" : "false",
        goal_mi, streak, streak_best, today_record ? "true" : "false",
        b7_mi, b7_s, b30_mi, b30_s,
        allrec_mi, amid, allrec_s, asd,
        w_mi, lw_mi,
        life_mi, (double)MILESTONE_MI, ms_next,
        dots[0], dots[1], dots[2], dots[3], dots[4], dots[5], dots[6]);
    bool first = true;
    for (int i = STAT_DAYS - 60; i < STAT_DAYS && synced; i++) {
        if (i < 0 || day_s[i] == 0) continue;
        long ldn = ldn_now - (today_idx - i);
        time_t t = (time_t)(ldn * 86400); struct tm tmv; gmtime_r(&t, &tmv);
        char d[11]; strftime(d, sizeof(d), "%Y-%m-%d", &tmv);
        o += snprintf(out + o, 8192 - o, "%s{\"d\":\"%s\",\"hours\":%.3f,\"miles\":%.3f}",
                      first ? "" : ",", d, day_s[i] / 3600.0, day_mi[i]);
        first = false;
        if (o > 7900) break;
    }
    o += snprintf(out + o, 8192 - o, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, out, o);
    free(out);
    return ESP_OK;
}

/* GET /goal -> read current daily goal; GET /goal?mi=2.5 -> set + persist (0.1..50 mi) */
static esp_err_t goal_get(httpd_req_t *req) {
    char q[64], v[16];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "mi", v, sizeof(v)) == ESP_OK) {
        double mi = atof(v);
        if (mi >= 0.1 && mi <= 50.0) nvs_save_d("goalmi", mi);
    }
    char buf[48];
    int n = snprintf(buf, sizeof(buf), "{\"goal_mi\":%.2f}", nvs_load_d("goalmi", GOAL_MI_DEFAULT));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t logreset_get(httpd_req_t *req) {
    xSemaphoreTake(g_log_mux, portMAX_DELAY);
    if (g_logf) { fclose(g_logf); g_logf = NULL; }
    char p[24];
    for (int i = 0; i < LOG_SEGS; i++) { seg_path(i, p, sizeof(p)); remove(p); }
    g_seg = 0; g_seg_bytes = 0; g_log_records = 0; g_logged_dmph = -1;
    seg_path(0, p, sizeof(p)); g_logf = fopen(p, "ab");
    nvs_save_seg(0);
    xSemaphoreGive(g_log_mux);
    httpd_resp_sendstr(req, "log cleared");
    return ESP_OK;
}

/* OTA: raw firmware .bin POSTed as the request body */
static esp_err_t update_post(httpd_req_t *req) {
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t ota = 0;
    if (!part || esp_ota_begin(part, OTA_SIZE_UNKNOWN, &ota) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin failed");
        return ESP_FAIL;
    }
    char *buf = malloc(2048);
    if (!buf) { esp_ota_abort(ota); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_FAIL; }
    int remaining = req->content_len, total = 0;
    while (remaining > 0) {
        int r = httpd_req_recv(req, buf, MINI(remaining, 2048));
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) { free(buf); esp_ota_abort(ota); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv error"); return ESP_FAIL; }
        if (esp_ota_write(ota, buf, r) != ESP_OK) { free(buf); esp_ota_abort(ota); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_write failed"); return ESP_FAIL; }
        remaining -= r; total += r;
    }
    free(buf);
    if (esp_ota_end(ota) != ESP_OK) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_end failed (bad image?)"); return ESP_FAIL; }
    if (esp_ota_set_boot_partition(part) != ESP_OK) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_boot failed"); return ESP_FAIL; }
    ESP_LOGI(TAG, "OTA received %d bytes, rebooting", total);
    httpd_resp_sendstr(req, "OTA OK — rebooting into new firmware");
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
    return ESP_OK;
}

static int hexv(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
static void urldecode(char *s) {
    char *o = s;
    for (char *p = s; *p; p++) {
        if (*p == '%' && p[1] && p[2]) { int hi = hexv(p[1]), lo = hexv(p[2]); if (hi >= 0 && lo >= 0) { *o++ = (char)((hi << 4) | lo); p += 2; continue; } }
        *o++ = (*p == '+') ? ' ' : *p;
    }
    *o = 0;
}
static esp_err_t wifi_post(httpd_req_t *req) {
    char buf[256];
    int len = MINI(req->content_len, (int)sizeof(buf) - 1);
    int r = httpd_req_recv(req, buf, len);
    if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body"); return ESP_FAIL; }
    buf[r] = 0;
    char ssid[33] = {0}, pass[65] = {0};
    httpd_query_key_value(buf, "ssid", ssid, sizeof(ssid));
    httpd_query_key_value(buf, "pass", pass, sizeof(pass));
    urldecode(ssid); urldecode(pass);
    if (strlen(ssid) < 1) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid required"); return ESP_FAIL; }
    nvs_save_creds(ssid, pass);
    httpd_resp_sendstr(req, "Saved. Rebooting to join the new network...");
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
    return ESP_OK;
}

static const char PAGE[] =
"<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Walking Pad</title><style>"
"body{font-family:system-ui,sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:2rem}"
".big{font-size:4rem;font-weight:700;margin:.15em}.u{opacity:.6;font-size:.95rem}"
"h3.sec{margin:1.5rem 0 .3rem;opacity:.55;font-weight:600;letter-spacing:.08em;text-transform:uppercase;font-size:.8rem}"
".row{display:flex;justify-content:center;gap:.7rem;margin-top:.4rem;flex-wrap:wrap}"
".card{background:#1c1c1c;padding:.75rem 1rem;border-radius:12px;min-width:4.3rem}"
".card .big{font-size:2rem;margin:.1em}"
"button{margin:.4rem;padding:.5rem 1rem;border:0;border-radius:8px;background:#2d6;color:#000;font-weight:600}"
"a{color:#6cf}.stale{color:#e55}small{opacity:.5}"
".gm{font-size:1.05rem;margin-top:.5rem}"
".bar{height:.5rem;background:#2a2a2a;border-radius:6px;overflow:hidden;max-width:20rem;margin:.35rem auto}"
".bar>i{display:block;height:100%;width:0;background:#2d6;transition:width .4s}"
".up{color:#2d6}.dn{color:#e88}.flat{opacity:.55}"
"#dots{letter-spacing:.3rem;font-size:1.15rem;margin-top:.2rem}"
"input[type=number]{width:4rem}</style>"
"<h2>&#128694; Walking Pad</h2>"
"<div class=big id=spd>--</div><div class=u>mph now (<span id=kmh>--</span> km/h)</div>"
"<div class=gm id=streak></div>"
"<div class=bar><i id=goalfill></i></div><div class=u id=goaltxt></div>"
"<h3 class=sec>This Session</h3><div class=row>"
"<div class=card><div class=big id=s_mi>--</div><div class=u>miles</div></div>"
"<div class=card><div class=big id=s_tm>--</div><div class=u>time</div></div>"
"<div class=card><div class=big id=s_av>--</div><div class=u>avg mph</div></div>"
"<div class=card><div class=big id=s_st>--</div><div class=u>steps</div></div></div>"
"<h3 class=sec>Today <span id=pr></span></h3><div class=row>"
"<div class=card><div class=big id=d_mi>--</div><div class=u>miles</div></div>"
"<div class=card><div class=big id=d_tm>--</div><div class=u>time</div></div>"
"<div class=card><div class=big id=d_av>--</div><div class=u>avg mph</div></div>"
"<div class=card><div class=big id=d_st>--</div><div class=u>steps</div></div></div>"
"<h3 class=sec>This Week</h3><div class=row>"
"<div class=card><div class=big id=w_mi>--</div><div class=u>miles</div></div>"
"<div class=card><div class=big id=w_tm>--</div><div class=u>time</div></div>"
"<div class=card><div class=big id=w_av>--</div><div class=u>avg mph</div></div>"
"<div class=card><div class=big id=w_st>--</div><div class=u>steps</div></div></div>"
"<div class=u id=mom style=margin-top:.35rem></div><div id=dots></div>"
"<h3 class=sec>This Month</h3><div class=row>"
"<div class=card><div class=big id=m_mi>--</div><div class=u>miles</div></div>"
"<div class=card><div class=big id=m_tm>--</div><div class=u>time</div></div>"
"<div class=card><div class=big id=m_av>--</div><div class=u>avg mph</div></div>"
"<div class=card><div class=big id=m_st>--</div><div class=u>steps</div></div></div>"
"<h3 class=sec>Lifetime</h3><div class=row>"
"<div class=card><div class=big id=t_mi>--</div><div class=u>miles</div></div>"
"<div class=card><div class=big id=t_tm>--</div><div class=u>time</div></div>"
"<div class=card><div class=big id=t_av>--</div><div class=u>avg mph</div></div>"
"<div class=card><div class=big id=t_st>--</div><div class=u>steps</div></div></div>"
"<div class=u id=msln style=margin-top:.4rem></div><div class=bar><i id=msfill></i></div>"
"<h3 class=sec>Best Day &mdash; distance</h3><div class=row>"
"<div class=card><div class=big id=bd_7>--</div><div class=u>last 7d</div></div>"
"<div class=card><div class=big id=bd_30>--</div><div class=u>last 30d</div></div>"
"<div class=card><div class=big id=bd_all>--</div><div class=u id=bd_ald>all-time</div></div></div>"
"<h3 class=sec>Longest Day &mdash; time</h3><div class=row>"
"<div class=card><div class=big id=bt_7>--</div><div class=u>last 7d</div></div>"
"<div class=card><div class=big id=bt_30>--</div><div class=u>last 30d</div></div>"
"<div class=card><div class=big id=bt_all>--</div><div class=u id=bt_ald>all-time</div></div></div>"
"<div class=u id=meta style=margin-top:1rem></div>"
"<div style=margin-top:1rem><a href=/log.csv>download log CSV</a> &nbsp; <a href=/stats>stats JSON</a></div>"
"<div style=margin-top:1rem class=u>daily goal <input id=goalin type=number step=0.1 min=0.1> mi "
"<button style='padding:.3rem .6rem' onclick=setgoal()>set</button></div>"
"<div style=margin-top:.6rem><button onclick=\"fetch('/reset')\">reset odometer</button></div>"
"<div style=margin-top:1.5rem><small>firmware update (.bin):</small><br>"
"<input type=file id=fw><button onclick=up()>flash</button> &nbsp;<a href=/setup>wifi setup</a></div>"
"<script>function gid(i){return document.getElementById(i);}"
"function dur(h){var m=Math.round(h*60);return m<60?m+'m':Math.floor(m/60)+'h '+String(m%60).padStart(2,'0')+'m';}"
"function fill(p,o){gid(p+'_mi').textContent=o.miles.toFixed(2);"
"gid(p+'_tm').textContent=dur(o.hours);"
"gid(p+'_av').textContent=(o.hours>0?o.miles/o.hours:0).toFixed(1);"
"gid(p+'_st').textContent=o.steps.toFixed(0);}"
"async function u(){try{let d=await(await fetch('/status')).json();"
"spd.textContent=d.speed_mph.toFixed(1);kmh.textContent=d.speed_kmh.toFixed(2);"
"fill('s',{miles:d.session_mi,hours:d.session_s/3600,steps:d.session_steps});"
"meta.textContent='V='+d.raw_V+' cmd='+d.cmd+' link='+d.link+' rssi='+d.rssi+' ip='+d.ip+' odo='+d.distance_mi.toFixed(2)+'mi log='+d.log_records+' up '+d.uptime_s+'s';"
"meta.className=d.link=='ok'?'u':'u stale';}catch(e){meta.textContent='no connection';}}"
"async function s(){try{let d=await(await fetch('/stats')).json();"
"fill('d',d.today);fill('w',d.week);fill('m',d.month);"
"fill('t',{miles:d.total_miles,hours:d.total_hours,steps:d.total_steps});game(d);}catch(e){}}"
"function game(d){var g=d.game;if(!g)return;"
"streak.textContent='\\u{1F525} '+g.streak+' day streak \\u00b7 best '+g.streak_best;"
"var goal=g.goal_mi,tm=d.today.miles,pct=goal>0?Math.min(100,tm/goal*100):0;"
"goalfill.style.width=pct+'%';"
"goaltxt.textContent=tm.toFixed(2)+' / '+goal.toFixed(1)+' mi today'+(tm>=goal?' \\u2713':'');"
"if(goalin.value=='')goalin.value=goal.toFixed(1);"
"pr.textContent=g.today_record?' \\u{1F3C6} best day!':'';"
"bd_7.textContent=g.best.d7.mi.toFixed(2);bd_30.textContent=g.best.d30.mi.toFixed(2);"
"bd_all.textContent=g.best.all.mi.toFixed(2);bd_ald.textContent='all-time'+(g.best.all.mi_date?' \\u00b7 '+g.best.all.mi_date:'');"
"bt_7.textContent=dur(g.best.d7.s/3600);bt_30.textContent=dur(g.best.d30.s/3600);"
"bt_all.textContent=dur(g.best.all.s/3600);bt_ald.textContent='all-time'+(g.best.all.s_date?' \\u00b7 '+g.best.all.s_date:'');"
"var lw=g.lastweek_mi,tw=g.week_mi;"
"if(lw>0){var pp=Math.round((tw-lw)/lw*100);mom.textContent=(pp>=0?'\\u25b2 '+pp:'\\u25bc '+(-pp))+'% vs last week';mom.className='u '+(pp>0?'up':pp<0?'dn':'flat');}"
"else{mom.textContent=tw>0?'first week \\u2014 keep it going':'';mom.className='u flat';}"
"dots.textContent=g.dots.map(function(x){return x?'\\u25cf':'\\u25cb';}).join(' ');"
"var life=g.life_mi,step=g.milestone_step,nx=g.milestone_next,prv=nx-step;"
"var mp=step>0?Math.min(100,(life-prv)/step*100):0;"
"msln.textContent=life.toFixed(1)+' mi lifetime \\u00b7 '+(nx-life).toFixed(1)+' to your '+nx+'-mi marker';"
"msfill.style.width=mp+'%';}"
"async function setgoal(){var v=parseFloat(goalin.value);if(!(v>0))return;await fetch('/goal?mi='+v);s();}"
"async function up(){let f=fw.files[0];if(!f){alert('pick a .bin');return;}"
"if(!confirm('flash '+f.name+' ('+f.size+' bytes)?'))return;"
"let r=await fetch('/update',{method:'POST',body:f});alert(await r.text());}"
"setInterval(u,500);u();setInterval(s,10000);s();</script>";

static esp_err_t root_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static const char SETUP[] =
"<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>WiFi setup</title><style>body{font-family:system-ui;background:#111;color:#eee;text-align:center;padding:2rem}"
"input{display:block;margin:.5rem auto;padding:.5rem;width:16rem}button{padding:.5rem 1rem;margin-top:.5rem}</style>"
"<h3>Walking Pad — WiFi setup</h3><form method=POST action=/wifi>"
"<input name=ssid placeholder=SSID maxlength=32 required>"
"<input name=pass placeholder=password type=password maxlength=63>"
"<button>save &amp; reboot</button></form>";
static esp_err_t setup_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, SETUP, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void start_http(void) {
    static bool started = false;
    if (started) return;
    httpd_handle_t s = NULL;
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.stack_size = 8192;
    c.max_uri_handlers = 12;
    c.lru_purge_enable = true;
    if (httpd_start(&s, &c) == ESP_OK) {
        httpd_uri_t h[] = {
            { .uri = "/",          .method = HTTP_GET,  .handler = root_get     },
            { .uri = "/status",    .method = HTTP_GET,  .handler = status_get   },
            { .uri = "/reset",     .method = HTTP_GET,  .handler = reset_get     },
            { .uri = "/setup",     .method = HTTP_GET,  .handler = setup_get     },
            { .uri = "/log.csv",   .method = HTTP_GET,  .handler = logcsv_get    },
            { .uri = "/stats",     .method = HTTP_GET,  .handler = stats_get     },
            { .uri = "/goal",      .method = HTTP_GET,  .handler = goal_get      },
            { .uri = "/log/reset", .method = HTTP_GET,  .handler = logreset_get  },
            { .uri = "/update",    .method = HTTP_POST, .handler = update_post   },
            { .uri = "/wifi",      .method = HTTP_POST, .handler = wifi_post     },
        };
        for (int i = 0; i < (int)(sizeof(h) / sizeof(h[0])); i++) httpd_register_uri_handler(s, &h[i]);
        started = true;
        ESP_LOGI(TAG, "http server up");
    }
}

/* ------------------- WiFi + time ------------------- */
static void sntp_start(void) {
    static bool done = false;
    if (done) return;
    done = true;
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started (%s)", NTP_SERVER);
}
static void time_sync_cb(struct timeval *tv) { g_time_synced = true; ESP_LOGI(TAG, "time synced: %lld", (long long)time(NULL)); }

static void start_setup_ap(void) {
    if (g_ap_active) return;
    g_ap_active = true;
    ESP_LOGW(TAG, "no STA connection — opening 'walkingpad-setup' AP (192.168.4.1)");
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    wifi_config_t ap = {0};
    strcpy((char *)ap.ap.ssid, "walkingpad-setup");
    ap.ap.ssid_len = strlen("walkingpad-setup");
    strcpy((char *)ap.ap.password, "walkingpad");
    ap.ap.max_connection = 2;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.channel = 1;
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    ota_mark_valid();
}
static void fallback_cb(void *arg) { if (!g_connected) start_setup_ap(); }

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_connected = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(g_ip, sizeof(g_ip), IPSTR, IP2STR(&e->ip_info.ip));
        g_connected = true;
        ESP_LOGI(TAG, "got IP %s  ->  http://%s.local/  (or http://%s/)", g_ip, HOSTNAME, g_ip);
        ota_mark_valid();
        sntp_start();
    }
}

static void mdns_start(void) {
    if (mdns_init() != ESP_OK) return;
    mdns_hostname_set(HOSTNAME);
    mdns_instance_name_set("Walking Pad Bridge");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS: http://%s.local/", HOSTNAME);
}

static void wifi_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi, NULL, NULL));
    sntp_set_time_sync_notification_cb(&time_sync_cb);

    char ssid[33], pass[65];
    if (!nvs_load_creds(ssid, sizeof(ssid), pass, sizeof(pass))) {
        strlcpy(ssid, WIFI_SSID, sizeof(ssid));
        strlcpy(pass, WIFI_PASS, sizeof(pass));
    }
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    const esp_timer_create_args_t ta = { .callback = &fallback_cb, .name = "apfb" };
    esp_timer_create(&ta, &g_fallback_timer);
    esp_timer_start_once(g_fallback_timer, AP_FALLBACK_US);
}

static void pad_uart_start(void) {
    uart_config_t uc = {
        .baud_rate = PAD_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PAD_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PAD_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(PAD_UART, UART_PIN_NO_CHANGE, PAD_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(uart_task, "pad_uart", 4096, NULL, 10, NULL);
}

void app_main(void) {
    g_boot_us = esp_timer_get_time();
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }
    setenv("TZ", LOCAL_TZ, 1); tzset();     /* records store UTC epoch; /stats buckets by local day */
    g_log_mux = xSemaphoreCreateMutex();
    log_fs_init();
    g_dist_mi = nvs_load_odo();
    g_life_mi = nvs_load_d("lifemi", -1.0);          /* seed once from best available figure */
    if (g_life_mi < 0.0) { g_life_mi = g_dist_mi; nvs_save_d("lifemi", g_life_mi); }
    pad_uart_start();
    wifi_start();
    start_http();
    mdns_start();
    ESP_LOGI(TAG, "walkingpad-bridge up: UART2 RX=GPIO%d @ %d 8N1, odo=%.3f mi, log=%lu recs (seg %d)",
             PAD_RX_GPIO, PAD_BAUD, g_dist_mi, (unsigned long)g_log_records, g_seg);
}
