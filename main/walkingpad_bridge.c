/*
 * walkingpad-bridge — taps the walking pad's console->drive UART (pin 5, 2400 8N1),
 * decodes speed, integrates distance, and serves it as JSON + a tiny web page over WiFi.
 *
 * Frame: AA <cmd> <HH> <LL> <CK> 55     (CK = (cmd + HH + LL) & 0xFF)
 *   cmd 0x42 = run-at-speed, value V = HH*256 + LL, mph = (V - 105) / 160
 *   cmd 0x40 / 0xA2 = idle/stopped -> 0 mph
 * Distance = integral of speed*dt (same as the console). Steps NOT on the wire; est only.
 *
 * Wiring: pin5 --(4.7k series + 10k to GND, ~3.4V)--> RX2/GPIO16 (UART2 RX); pin1 -> GND;
 *         pad 5V rail -> ESP32 5V/VIN. RX only, no TX wire. (Passive divider — do NOT use
 *         an active/bidirectional level shifter here; it back-drives pin5 and trips E07.)
 *
 * Sealed-unit features: OTA (/update, dual-slot + rollback), mDNS (walkingpad.local),
 * WiFi setup AP fallback, distance persisted to NVS.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
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
#define STRIDE_M    0.67               /* metres/step (rough; tune to taste) */
/* --------------------------------------------------- */

#define PAD_UART       UART_NUM_2
#define PAD_RX_GPIO    16              /* = the board's RX2 pin */
#define PAD_BAUD       2400
#define STALE_US       2000000LL       /* no valid frame for 2s -> speed 0 */
#define AP_FALLBACK_US 30000000LL      /* no STA connect in 30s -> open setup AP */
#define ODO_SAVE_US    30000000LL      /* persist distance at most every 30s */

static const char *TAG = "pad";

#define MINI(a,b) ((a) < (b) ? (a) : (b))

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static float    g_speed_mph = 0.0f;
static double   g_dist_mi   = 0.0;
static uint16_t g_rawV = 0;
static uint8_t  g_cmd  = 0;
static uint32_t g_frames = 0;
static int64_t  g_last_frame_us = 0;
static int64_t  g_boot_us = 0;
static volatile bool g_connected = false;
static volatile bool g_ap_active = false;
static char     g_ip[16] = "0.0.0.0";
static esp_timer_handle_t g_fallback_timer;

static inline float mph_from_V(uint16_t V) {
    float m = ((float)V - 105.0f) / 160.0f;
    return m < 0.0f ? 0.0f : m;
}

/* ------------------- NVS: creds + odometer ------------------- */
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

/* mark a freshly-OTA'd image good so the bootloader won't roll it back */
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

/* ------------------- UART decode + distance ------------------- */
static void uart_task(void *arg) {
    uint8_t rx[128];
    uint8_t acc[64];
    int alen = 0;
    int64_t last_int = esp_timer_get_time();
    int64_t last_save = last_int;
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
                    /* only cmd 0x42 carries a speed setpoint; all other cmds
                       (0x40/0xA2/0x50 idle polls, etc.) mean not running -> 0 */
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
        g_dist_mi += (double)g_speed_mph * ((double)dt / 1e6) / 3600.0;
        double snapshot = g_dist_mi;
        taskEXIT_CRITICAL(&mux);
        /* persist odometer periodically if it moved */
        if (now - last_save > ODO_SAVE_US && snapshot - last_saved > 0.005) {
            nvs_save_odo(snapshot); last_saved = snapshot; last_save = now;
        }
    }
}

/* ------------------- HTTP ------------------- */
static esp_err_t status_get(httpd_req_t *req) {
    float sp; double mi; uint16_t V; uint8_t cmd; uint32_t fr; int64_t lf;
    taskENTER_CRITICAL(&mux);
    sp = g_speed_mph; mi = g_dist_mi; V = g_rawV; cmd = g_cmd; fr = g_frames; lf = g_last_frame_us;
    taskEXIT_CRITICAL(&mux);
    double m = mi * 1609.34;
    int link = (lf && (esp_timer_get_time() - lf) < STALE_US) ? 1 : 0;
    wifi_ap_record_t ap; int rssi = 0;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    char buf[420];
    int len = snprintf(buf, sizeof(buf),
        "{\"speed_mph\":%.1f,\"speed_kmh\":%.2f,\"distance_mi\":%.3f,\"distance_m\":%.1f,"
        "\"steps_est\":%.0f,\"uptime_s\":%lld,\"raw_V\":%u,\"cmd\":\"0x%02X\",\"frames\":%lu,"
        "\"link\":\"%s\",\"rssi\":%d,\"ip\":\"%s\",\"ap_mode\":%s,\"heap\":%lu}",
        sp, sp * 1.60934, mi, m, m / STRIDE_M,
        (long long)((esp_timer_get_time() - g_boot_us) / 1000000),
        V, cmd, (unsigned long)fr, link ? "ok" : "stale", rssi, g_ip,
        g_ap_active ? "true" : "false", (unsigned long)esp_get_free_heap_size());
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
    httpd_resp_sendstr(req, "reset ok");
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

/* setup AP: save new WiFi creds and reboot */
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
".big{font-size:4rem;font-weight:700;margin:.15em}.u{opacity:.6;font-size:1rem}"
".row{display:flex;justify-content:center;gap:1.5rem;margin-top:1rem;flex-wrap:wrap}"
".card{background:#1c1c1c;padding:1rem 1.5rem;border-radius:12px}"
"button{margin:.4rem;padding:.5rem 1rem;border:0;border-radius:8px;background:#2d6;color:#000;font-weight:600}"
"a{color:#6cf}.stale{color:#e55}small{opacity:.5}</style>"
"<h2>&#128694; Walking Pad</h2>"
"<div class=big id=spd>--</div><div class=u>mph (<span id=kmh>--</span> km/h)</div>"
"<div class=row><div class=card><div class=big id=dist>--</div><div class=u>miles</div></div>"
"<div class=card><div class=big id=steps>--</div><div class=u>steps (est)</div></div></div>"
"<div class=u id=meta style=margin-top:1rem></div>"
"<div style=margin-top:1rem><button onclick=\"fetch('/reset')\">reset distance</button></div>"
"<div style=margin-top:1.5rem><small>firmware update (.bin):</small><br>"
"<input type=file id=fw><button onclick=up()>flash</button> "
"&nbsp;<a href=/setup>wifi setup</a></div>"
"<script>async function u(){try{let d=await(await fetch('/status')).json();"
"spd.textContent=d.speed_mph.toFixed(1);kmh.textContent=d.speed_kmh.toFixed(2);"
"dist.textContent=d.distance_mi.toFixed(3);steps.textContent=d.steps_est;"
"meta.textContent='V='+d.raw_V+' cmd='+d.cmd+' link='+d.link+' rssi='+d.rssi+' ip='+d.ip+' up '+d.uptime_s+'s';"
"meta.className=d.link=='ok'?'u':'u stale';}catch(e){meta.textContent='no connection';}}"
"async function up(){let f=fw.files[0];if(!f){alert('pick a .bin');return;}"
"if(!confirm('flash '+f.name+' ('+f.size+' bytes)?'))return;"
"let r=await fetch('/update',{method:'POST',body:f});alert(await r.text());}"
"setInterval(u,500);u();</script>";

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
    c.max_uri_handlers = 10;
    c.lru_purge_enable = true;
    if (httpd_start(&s, &c) == ESP_OK) {
        httpd_uri_t h[] = {
            { .uri = "/",       .method = HTTP_GET,  .handler = root_get   },
            { .uri = "/status", .method = HTTP_GET,  .handler = status_get },
            { .uri = "/reset",  .method = HTTP_GET,  .handler = reset_get  },
            { .uri = "/setup",  .method = HTTP_GET,  .handler = setup_get  },
            { .uri = "/update", .method = HTTP_POST, .handler = update_post},
            { .uri = "/wifi",   .method = HTTP_POST, .handler = wifi_post  },
        };
        for (int i = 0; i < (int)(sizeof(h) / sizeof(h[0])); i++) httpd_register_uri_handler(s, &h[i]);
        started = true;
        ESP_LOGI(TAG, "http server up");
    }
}

/* ------------------- WiFi ------------------- */
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
    ota_mark_valid();   /* reachable via AP -> not bricked */
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
    g_dist_mi = nvs_load_odo();
    pad_uart_start();
    wifi_start();
    start_http();     /* binds 0.0.0.0 — serves on STA and/or AP */
    mdns_start();
    ESP_LOGI(TAG, "walkingpad-bridge up: UART2 RX=GPIO%d @ %d 8N1, odo=%.3f mi",
             PAD_RX_GPIO, PAD_BAUD, g_dist_mi);
}
