# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ESP-IDF (v5.4) firmware for an ESP32-D0WD (WROOM) that **listens** to a walking pad's internal
console→drive UART (pad connector pin 5, 2400 8N1, 5 V logic, tapped on GPIO16/UART2 RX), decodes
the speed setpoint, integrates distance, logs speed changes to flash, and serves a dashboard plus
a JSON API over WiFi at `http://walkingpad.local/`. This is a public portfolio repo.
`README.md` is the public write-up: the reverse-engineering story, the tap, the firmware design,
and how the project was built with Claude Code. `docs/REFERENCE.md` is the technical reference:
every endpoint and JSON field, the log format, the flash layout, and the ops/recovery runbook.

The ESP32 is **sealed inside the pad**. The normal way to deploy is OTA over WiFi. USB flashing
means opening the case, and there is usually no serial console to watch.

## Commands

The build and flash wrappers live outside this repo in `~/Workspace/claude/mcu-tools/`. They
source `mcu_env.sh`, which activates the local ESP-IDF at `~/Workspace/claude/esp-idf`. The
`mcu-build` / `mcu-flash` / `mcu-detect` skills wrap the same scripts.

```bash
# Build -> build/walkingpad_bridge.bin (runs `idf.py set-target esp32` first if sdkconfig is missing)
~/Workspace/claude/mcu-tools/mcu_build.sh esp32 ~/Workspace/treadmill/walkingpad-bridge

# Deploy over WiFi (normal path). A bad image auto-rolls back.
curl --data-binary @build/walkingpad_bridge.bin http://walkingpad.local/update

# Verify on the live device
curl -s http://walkingpad.local/status   # live telemetry, link ok/stale, log_records, time_synced
curl -s http://walkingpad.local/stats    # reconstructed history + "game" object

# USB flash (first time / recovery / partition-table changes). Must use 115200 because of the marginal crystal.
ESP32_PORT=/dev/ttyUSB1 ESP32_FLASH_BAUD=115200 \
  ~/Workspace/claude/mcu-tools/mcu_flash.sh esp32 ~/Workspace/treadmill/walkingpad-bridge

# Serial console (USB only, 115200)
~/Workspace/claude/mcu-tools/read_serial.py /dev/ttyUSBx --seconds 10

# Bench decoders that use a 5 V FTDI on pin 5 instead of the ESP32 (need pyserial)
python3 tools/speedo.py [port] [stride_m]
python3 tools/framemon.py [port] [baud] [seconds]
```

USB ports re-enumerate. Find boards by VID with `mcu_detect.py`: the ESP32 is CP210x `10c4`, the
FTDI is `0403`.

There is no test suite or linter. "Verified" means it builds cleanly and, when the device is
reachable, it OTAs and `/status` plus `/stats` return sane JSON afterwards.

## Build configuration

- `sdkconfig` is generated and gitignored, so put config changes in `sdkconfig.defaults`. An
  existing `sdkconfig` does not pick up edits to the defaults. Delete it so the next build
  regenerates it (this also makes `set-target` clear `build/`).
- `CONFIG_ESP32_XTAL_FREQ_40=y` is pinned on purpose. This board's crystal is marginal, and
  without the pin some boots derive the clock from 26 MHz, which shifts every baud rate.
- `partitions.csv` holds `factory` (USB-only golden image) + `ota_0` + `ota_1` + `storage`
  (~960 KB LittleFS for the log). **Any change to the partition table needs a USB flash.** OTA
  cannot apply it.
- Managed components (`espressif/mdns`, `joltwallet/littlefs`) are declared in
  `main/idf_component.yml` and pinned in `dependencies.lock`. `managed_components/` is gitignored.
- Secrets go in gitignored `main/wifi_secrets.h` (`WIFI_SSID` / `WIFI_PASS`, plus optional
  `SETUP_AP_PASS`), picked up via `__has_include`. Without that file the build falls back to
  placeholder or default values. Credentials saved in NVS through the setup AP override the
  compiled ones. Don't print the contents of that file or copy them anywhere tracked.

## Firmware architecture (`main/walkingpad_bridge.c`)

All of the firmware is this one file, top to bottom: config `#define`s → NVS helpers → circular
log → UART task → HTTP handlers → embedded HTML/JS → WiFi/SNTP/mDNS → `app_main`.

**Data flow.** `uart_task` (prio 10) is the only producer. It byte-syncs on
`AA <cmd> HH LL CK 55` with a checksum, so it never relies on idle gaps (the Python tools frame
by idle gap instead). Any cmd other than `0x42` means 0 mph, and no valid frame for `STALE_US`
(2 s) also forces speed to 0. Every ~50 ms loop it integrates `speed·dt` into three counters,
writes the odometer to NVS at most every 30 s, and appends a log record when the speed changes
(0.1 mph resolution, 0.4 s debounce) or every 4 min while moving (heartbeat).

**Concurrency.** There are two locks, and they are not interchangeable:
- `mux` (a `portMUX` spinlock, `taskENTER_CRITICAL`) guards the live telemetry globals
  (`g_speed_mph`, `g_dist_mi`, `g_sess_*`, `g_life_mi`, `g_rawV`, `g_cmd`, `g_frames`, ...).
  Hold it only long enough to snapshot or update values. Never do NVS, file, or network I/O
  inside it.
- `g_log_mux` (a FreeRTOS mutex) guards the LittleFS log state (`g_logf`, `g_seg`,
  `g_seg_bytes`, `g_log_records`). `/stats` and `/log.csv` hold it while scanning the whole log.

**Distance and time figures come from different sources and do not have to agree:**
| Figure | Storage | Reset by |
|---|---|---|
| `g_dist_mi` odometer (`distance_mi`) | NVS `odo` | `/reset` |
| `g_sess_mi` / `g_sess_s` session (moving seconds only) | RAM | every boot |
| `g_life_mi` lifetime, for milestones | NVS `lifemi` | nothing (seeded once from the odometer) |
| `/stats` today/week/month/total | rebuilt from the log on each request | `/log/reset`, oldest segment rolling off |

`/stats` stops at the last logged record, so mid-walk it can trail the live odometer by up to one
heartbeat.

**Persistent log.** Each record is an 8-byte packed `logrec_t {u32 epoch, u16 dmph, u8 flags, u8 ck}`,
where `ck` is the sum of the first 7 bytes. Records go into `LOG_SEGS` (6) files `/lfs/walk.N` of
`LOG_SEG_BYTES` (100 KB) each. When the active segment fills, `log_advance()` deletes the next
segment and starts writing there, and the active index is saved in NVS `logseg`. Readers walk
segments `(g_seg+1)%N … g_seg` to go oldest→newest. OTA leaves the `storage` partition alone, so
**changing `logrec_t` breaks the history already on the device** unless you add migration or
versioning. Records written before SNTP sync have `ts=0` and are skipped. Moving intervals longer
than `LOG_GAP_CAP_S` (10 min) count as power gaps and are excluded.

**Time and bucketing.** Records store UTC epoch. `app_main` sets `TZ=LOCAL_TZ` (US Eastern,
DST-aware) so `localtime_r` gives local calendar days. `days_from_civil()` turns a local date into
a day number. Weeks start on Sunday: `(ldn + 4) / 7`, because epoch day 0 was a Thursday. Day
numbers turn back into `YYYY-MM-DD` with `gmtime_r(ldn*86400)`. That is correct because `ldn` is
already a local civil-day index, so don't "fix" it to `localtime_r`.

**Gamification (`stats_get`).** On each request it rebuilds per-day arrays over a 370-day window
(`STAT_DAYS`, stored as `static` arrays to keep them off the httpd stack). From those it derives
the streak, 7d/30d/window bests, week-over-week momentum, the 7-day activity dots, and the
milestone. Streaks are always recomputed and never persisted, because they are relative to the
goal. All-time records **are** persisted (NVS `recdmi`/`recdmid`/`recds`/`recdsd`) and lazily
bumped inside this GET handler, because the log eventually drops old data. The JSON `days[]` array
emits only the last 60 days. Distance and steps rank identically (steps = miles·1609.34/`STRIDE_M`),
so there is no separate steps record.

**NVS.** Everything lives in namespace `cfg`, with keys `ssid`, `pass`, `odo`, `logseg`, `lifemi`,
`goalmi`, `recdmi`, `recdmid`, `recds`, `recdsd`. NVS keys are limited to 15 characters.

**HTTP.** All JSON is built by hand with `snprintf` into fixed buffers: 640 B for `/status` and an
8 KB heap buffer for `/stats`, which stops adding `days[]` entries past ~7900 B. **Grow the buffer
when you add fields.** `max_uri_handlers = 12` and 10 are registered, so bump it before adding a
third new route. `/reset`, `/log/reset`, and `/goal?mi=` change state through GET.

**Web UI.** The dashboard (`PAGE`) and the setup form (`SETUP`) are C string literals. JS escapes
need double backslashes (`'\\u{1F525}'`), and the JS refers to elements through their ids as
implicit globals (`spd`, `meta`, `streak`, ...). The page polls `/status` every 500 ms and
`/stats` every 10 s.

**WiFi and OTA rollback.** The device boots in STA mode with NVS credentials, falling back to the
compiled ones. If it isn't connected after 30 s, it switches to APSTA with the `walkingpad-setup`
AP (password `SETUP_AP_PASS`, `192.168.4.1/setup`). `ota_mark_valid()` runs only on `GOT_IP` or when the
setup AP starts. **A new image that reaches neither one will roll back** on the next reboot, so
keep one of those paths in any startup change. SNTP starts on the first `GOT_IP`.

## Hardware constraints that affect code

- **The tap is receive-only. Never transmit on, or drive, the pad UART.** `uart_set_pin` leaves TX
  unassigned on purpose. Anything that drives pin 5 (an active level shifter did this) corrupts the
  console↔drive handshake, and the pad throws **E07**.
- As built, pin 5 (5 V) goes **directly** into GPIO16 with no divider, which is out of spec. If
  `frames` stops climbing, `link` flaps, or `raw_V` reads garbage, suspect this hardware first,
  before the decoder.
- Speed formula: `mph = (V − 105) / 160` for cmd `0x42`. Distance and steps are not on the wire.
  Steps are an estimate, `distance / STRIDE_M`, with `STRIDE_M = 0.624` calibrated against the
  console's own step counter. `tools/speedo.py` still defaults to 0.67.

## Docs convention

- When an endpoint, JSON field, tunable, or NVS-persisted value changes, update
  `docs/REFERENCE.md` and the header comment in `walkingpad_bridge.c` in the same commit.
- Update `README.md` only for changes that affect the project story or the feature list. Keep it
  public-facing: no LAN IPs, MAC/BT addresses, local paths, or credentials. Keep it honest about
  who did what. Claude Code wrote the code; the owner did the hardware work, made the decisions,
  and tested. Commits keep the `Co-Authored-By: Claude` trailer.
