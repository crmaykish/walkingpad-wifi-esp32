# Technical reference

Operational and API detail for the walking pad bridge. For the project overview and backstory, see
the [README](../README.md).

## Operations

**Deploy over WiFi (the normal path; the unit is sealed):**
```bash
idf.py build
curl --data-binary @build/walkingpad_bridge.bin http://walkingpad.local/update
```
The dashboard has a file picker that does the same thing. The device reboots into the other OTA
slot. An image that never gets a WiFi IP or starts the setup AP isn't marked valid, so it rolls
back to the previous app on the next reboot. The odometer and the log survive an OTA.

**WiFi won't connect** (router changed, unit moved): after 30 s without a connection, the ESP32
opens an AP named `walkingpad-setup`, using the password from `SETUP_AP_PASS` in
`main/wifi_secrets.h`. Join it, browse to `http://192.168.4.1/setup`, and enter the new
SSID/password. They're saved to NVS, which overrides the compiled-in credentials, and the device
reboots.

**Recovery:** the USB-only `factory` partition is never touched by OTA. If both OTA slots were
somehow bad, open the case and do a USB flash.

**USB flash** (first install, recovery, or any change to `partitions.csv`):
```bash
idf.py -p /dev/ttyUSB0 -b 115200 flash
```
The board's crystal is marginal, so flash at **115200**. The 460800 default fails on boots that
come up on the 26 MHz-derived clock, and a retry usually works. `esptool erase_flash` can fail for
the same reason, but a full flash still works.

On my machine, all of this goes through a small set of wrapper scripts and Claude Code skills
(`mcu-build`, `mcu-flash`, `mcu-detect`) that locate ESP-IDF, find the board by USB vendor ID
(CP210x `10c4`; the FTDI used for bench work is `0403`), and apply the safe flash baud.

## HTTP API

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Dashboard. Polls `/status` every 0.5 s and `/stats` every 10 s. |
| `/status` | GET | Live telemetry (JSON) |
| `/stats` | GET | History and gamification, rebuilt from the log (JSON) |
| `/log.csv` | GET | The whole log as `epoch,utc,mph`, oldest to newest, in UTC |
| `/goal` | GET | Read the daily goal; `?mi=2.5` sets it (0.1–50 mi, saved to NVS) |
| `/reset` | GET | Zero the odometer. Doesn't touch the log or lifetime miles. |
| `/log/reset` | GET | Wipe the log |
| `/update` | POST | OTA. The body is the raw `.bin`. |
| `/setup`, `/wifi` | GET, POST | WiFi credentials form and the handler that receives it |

There is no authentication; see the README's *Known limitations*.

### `/status`

| Field | Meaning |
|---|---|
| `speed_mph`, `speed_kmh` | Current speed (0 if no valid frame for 2 s) |
| `distance_mi`, `distance_m`, `steps_est` | Odometer, saved to NVS and reset by `/reset` |
| `session_mi`, `session_steps`, `session_s` | Since power-on. `session_s` counts moving seconds only, so the session average is `session_mi / (session_s/3600)`. |
| `raw_V`, `cmd`, `frames`, `link` | Last decoded value and command, valid-frame count, `ok`/`stale` |
| `uptime_s`, `rssi`, `ip`, `ap_mode`, `heap` | Device health |
| `log_records`, `log_seg`, `time_synced`, `epoch` | Log and clock state |

`link: stale` only means no frames are arriving right now (the pad is off). The WiFi and web side
is still up.

### `/stats`

```
{ today|week|month: {hours, miles, steps},
  total_hours, total_miles, total_steps, records, synced,
  game: { goal_mi, streak, streak_best, today_record,
          best: { d7: {mi, s}, d30: {mi, s}, all: {mi, mi_date, s, s_date} },
          week_mi, lastweek_mi, life_mi, milestone_step, milestone_next, dots[7] },
  days: [ {d, hours, miles}, … ] }      // last 60 days that have activity
```

- Buckets follow the **US Eastern** calendar (`LOCAL_TZ = EST5EDT`, DST-aware). Weeks start on
  Sunday.
- Time is walking time only. Intervals longer than 10 min (`LOG_GAP_CAP_S`) are treated as power
  gaps and skipped.
- `/stats` counts up to the **last logged record**, so mid-walk it can trail the live odometer by
  up to one heartbeat (~4 min).
- Steps = miles · 1609.34 / `STRIDE_M`. Distance and steps therefore rank identically, which is why
  there is no separate best-steps record.
- **Streaks** are recomputed from a 370-day window (`STAT_DAYS`) on each call because they depend
  on the goal. **All-time bests** are saved to NVS with their dates because the log eventually
  rolls off. `life_mi` is a lifetime counter that only goes up (NVS, not cleared by `/reset`); it
  was seeded once from the odometer.

## Persistent log

- Record format: 8 bytes, packed: `{u32 epoch, u16 speed in 0.1 mph, u8 flags, u8 checksum}`. The
  checksum is the sum of the first 7 bytes.
- A record is written when the speed settles on a new value (0.4 s debounce) and as a heartbeat
  every 4 min while moving, so a power cut loses at most a few minutes.
- The log is `LOG_SEGS` (6) files of `LOG_SEG_BYTES` (100 KB) each on the `storage` LittleFS
  partition. When the active segment fills, the next one is erased and reused, dropping the oldest
  data. That's about 75k records, roughly four years at ~50 speed changes per day.
- Timestamps come from SNTP and are stored as UTC epoch. Records written before the first sync have
  `epoch = 0` and are skipped by `/stats`.
- Tunables are at the top of `main/walkingpad_bridge.c`: `LOG_SEGS`, `LOG_SEG_BYTES`, `LOG_HB_US`,
  `LOG_GAP_CAP_S`, `STRIDE_M`, `GOAL_MI_DEFAULT`, `STAT_DAYS`, `MILESTONE_MI`, `ACTIVE_MI`.

## Flash layout

4 MB flash with a custom `partitions.csv`:

| Partition | Size | Purpose |
|---|---|---|
| `nvs` | 16 KB | WiFi creds, odometer, goal, records, log segment index |
| `factory` | 1 MB | Golden image, written only over USB |
| `ota_0`, `ota_1` | 1 MB each | OTA updates alternate between these |
| `storage` | ~960 KB | LittleFS for the circular log |

The app fills ~91% of a 1 MB slot. Any change to this table needs a USB flash.

## Protocol notes

- UART: 2400 baud, 8N1, 5 V logic. 6-byte frames: `AA cmd HH LL CK 55`, with
  `CK = (cmd + HH + LL) & 0xFF`.
- Pin 5 (console→drive) carries `0x42` run-at-speed frames, where `mph = (V − 105)/160` and each
  0.1 mph step is 16 counts. With the pad assembled, the idle frames are `AA A2 AA AA F6 55` and
  `AA 40 01 AA EB 55`. A console on its own, with no drive board, sends `AA 50 00 55 A5 55` about
  every 280 ms.
- Pin 4 (drive→console) uses the same format and sends a keepalive, `AA 42 00 00 42 55`.
- The firmware byte-syncs on header, trailer, and checksum. The bench tools frame by idle gap
  instead.

## Bench tools

These decode the pad without the ESP32, using only a 5 V FTDI on pin 5 (FTDI RX ← pin 5,
FTDI GND ← pin 1, TX left unconnected). They need `pyserial`.

```bash
python3 tools/speedo.py [port] [stride_m]          # live mph / distance / step estimate; auto-detects the FTDI
python3 tools/framemon.py [port] [baud] [seconds]  # prints frames when they change, with carets under changed bytes
```

## Gotchas

- `CONFIG_ESP32_XTAL_FREQ_40` is pinned in `sdkconfig.defaults` because of the marginal crystal.
  The console runs at 115200.
- USB serial ports re-enumerate, so match boards by vendor ID rather than by `/dev/ttyUSB*` number.
- **If RX goes flaky** (`frames` stops climbing, `link` flaps, `raw_V` shows garbage), suspect the
  out-of-spec direct 5 V tap first and retrofit the 4.7k/10k divider.
- **Never use an active or bidirectional level shifter on pin 5.** It back-drives the line and the
  pad throws E07.
- Steps will never be exact. `STRIDE_M = 0.624` was calibrated against the console's footfall
  counter. Re-tune it if the estimate drifts from the console.
- Keep the ESP32 antenna away from the steel frame. Through the closed enclosure, RSSI was ~-54 dBm.
