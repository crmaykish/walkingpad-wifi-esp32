# Walking Pad Telemetry Bridge

Reverse-engineered the internal serial link of a cheap Amazon walking pad and built an
**ESP32 that taps it and serves live telemetry over WiFi** (speed + distance as a web
dashboard and JSON API). The pad itself only ever exposed a Bluetooth *speaker* — there
was no telemetry interface. This project adds one.

> **Future me, in 3 months:** everything you need is below. The 30-second version is in
> **[Quick reference](#quick-reference)**. The firmware is in `main/`, bench tools in `tools/`.

---

## Quick reference

**Read telemetry** (any device on the LAN):
- Dashboard: **http://walkingpad.local/** (or the DHCP IP; was `192.168.1.223`)
- JSON: `GET /status` → `speed_mph`, `speed_kmh`, `distance_mi`, `distance_m`, `steps_est`,
  `uptime_s`, `raw_V`, `cmd`, `frames`, `link` (`ok`/`stale`), `rssi`, `ip`, `ap_mode`, `heap`
- Reset odometer: `GET /reset`
- **History:** `GET /stats` (JSON — total hours/miles + per-day) · `GET /log.csv` (download the full event log) · `GET /log/reset` (wipe it)

**Update firmware over WiFi** (no disassembly):
```bash
~/Workspace/claude/mcu-tools/mcu_build.sh esp32 ~/Workspace/treadmill/walkingpad-bridge
curl --data-binary @~/Workspace/treadmill/walkingpad-bridge/build/walkingpad_bridge.bin \
     http://walkingpad.local/update
```
…or pick the `.bin` in the dashboard. A bad image auto-rolls-back.

**WiFi won't connect** (moved, changed router): the ESP opens an AP **`walkingpad-setup`**
(password `walkingpad`) → join it → browse **http://192.168.4.1/setup** → enter new
SSID/password → it saves to NVS and reboots.

**Bricked** (should never happen — OTA has rollback): the USB-only `factory` image is
still intact. Open it up, `mcu_flash.sh` (see [Flashing](#flashing-usb--first-time--recovery)).

- ESP32 MAC (for a DHCP reservation): `b0:cb:d8:ce:14:20`
- Firmware source: `main/walkingpad_bridge.c` · Bench tools: `tools/`

---

## The hardware findings

Board silkscreen: **`KRDJK1608E-2.2.4GDY`** (the pad's digital/console PCB).

| Chip | What it is | Role |
|---|---|---|
| **SC95F7616BP** | SinOne 8051 MCU (1T core, runs at **5 V**) | The brain: drives the LCD, commands motor speed, counts steps. **Telemetry originates here.** |
| **AC223…RYD** (24 MHz + trace antenna) | **Jieli** Bluetooth-audio SoC (BT company ID `0x05D6`; Modalias `bluetooth:v05D6p000Ad0240`) | The "bluetooth mode" **speaker**. Audio only. |
| sanded SOIC-16 + 12 MHz + SOIC-8 flash | (marking removed) | Likely the voice-prompt subsystem. Not relevant. |
| unpopulated 14-pin footprint w/ trace antenna | Tuya/ESP8285 WiFi-module socket | The "smart SKU" WiFi option — **firmware-gated on this unit, dead** (see below). |

**Bluetooth is a dead end for telemetry.** The pad's only BT interface is a Classic-BT
A2DP audio sink (device name literally `"Bluetooth"`, addr `82:1B:A6:1F:BD:2E`, class
`0x240404`). SDP shows only Audio Sink + AVRCP + Device-ID. No SPP, no BLE, no data.

**The Tuya WiFi port is also dead.** The empty module footprint has a 5 V UART wired to
the SC95F, but this no-WiFi SKU's firmware never starts the Tuya task — the line is
totally silent in every state (idle, running, E07-cleared, both TX/RX orientations).
Not worth revisiting.

**The telemetry lives on the console↔drive-board link** — a 5-pin connector:

| Pin | Signal |
|---|---|
| 1 | GND |
| 2 | +12 V |
| 3 | ~3.6 V static (reference/enable) |
| 4 | data **drive→console** (keepalive/status, `AA 42 00 00 42 55`) |
| **5** | data **console→drive** ← **the speed command we tap** |

---

## The protocol (decoded)

**UART: 2400 baud, 8N1, 5 V logic.** Frame is 6 bytes:

```
AA  <cmd>  <HH>  <LL>  <CK>  55
│    │      └──┬──┘     │     └─ trailer 0x55
│    │        │        └─────── checksum = (cmd + HH + LL) & 0xFF
│    │        └──────────────── 16-bit value, big-endian: V = HH*256 + LL
│    └───────────────────────── command
└────────────────────────────── header 0xAA
```

- **`cmd 0x42` = run-at-speed.** `V` is the speed setpoint (what the display shows):

  **`mph = (V − 105) / 160`**  (equivalently `V = 160·mph + 105`; +16 counts per 0.1 mph)

  Verified: `V=185`→0.5, `201`→0.6, `217`→0.7 … `281`→1.1 mph.
- **`cmd 0x40`, `0xA2`** = assembled idle/stopped. **`cmd 0x50`** = standalone idle (no drive
  board). All non-`0x42` commands ⇒ **0 mph**.
- **Distance** is not transmitted — the console just integrates speed over time, so we do the
  same: `distance = ∫ speed·dt`.
- **Steps** are not on the wire at all. The console counts real footfalls from a dedicated
  sensor and only displays them. `steps_est` here is `distance / STRIDE_M` — an estimate.

---

## The tap (wiring)

Pin 5 is 5 V; the ESP32 is **not** 5 V-tolerant. Use a **passive resistor divider** —
listen-only, so it never disturbs the bus:

```
pin 5 ──[ 4.7k ]──┬──► RX2 / GPIO16   (UART2 RX on the ESP32)
                  │
                [ 10k ]
                  │
                 GND (common with pad pin 1 and ESP GND)
```
→ ~3.4 V logic high. Also: **pad 5 V rail → ESP32 `VIN`/`5V`** (its regulator makes 3.3 V).

> ⚠️ **Do NOT use an active/bidirectional level shifter (e.g. TXS0108E) here.** It back-drives
> pin 5 during ESP boot and corrupts the console↔drive handshake → the pad throws **E07**.
> A bare FTDI or the passive divider is fine because they're high-impedance / receive-only.

The pad's low-voltage ground is **isolated from mains** (verified — no continuity from
logic GND to the AC pins), so tying a grounded FTDI/PC to it during bench work is safe.

---

## The ESP32 firmware

- **Board:** ESP32-D0WD-V3 (WROOM), classic dual-core Xtensa, CP2102 USB-UART.
- **SDK:** ESP-IDF v5.4 (`~/Workspace/claude/esp-idf`), via the `mcu-*` skills/scripts in
  `~/Workspace/claude/mcu-tools/`.
- **Source:** `main/walkingpad_bridge.c`.

**Features**
- UART2 RX = **GPIO16** (the board's `RX2` pin) @ 2400 8N1; byte-syncs on `AA…55` + checksum.
- WiFi station; serves an auto-refresh dashboard (`/`) + JSON (`/status`) + `/reset`.
- **OTA** (`POST /update`, raw `.bin`) with dual app slots + rollback.
- **mDNS** → `walkingpad.local`.
- **WiFi setup-AP fallback** (`walkingpad-setup` / `walkingpad`) → `/setup` form → NVS creds.
- **Odometer persisted to NVS** (survives reboots).
- **Persistent event log** on a LittleFS partition — see [Persistent logging](#persistent-logging).

**Partition layout** (4 MB flash, custom `partitions.csv`): `factory` (golden, USB-only
recovery) + `ota_0` + `ota_1` (OTA ping-pongs between them) + **`storage`** (~960 KB LittleFS
for the event log). `FLASHSIZE_4MB` + `APP_ROLLBACK_ENABLE` in `sdkconfig.defaults`. Changing
this table (e.g. resizing `storage`) needs a **USB** flash — it can't be done over OTA.

**WiFi credentials** live in gitignored **`main/wifi_secrets.h`** — two `#define`s
(`WIFI_SSID` / `WIFI_PASS`) — so they're never committed. If that file is missing (e.g. a
fresh clone), the build falls back to placeholder defines that won't connect; recreate it,
or just set creds at runtime via the setup AP (stored in NVS, which overrides the compiled
default). Example `main/wifi_secrets.h`:
```c
#pragma once
#define WIFI_SSID "YourNetwork"
#define WIFI_PASS "YourPassword"
```

**Config knobs** (top of `main/walkingpad_bridge.c`): `WIFI_SSID/PASS`, `HOSTNAME`,
`STRIDE_M` (step estimate), `PAD_RX_GPIO`, `PAD_BAUD`.

### Building
```bash
~/Workspace/claude/mcu-tools/mcu_build.sh esp32 ~/Workspace/treadmill/walkingpad-bridge
```

### Flashing (USB — first time / recovery)
The ESP shows up as a CP2102 (`mcu-detect` finds the port; it drifts, was `/dev/ttyUSB1`).
This board has a **marginal 26 MHz crystal**, so flash at **115200** (the tooling default).
`esptool erase_flash` may fail on a skewed boot — that's fine, a full flash still works:
```bash
ESP32_PORT=/dev/ttyUSB1 ESP32_FLASH_BAUD=115200 \
  ~/Workspace/claude/mcu-tools/mcu_flash.sh esp32 ~/Workspace/treadmill/walkingpad-bridge
```

### Updating (WiFi — normal case)
See [Quick reference](#quick-reference). Runs from `ota_0`/`ota_1`; `factory` is untouched.

---

## Persistent logging

Every settled **speed change** is written as an 8-byte record `{epoch, 0.1 mph, flags, checksum}`
to a **segmented circular log** on the `storage` LittleFS partition, plus a heartbeat every
~4 min while moving (so a power cut loses at most a few minutes). Time comes from **SNTP** (UTC).

The log is `LOG_SEGS` (6) files × `LOG_SEG_BYTES` (100 KB) ≈ 600 KB. When the active segment
fills, it advances and **erases the next one — dropping the oldest data**. So it's bounded
forever and never fails silently: ~75k records ≈ several years (at ~50 changes/day, ~4 yrs).

- `GET /stats` → `{total_hours, total_miles, records, days:[{d,hours,miles}]}` (per-day for the
  last 60 days; power-off gaps filtered by a 10-min cap so they aren't counted as walking).
- `GET /log.csv` → the raw log as `epoch,utc,mph`, oldest→newest. Reconstruct however you like:
  between consecutive records speed is constant, so `hours += Δt`, `miles += mph·Δt`.
- `GET /log/reset` → wipe the log.
- Records logged before the first NTP sync (e.g. an idle boot) have `epoch=0` and are skipped by
  `/stats`. Logging is UTC — apply your timezone when reconstructing.
- `/stats` counts only up to the **last logged record**, so mid-walk it can trail the live
  `distance_mi` odometer (`/status`) by up to one heartbeat (~4 min). The odometer is the
  continuous live figure; `/stats` is the durable reconstructed history.
- Tunables at the top of `main/walkingpad_bridge.c`: `LOG_SEGS`, `LOG_SEG_BYTES`, `LOG_HB_US`,
  `LOG_GAP_CAP_S`.

## Bench debugging tools (`tools/`)

For decoding/verifying the pad **without** the ESP32 — just a **5 V FTDI** on pin 5
(FTDI RX ← pin 5 via the same divider or direct, FTDI GND ← pin 1, no TX):

- **`tools/speedo.py`** — live speedometer. Auto-detects the FTDI, decodes pin 5, prints
  mph + accumulated distance + step estimate. Compare against the console display.
  ```bash
  python3 tools/speedo.py            # auto-detect FTDI
  python3 tools/speedo.py /dev/ttyUSB0 0.67   # explicit port + stride(m)
  ```
- **`tools/framemon.py`** — raw frame monitor; prints a frame each time it *changes*, with
  carets under the bytes that moved. How the speed byte was originally found.
  ```bash
  python3 tools/framemon.py /dev/ttyUSB1 2400 30   # port baud seconds
  ```
Both need `pyserial` (`pip install --user pyserial`).

---

## Gotchas / notes

- **Marginal crystal:** `CONFIG_ESP32_XTAL_FREQ_40` is pinned; flash/console at 115200.
- **Serial port drift:** USB ports re-enumerate — use `mcu-detect` / match by USB VID
  (ESP32 = CP210x `10c4`, the FTDI = `0403`).
- **`link: stale` in /status** just means no frames right now (pad off / not wired) — the
  WiFi/web side is still up.
- **Steps** will never be exact — no footfall data on the wire. Tune `STRIDE_M` if you care.
- Before sealing, sanity-check **RSSI** from the install spot via `/status`; keep the ESP
  antenna away from the steel frame.

---

## How it was found (short version)

BT scan → the pad's only radio is a Jieli A2DP speaker (no telemetry) → the unpopulated
Tuya WiFi port is firmware-gated/silent → opened the console, found the 5-pin console↔drive
connector → scoped pin 5, `2400 8N1` (a scope "period" read was a 2-bit cycle, so half of
831 µs = 2400 baud) → captured frames while stepping speed → `cmd 0x42`, `mph=(V−105)/160`,
checksum `(cmd+HH+LL)&0xFF` → tapped pin 5 into an ESP32 (passive divider; a TXS0108E tripped
E07) → firmware serves it over WiFi with OTA so it never needs opening again.
