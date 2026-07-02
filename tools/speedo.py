#!/usr/bin/env python3
"""Live walking-pad speedometer, decoded from pin 5 (console->drive, 2400 8N1).
Run frame: AA 42 HH LL CK 55 ; V=HH*256+LL ; mph=(V-105)/160 ; CK=(0x42+HH+LL)&0xFF
Idle/stopped frames use cmd 0x40/0xA2 -> shown as 0.0 mph.
Frames by idle-gap (robust to 0xAA inside payload). Ctrl-C to stop.

Usage: python3 speedo.py [port] [stride_m]   (port auto-detected if omitted)
"""
import serial, time, sys, glob, os

def find_ftdi():
    for dev in sorted(glob.glob('/dev/ttyUSB*')):
        p = os.path.realpath(f'/sys/class/tty/{os.path.basename(dev)}/device')
        while p and p != '/':
            vid = os.path.join(p, 'idVendor')
            if os.path.exists(vid):
                if open(vid).read().strip() == '0403':
                    return dev
                break
            p = os.path.dirname(p)
    return None

port   = sys.argv[1] if len(sys.argv) > 1 else (find_ftdi() or '/dev/ttyUSB0')
stride = float(sys.argv[2]) if len(sys.argv) > 2 else 0.67   # metres/step (est.)

def mph(V): return max(0.0, (V - 105) / 160.0)

s = serial.Serial(port, 2400, timeout=0.02); s.reset_input_buffer()
print(f"# speedo on {port} @2400 8N1 — walk & change speed, compare 'mph' to the display. Ctrl-C to stop.")
cur = bytearray(); last = None
speed = 0.0; dist_mi = 0.0; V = 0; cmd = 0
t0 = time.time(); lt = t0; lp = 0.0; GAP = 0.08
try:
    while True:
        n = s.in_waiting
        if n:
            cur += s.read(n); last = time.time()
        elif cur and last and time.time() - last > GAP:
            f = bytes(cur); cur = bytearray(); last = None
            if len(f) == 6 and f[0] == 0xAA and f[5] == 0x55 and ((f[1]+f[2]+f[3]) & 0xFF) == f[4]:
                cmd = f[1]; V = f[2]*256 + f[3]
                if   cmd == 0x42:            speed = mph(V)     # run-at-speed
                elif cmd in (0x40, 0xA2):    speed = 0.0        # idle/stopped
        else:
            time.sleep(0.004)
        now = time.time(); dt = now - lt; lt = now
        dist_mi += speed * dt / 3600.0
        if now - lp > 0.2:
            lp = now; dm = dist_mi * 1609.34
            print(f"\r  {speed:4.1f} mph  (cmd {cmd:02X} V={V:<4d}) | "
                  f"{dist_mi:6.3f} mi ({dm:6.1f} m) | {dm/stride:5.0f} steps | {now-t0:4.0f}s  ",
                  end="", flush=True)
except KeyboardInterrupt:
    dm = dist_mi * 1609.34
    print(f"\n# stopped: {dist_mi:.3f} mi / {dm:.0f} m / ~{dm/stride:.0f} steps (stride {stride} m)")
