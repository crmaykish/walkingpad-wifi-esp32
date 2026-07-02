#!/usr/bin/env python3
"""Live UART frame monitor: prints a frame only when it changes vs the last one,
with a caret line marking which bytes changed. Great for correlating with speed.
Usage: python3 framemon.py [port] [baud] [seconds]
"""
import serial, time, sys
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2400
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 45
GAP  = 0.12

def dump(b): return " ".join(f"{x:02X}" for x in b)

def diffline(prev, cur):
    out = []
    for i in range(max(len(prev), len(cur))):
        a = prev[i] if i < len(prev) else None
        b = cur[i]  if i < len(cur) else None
        out.append(" ^" if a != b else "  ")
    return "".join(out)

s = serial.Serial(port, baud, timeout=0.01); s.reset_input_buffer()
print(f"# {port} @ {baud} 8N1 — watching {secs:.0f}s. Change the speed and call out timings.")
t0 = time.time(); last = b""; cur = bytearray(); lastrx = None; nframes = 0
while time.time() - t0 < secs:
    n = s.in_waiting
    if n:
        cur += s.read(n); lastrx = time.time()
    else:
        if cur and lastrx and (time.time() - lastrx) > GAP:
            frame = bytes(cur); cur = bytearray(); lastrx = None; nframes += 1
            if frame != last:
                t = time.time() - t0
                print(f"[{t:6.2f}s] {dump(frame)}")
                if last:
                    print(f"         {diffline(last, frame)}")
                last = frame
        time.sleep(0.004)
s.close()
print(f"# done. {nframes} frames seen; last = {dump(last)}")
