#!/usr/bin/env python3
"""Write the ESP32's serial output to a file with timestamps.
Unlike `pio device monitor` it needs no terminal and runs in the background,
so the analysis can read along.

It buffers by itself instead of using readline(): on a timeout readline()
returns fragments, which would split frames in the middle of the hex string.
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
out = sys.argv[2] if len(sys.argv) > 2 else "captures/rx.log"

with serial.Serial(port, 115200, timeout=0.2) as s, open(out, "a", buffering=1) as f:
    f.write(f"--- Start {time.strftime('%H:%M:%S')} ---\n")
    buffer = b""
    while True:
        data = s.read(s.in_waiting or 1)
        if not data:
            continue
        buffer += data
        while b"\n" in buffer:                    # write complete lines only
            line, buffer = buffer.split(b"\n", 1)
            text = line.decode("utf-8", "replace").rstrip()
            if text:
                f.write(f"{time.strftime('%H:%M:%S')} {text}\n")
