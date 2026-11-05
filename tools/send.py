#!/usr/bin/env python3
"""Send one command to the fireplace firmware and show what is received afterwards.

The logger holds the serial port exclusively; this script takes it over
briefly so sending and reading happen in one step.

Usage:  python3 tools/send.py 1        ON
        python3 tools/send.py 2        OFF
        python3 tools/send.py a        level 1   ... o = level 15
        python3 tools/send.py '' 60    send nothing, just listen for 60 s
"""
import re
import sys
import time

import serial

PORT = "/dev/cu.usbserial-0001"
command = sys.argv[1] if len(sys.argv) > 1 else ""
duration = int(sys.argv[2]) if len(sys.argv) > 2 else 45

STATE = {(0x69, 0x1a): "off", (0x69, 0x5a): "ignition/run-on phase",
         (0x6b, 0x58): "FLAME ON"}

with serial.Serial(PORT, 115200, timeout=0.2) as s:
    time.sleep(0.3)
    s.reset_input_buffer()
    if command:
        s.write(command.encode())
        print(f"sent: '{command}'  -  listening for {duration} s\n")
    else:
        print(f"listening for {duration} s without sending\n")

    deadline = time.time() + duration
    buffer = b""
    while time.time() < deadline:
        data = s.read(s.in_waiting or 1)
        if not data:
            continue
        buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            t = line.decode("utf-8", "replace").strip()
            if not t or len(t) > 200:        # hide boot garbage
                continue
            info = ""
            m = re.search(r"dBm\s+([0-9a-f]{40,})", t)
            if m:
                h = m.group(1)
                b = [int(h[i:i+2], 16) for i in range(0, len(h), 2)]
                if len(b) == 22:
                    kind = "command" if b[6] == 0x78 else "status"
                    info = f"   [{kind}, {STATE.get((b[12], b[13]), '?')}]"
            print(f"{time.strftime('%H:%M:%S')} {t}{info}")
