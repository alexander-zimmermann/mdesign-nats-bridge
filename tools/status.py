#!/usr/bin/env python3
"""Watch the fireplace receiver's status telegrams and show only changes,
so it is obvious which byte reacts to which action.

The sequence and toggle bytes (4, 5) and the checksum (19-21) change with
every telegram and are hidden; they carry no state.

Usage:  python3 tools/status.py [logfile]
        (keeps reading, like `tail -f`)
"""
import re
import sys
import time

LOG = sys.argv[1] if len(sys.argv) > 1 else "captures/stufen.log"

# Bytes that change with every telegram and carry no state
IGNORIEREN = {0, 1, 2, 3, 4, 5, 19, 20, 21}

# Rough power range from byte 15, derived from a level sweep
BEREICH = {0x44: "1-4", 0x53: "1-4", 0x4d: "4-8", 0x69: "4-8",
           0xb5: "8-10", 0x66: "9-12", 0xbc: "11-13",
           0xa5: "13-15", 0x71: "13-15"}


def zerlege(h):
    return [int(h[i:i+2], 16) for i in range(0, len(h), 2)]


def main():
    letzter = None
    print("Watching status telegrams - only changes are shown.\n")
    print("Time      RSSI  Byte:  6..11              12 13 14 15 16   Range")
    with open(LOG) as f:
        f.seek(0, 2)                      # jump to the end, read only new lines
        while True:
            line = f.readline()
            if not line:
                time.sleep(0.2)
                continue
            m = re.search(r"(\d\d:\d\d:\d\d).*?(-?\d+) dBm\s+([0-9a-f]{40,})", line)
            if not m:
                continue
            t, rssi, b = m.group(1), int(m.group(2)), zerlege(m.group(3))
            if len(b) < 22:
                continue
            adr = f"{b[2]:02x}{b[3]:02x}"
            if adr == "f939":
                print(f"{t}  {rssi:4d}   >>> SWITCH COMMAND  {m.group(3)[20:28]}")
                continue
            if adr != "f7b9" or b[6] != 0x0b:      # status telegrams only
                continue

            kern = tuple(b[i] for i in range(len(b)) if i not in IGNORIEREN)
            if kern == letzter:
                continue                            # unchanged, skip
            # mark the bytes that changed since the last state
            diff = ""
            if letzter is not None:
                stellen = [i for i, (a, c) in enumerate(zip(letzter, kern)) if a != c]
                diff = "   changed at position " + ",".join(map(str, stellen))
            letzter = kern

            rumpf = "".join(f"{b[i]:02x}" for i in range(6, 12))
            rest = " ".join(f"{b[i]:02x}" for i in range(12, 17))
            ber = BEREICH.get(b[15], "?")
            print(f"{t}  {rssi:4d}   {rumpf}   {rest}   {ber:6s}{diff}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
