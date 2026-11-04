#!/usr/bin/env python3
"""Zeigt aus dem Mitschnitt nur die Kommandoframes des Handsenders mit dem
Stellwert (Byte 17). Statustelegramme und Rauschen werden ausgeblendet."""
import re, sys
log = sys.argv[1] if len(sys.argv) > 1 else "captures/stufen.log"
seen = None
for line in open(log):
    m = re.search(r"(\d\d:\d\d:\d\d).*?(-?\d+) dBm\s+([0-9a-f]{40,})", line)
    if not m:
        continue
    t, rssi, h = m.group(1), int(m.group(2)), m.group(3)
    b = [int(h[i:i+2], 16) for i in range(0, len(h), 2)]
    if len(b) < 20:
        continue
    adr = f"{b[2]:02x}{b[3]:02x}"
    if adr == "f939":                                  # Schaltbefehl AN/AUS
        print(f"{t}  {rssi:4d}  SCHALT   {h[20:28]}")
        continue
    if adr != "f7b9" or b[6] != 0x78:                  # nur Handsender-Kommandos
        continue
    if b[17] != seen:
        print(f"{t}  {rssi:4d}  Stellwert {b[17]:02x}")
        seen = b[17]
