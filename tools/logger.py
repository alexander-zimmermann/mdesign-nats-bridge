#!/usr/bin/env python3
"""Schreibt die Serial-Ausgabe des ESP32 mit Zeitstempel in eine Datei.
Anders als `pio device monitor` braucht das kein Terminal und laeuft im
Hintergrund - so kann die Auswertung parallel mitlesen.

Wichtig: Es wird selbst gepuffert statt readline() zu benutzen. readline()
liefert bei Timeout Bruchstuecke, wodurch Frames mitten im Hex-String
zerrissen werden.
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
out = sys.argv[2] if len(sys.argv) > 2 else "captures/rx.log"

with serial.Serial(port, 115200, timeout=0.2) as s, open(out, "a", buffering=1) as f:
    f.write(f"--- Start {time.strftime('%H:%M:%S')} ---\n")
    puffer = b""
    while True:
        daten = s.read(s.in_waiting or 1)
        if not daten:
            continue
        puffer += daten
        while b"\n" in puffer:                    # nur vollstaendige Zeilen schreiben
            zeile, puffer = puffer.split(b"\n", 1)
            text = zeile.decode("utf-8", "replace").rstrip()
            if text:
                f.write(f"{time.strftime('%H:%M:%S')} {text}\n")
