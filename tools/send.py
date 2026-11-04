#!/usr/bin/env python3
"""Sendet einen Befehl an die Kamin-Firmware und zeigt, was danach empfangen wird.

Der Logger belegt den seriellen Port exklusiv - dieses Skript uebernimmt ihn
kurzzeitig, damit Senden und Mitlesen in einem Schritt moeglich sind.

Aufruf:  python3 tools/send.py 1        AN
         python3 tools/send.py 2        AUS
         python3 tools/send.py a        Stufe 1   ... o = Stufe 15
         python3 tools/send.py '' 60    nichts senden, 60 s nur lauschen
"""
import sys
import time

import serial

PORT = "/dev/cu.usbserial-0001"
befehl = sys.argv[1] if len(sys.argv) > 1 else ""
dauer = int(sys.argv[2]) if len(sys.argv) > 2 else 45

ZUSTAND = {(0x69, 0x1a): "aus", (0x69, 0x5a): "Zuend-/Nachlaufphase",
           (0x6b, 0x58): "FLAMME BRENNT"}

with serial.Serial(PORT, 115200, timeout=0.2) as s:
    time.sleep(0.3)
    s.reset_input_buffer()
    if befehl:
        s.write(befehl.encode())
        print(f"gesendet: '{befehl}'  -  lausche {dauer} s\n")
    else:
        print(f"lausche {dauer} s ohne zu senden\n")

    ende = time.time() + dauer
    puffer = b""
    while time.time() < ende:
        daten = s.read(s.in_waiting or 1)
        if not daten:
            continue
        puffer += daten
        while b"\n" in puffer:
            zeile, puffer = puffer.split(b"\n", 1)
            t = zeile.decode("utf-8", "replace").strip()
            if not t or len(t) > 200:        # Boot-Muell ausblenden
                continue
            info = ""
            if "RX" in t and len(t) > 40:
                h = t.split()[-1] if "verworfen" not in t else t.split()[3]
                try:
                    b = [int(h[i:i+2], 16) for i in range(0, len(h), 2)]
                    if len(b) == 22:
                        art = "Kommando" if b[6] == 0x78 else "Status"
                        info = f"   [{art}, {ZUSTAND.get((b[12], b[13]), '?')}]"
                except ValueError:
                    pass
            print(f"{time.strftime('%H:%M:%S')} {t}{info}")
