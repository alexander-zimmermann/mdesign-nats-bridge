#!/usr/bin/env python3
"""Beobachtet die Statustelegramme des Kaminempfaengers und zeigt nur
Aenderungen an - so faellt sofort auf, welches Byte auf welche Aktion reagiert.

Die Sequenz- und Toggle-Bytes (4, 5) sowie die Pruefsumme (19-21) aendern sich
bei jedem Telegramm und werden ausgeblendet; sie tragen keine Zustandsinfo.

Aufruf:  python3 tools/status.py [logdatei]
         (liest fortlaufend weiter, wie `tail -f`)
"""
import re
import sys
import time

LOG = sys.argv[1] if len(sys.argv) > 1 else "captures/stufen.log"

# Bytes, die bei jedem Telegramm wechseln und keinen Zustand tragen
IGNORIEREN = {0, 1, 2, 3, 4, 5, 19, 20, 21}

# grober Leistungsbereich aus Byte 15, aus dem Stufenlauf abgeleitet
BEREICH = {0x44: "1-4", 0x53: "1-4", 0x4d: "4-8", 0x69: "4-8",
           0xb5: "8-10", 0x66: "9-12", 0xbc: "11-13",
           0xa5: "13-15", 0x71: "13-15"}


def zerlege(h):
    return [int(h[i:i+2], 16) for i in range(0, len(h), 2)]


def main():
    letzter = None
    print("Beobachte Statustelegramme - nur Aenderungen werden angezeigt.\n")
    print("Zeit      RSSI  Byte:  6..11              12 13 14 15 16   Bereich")
    with open(LOG) as f:
        f.seek(0, 2)                      # ans Ende springen, nur Neues lesen
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
                print(f"{t}  {rssi:4d}   >>> SCHALTBEFEHL  {m.group(3)[20:28]}")
                continue
            if adr != "f7b9" or b[6] != 0x0b:      # nur Statustelegramme
                continue

            kern = tuple(b[i] for i in range(len(b)) if i not in IGNORIEREN)
            if kern == letzter:
                continue                            # unveraendert, nicht ausgeben
            # geaenderte Bytes gegenueber dem letzten Stand markieren
            diff = ""
            if letzter is not None:
                stellen = [i for i, (a, c) in enumerate(zip(letzter, kern)) if a != c]
                diff = "   geaendert an Position " + ",".join(map(str, stellen))
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
