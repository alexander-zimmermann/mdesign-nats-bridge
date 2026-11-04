#!/usr/bin/env python3
"""Nimmt einen Tastendruck des Handsenders auf und speichert ihn im
gleichen Format wie die erste Aufnahme (.complex16s = int16 I/Q interleaved).

rtl_sdr liefert uint8 (offset binary), die Analyseskripte erwarten int16 --
deshalb wird direkt beim Schreiben konvertiert.

Aufruf:  python3 tools/capture.py AN
         python3 tools/capture.py HOCH --dauer 8
"""
import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

FS = 1_000_000        # wie bei der ersten Aufnahme, damit SPS in compare.py und measure.py stimmt
# Handsender sendet auf 868,3 MHz (gemessen 2026-07-28). Bewusst 200 kHz
# daneben zentrieren, damit das Signal nicht auf den DC-Spike des RTL-SDR faellt.
FC = 868_500_000
GAIN = 1              # dB; alles darueber uebersteuerte den ADC. Achtung: 0 = AGC

CAPTURES = Path(__file__).resolve().parent.parent / "captures"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="Kommando-Name, z.B. AN / AUS / HOCH / RUNTER")
    ap.add_argument("--dauer", type=float, default=6.0, help="Sekunden (Standard 6)")
    ap.add_argument("--gain", type=float, default=GAIN)
    ap.add_argument("--fc", type=float, default=FC, help="Mittenfrequenz in Hz")
    args = ap.parse_args()

    CAPTURES.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = CAPTURES / f"Kamin-{args.name}-{stamp}.complex16s"

    n_samples = int(FS * args.dauer)
    cmd = ["rtl_sdr", "-f", str(int(args.fc)), "-s", str(FS),
           "-g", str(args.gain), "-n", str(n_samples), "-"]

    print(f"Aufnahme {args.dauer:.0f}s @ {args.fc/1e6:.2f} MHz, Gain {args.gain:.0f} dB -> {out.name}")
    print("Taste jetzt mehrfach druecken (3-4x, ~1s Pause dazwischen).\n")

    # In eine Rohdatei schreiben statt durch eine Pipe: ueberlebt ein Ctrl+C,
    # die bis dahin aufgenommenen Samples bleiben nutzbar.
    raw = out.with_suffix(".u8")
    cmd[-1] = str(raw)
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nAbgebrochen - verwerte, was aufgenommen wurde.")

    if not raw.exists() or raw.stat().st_size == 0:
        sys.exit("Keine Samples aufgenommen - steckt der RTL-SDR? (rtl_test -t)")

    u8 = np.fromfile(raw, dtype=np.uint8)
    raw.unlink()
    # uint8 offset binary -> int16 signed, gleiche Skalierung wie die Erstaufnahme
    i16 = ((u8.astype(np.int16) - 128) << 8).astype(np.int16)
    i16.tofile(out)

    clip = np.mean(np.abs(i16) >= 32512) * 100
    print(f"\nGespeichert: {out}  ({out.stat().st_size/1e6:.1f} MB)")
    if clip > 0.01:
        print(f"WARNUNG: {clip:.2f}% der Samples clippen -> Gain senken "
              f"(--gain {max(0, args.gain-10):.0f}) oder Abstand vergroessern.")
    print(f"Auswerten:   .venv/bin/python3 tools/compare.py {out}")


if __name__ == "__main__":
    main()
