#!/usr/bin/env python3
"""Breitband-Scan: nimmt 2,4 MHz um 868,2 MHz auf und zeigt, WO im Band
waehrend der Aufnahme Bursts auftreten. Dient dazu, die tatsaechliche
Sendefrequenz des Handsenders zu finden.

Aufruf:  .venv/bin/python3 tools/scan.py           (6 s, Taste druecken)
         .venv/bin/python3 tools/scan.py --gain 30
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

FS = 2_400_000
FC = 868_200_000      # deckt damit 867,0 .. 869,4 MHz ab
TMP = Path("/tmp/kamin-scan.u8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dauer", type=float, default=6.0)
    ap.add_argument("--gain", type=float, default=20.0)
    ap.add_argument("--fc", type=float, default=FC)
    args = ap.parse_args()

    cmd = ["rtl_sdr", "-f", str(int(args.fc)), "-s", str(FS),
           "-g", str(args.gain), "-n", str(int(FS * args.dauer)), str(TMP)]
    print(f"Scan {args.dauer:.0f}s, {(args.fc-FS/2)/1e6:.2f}..{(args.fc+FS/2)/1e6:.2f} MHz, "
          f"Gain {args.gain:.0f} dB")
    print("Taste jetzt mehrfach druecken.\n")
    subprocess.run(cmd)
    if not TMP.exists() or TMP.stat().st_size == 0:
        sys.exit("Keine Samples aufgenommen.")

    u8 = np.fromfile(TMP, dtype=np.uint8).astype(np.float32) - 127.5
    iq = u8[0::2] + 1j * u8[1::2]
    TMP.unlink()
    clip = np.mean(np.abs(u8) >= 127) * 100
    print(f"{len(iq)/FS:.1f}s aufgenommen, Clipping {clip:.3f}%"
          + ("  <-- Gain senken!" if clip > 0.01 else ""))

    N = 2048
    n = len(iq) // N
    S = np.abs(np.fft.fftshift(
        np.fft.fft(iq[:n*N].reshape(n, N), axis=1), axes=1))**2
    freqs = np.fft.fftshift(np.fft.fftfreq(N, 1/FS))
    t = np.arange(n) * N / FS

    med = np.median(S, axis=0) + 1e-12      # Rauschteppich je Frequenzbin
    excess = 10*np.log10(S / med)           # Ueberhoehung je Zeit/Frequenz

    print(f"\nEreignisse (>15 dB ueber Rauschteppich):")
    peak_per_bin = excess.max(axis=0)
    active = np.where(peak_per_bin > 15)[0]
    if len(active) == 0:
        print("   keine gefunden - Gain erhoehen oder naeher an den Sender.")
        return

    # benachbarte Bins zu Kanaelen zusammenfassen
    groups = np.split(active, np.where(np.diff(active) > 3)[0] + 1)
    for g in groups:
        if len(g) < 2:
            continue
        f_lo = (args.fc + freqs[g[0]]) / 1e6
        f_hi = (args.fc + freqs[g[-1]]) / 1e6
        sub = excess[:, g].max(axis=1)
        times = t[sub > 15]
        print(f"   {f_lo:8.3f} .. {f_hi:8.3f} MHz  "
              f"max +{sub.max():4.1f} dB  "
              f"{len(times):4d} aktive Zeitfenster  "
              f"erste {times[0]:.2f}s, letzte {times[-1]:.2f}s")


if __name__ == "__main__":
    main()
