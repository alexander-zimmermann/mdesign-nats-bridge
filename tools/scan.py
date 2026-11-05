#!/usr/bin/env python3
"""Wideband scan: records 2.4 MHz around 868.2 MHz and shows WHERE in the band
bursts occur during the recording. Used to find the remote's actual
transmit frequency.

Usage:  .venv/bin/python3 tools/scan.py           (6 s, press keys)
        .venv/bin/python3 tools/scan.py --gain 30
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

FS = 2_400_000
FC = 868_200_000      # covers 867.0 .. 869.4 MHz
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
          f"gain {args.gain:.0f} dB")
    print("Press keys several times now.\n")
    subprocess.run(cmd)
    if not TMP.exists() or TMP.stat().st_size == 0:
        sys.exit("No samples recorded.")

    u8 = np.fromfile(TMP, dtype=np.uint8).astype(np.float32) - 127.5
    iq = u8[0::2] + 1j * u8[1::2]
    TMP.unlink()
    clip = np.mean(np.abs(u8) >= 127) * 100
    print(f"{len(iq)/FS:.1f}s recorded, clipping {clip:.3f}%"
          + ("  <-- lower the gain!" if clip > 0.01 else ""))

    N = 2048
    n = len(iq) // N
    S = np.abs(np.fft.fftshift(
        np.fft.fft(iq[:n*N].reshape(n, N), axis=1), axes=1))**2
    freqs = np.fft.fftshift(np.fft.fftfreq(N, 1/FS))
    t = np.arange(n) * N / FS

    med = np.median(S, axis=0) + 1e-12      # noise floor per frequency bin
    excess = 10*np.log10(S / med)           # excess per time/frequency

    print(f"\nEvents (>15 dB above the noise floor):")
    peak_per_bin = excess.max(axis=0)
    active = np.where(peak_per_bin > 15)[0]
    if len(active) == 0:
        print("   none found - raise the gain or move closer to the transmitter.")
        return

    # merge neighbouring bins into channels
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
              f"{len(times):4d} active time slots  "
              f"first {times[0]:.2f}s, last {times[-1]:.2f}s")


if __name__ == "__main__":
    main()
