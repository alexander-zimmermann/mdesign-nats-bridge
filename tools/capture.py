#!/usr/bin/env python3
"""Record key presses on the remote and save them in the same format as the
first recording (.complex16s = int16 I/Q interleaved).

rtl_sdr delivers uint8 (offset binary) while the analysis scripts expect
int16, so the samples are converted on write.

Usage:  python3 tools/capture.py ON
        python3 tools/capture.py UP --dauer 8
"""
import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

FS = 1_000_000        # as in the first recording, so SPS in compare.py and measure.py matches
# The remote transmits on 868.3 MHz. Centre 200 kHz off it so the signal
# stays clear of the RTL-SDR's DC spike.
FC = 868_500_000
GAIN = 1              # dB; anything higher overdrives the ADC. Note: 0 = AGC

CAPTURES = Path(__file__).resolve().parent.parent / "captures"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="command label, e.g. ON / OFF / UP / DOWN")
    ap.add_argument("--dauer", type=float, default=6.0, help="seconds (default 6)")
    ap.add_argument("--gain", type=float, default=GAIN)
    ap.add_argument("--fc", type=float, default=FC, help="centre frequency in Hz")
    args = ap.parse_args()

    CAPTURES.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = CAPTURES / f"Kamin-{args.name}-{stamp}.complex16s"

    n_samples = int(FS * args.dauer)
    cmd = ["rtl_sdr", "-f", str(int(args.fc)), "-s", str(FS),
           "-g", str(args.gain), "-n", str(n_samples), "-"]

    print(f"Recording {args.dauer:.0f}s @ {args.fc/1e6:.2f} MHz, gain {args.gain:.0f} dB -> {out.name}")
    print("Press the key several times now (3-4x, ~1s apart).\n")

    # Write to a raw file instead of a pipe: it survives Ctrl+C, and the
    # samples recorded so far stay usable.
    raw = out.with_suffix(".u8")
    cmd[-1] = str(raw)
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nAborted - using what was recorded.")

    if not raw.exists() or raw.stat().st_size == 0:
        sys.exit("No samples recorded - is the RTL-SDR plugged in? (rtl_test -t)")

    u8 = np.fromfile(raw, dtype=np.uint8)
    raw.unlink()
    # uint8 offset binary -> int16 signed, same scaling as the first recording
    i16 = ((u8.astype(np.int16) - 128) << 8).astype(np.int16)
    i16.tofile(out)

    clip = np.mean(np.abs(i16) >= 32512) * 100
    print(f"\nSaved: {out}  ({out.stat().st_size/1e6:.1f} MB)")
    if clip > 0.01:
        print(f"WARNING: {clip:.2f}% of samples clip -> lower the gain "
              f"(--gain {max(0, args.gain-10):.0f}) or move further away.")
    print(f"Analyse:   .venv/bin/python3 tools/compare.py {out}")


if __name__ == "__main__":
    main()
