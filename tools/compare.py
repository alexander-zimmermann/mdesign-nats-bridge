#!/usr/bin/env python3
"""Dekodiert alle Aufnahmen mit FESTER Symbolrate (38400 bit/s) und richtet
jeden Frame am Sync-Wort aus. Nur so sind die Frames untereinander
vergleichbar - eine pro Burst geschaetzte Rate verschiebt die Bitgrenzen.

Aufruf:  .venv/bin/python3 tools/compare.py
         .venv/bin/python3 tools/compare.py captures/Kamin-HOCH-*.complex16s
"""
import glob
import sys
from collections import defaultdict

import numpy as np
from scipy.signal import firwin, lfilter

FS = 1_000_000.0
BAUD = 38_400.0
SPS = FS / BAUD
SYNC = "0110100111001000"      # 0x69C8, direkt nach der 1010..-Praambel


def load(path):
    raw = np.fromfile(path, dtype=np.int16).astype(np.float64) / 32768.0
    return raw[0::2] + 1j * raw[1::2]


def find_bursts(iq):
    power = np.convolve(np.abs(iq)**2, np.ones(64)/64, mode="same")
    nz = power[power > 0]
    noise = np.percentile(nz, 60) if len(nz) else 1e-12
    idx = np.where(power > noise*10)[0]
    if len(idx) == 0:
        return []
    groups = np.split(idx, np.where(np.diff(idx) > 800)[0]+1)
    out = [(g[0], g[-1], power[g[0]:g[-1]].max()/noise)
           for g in groups if len(g) >= 300]
    return sorted(out, key=lambda b: b[0])


def demod(iq, s, e):
    pad = 120
    s, e = max(0, s-pad), min(len(iq), e+pad)
    seg = iq[s:e]
    mag = np.abs(seg)
    on = np.where(mag > mag.max()*0.3)[0][0]
    off = np.where(mag > mag.max()*0.3)[0][-1]
    inst0 = np.diff(np.unwrap(np.angle(seg)))*FS/(2*np.pi)
    w = mag[1:]**2
    fc = np.sum(inst0*w)/np.sum(w)
    t = np.arange(len(seg))
    bb = seg*np.exp(-1j*2*np.pi*fc/FS*t)
    bb = lfilter(firwin(63, 130e3/(FS/2)), 1, bb)
    bbn = bb/(np.abs(bb)+1e-9)
    inst = np.angle(bbn[1:]*np.conj(bbn[:-1]))*FS/(2*np.pi)
    return inst[on:off]


def bits_at_offset(inst, thr, phase):
    """Bits mit fester Symboldauer, Startphase in Samples."""
    n = int((len(inst)-phase)/SPS)
    out = []
    for i in range(n):
        c = phase + (i+0.5)*SPS
        a, b = int(c-SPS*0.35), int(c+SPS*0.35)
        if b > len(inst):
            break
        out.append(1 if inst[a:b].mean() > thr else 0)
    return "".join(map(str, out))


def refine_timing(inst, thr):
    """Symboldauer und Phase aus allen Flanken des Frames per Regression.

    Jede Flanke liegt auf einem Vielfachen der Symboldauer; eine Ausgleichs-
    gerade durch (Symbolindex, Flankenposition) liefert SPS deutlich genauer
    als eine feste Vorgabe und faengt den Taktdrift ueber den Frame ab.
    """
    sign = inst > thr
    edges = np.where(np.diff(sign.astype(int)) != 0)[0].astype(float)
    if len(edges) < 8:
        return SPS, 0.0
    sps = SPS
    for _ in range(3):                       # wenige Iterationen genuegen
        k = np.round((edges - edges[0]) / sps)
        keep = np.abs((edges - edges[0]) - k*sps) < sps*0.3   # Ausreisser raus
        if keep.sum() < 4:
            break
        A = np.vstack([k[keep], np.ones(keep.sum())]).T
        sps, off = np.linalg.lstsq(A, edges[keep], rcond=None)[0]
    return sps, (edges[0] + off - edges[0]) % sps


def bits_fixed(inst, thr, sps, phase):
    n = int((len(inst)-phase)/sps)
    out = []
    for i in range(n):
        c = phase + (i+0.5)*sps
        a, b = int(c-sps*0.35), int(c+sps*0.35)
        if b > len(inst):
            break
        out.append(1 if inst[a:b].mean() > thr else 0)
    return "".join(map(str, out))


def best_frame(inst):
    """Symbolrate verfeinern, beide Polaritaeten pruefen, am SYNC ausrichten.

    Die FSK-Polaritaet haengt davon ab, auf welcher Seite die Mittenfrequenz
    geschaetzt wurde - je nach Aufnahme ist das Signal invertiert.
    """
    hi, lo = np.percentile(inst, 85), np.percentile(inst, 15)
    thr = (hi+lo)/2
    sps, _ = refine_timing(inst, thr)
    best = None
    for phase in np.arange(0, sps, 0.2):
        s = bits_fixed(inst, thr, sps, phase)
        for cand in (s, s.translate(str.maketrans("01", "10"))):
            i = cand.find(SYNC)
            if i >= 0 and (best is None or i < best[0]):
                best = (i, cand[i:])
    return best[1] if best else None


def to_hex(bits):
    n = (len(bits)//8)*8
    return bytes(int(bits[i:i+8], 2) for i in range(0, n, 8)).hex()


# Aufnahmen mit abweichender Samplerate (die Juni-Erstaufnahme lief mit 500 kHz)
FS_OVERRIDE = {"Kamin-An.complex16s": 500_000.0}


def main(paths):
    global FS, SPS
    frames = defaultdict(list)
    for p in paths:
        name = p.split("/")[-1]
        FS = FS_OVERRIDE.get(name, 1_000_000.0)
        SPS = FS / BAUD
        cmd = name.split("-")[1].split(".")[0]
        iq = load(p)
        for s, e, pk in find_bursts(iq):
            f = best_frame(demod(iq, s, e))
            if f:
                frames[cmd].append((s/FS, f))

    for cmd in sorted(frames):
        print(f"\n=== {cmd} ({len(frames[cmd])} Frames mit Sync) ===")
        hexes = []
        for t, f in frames[cmd]:
            h = to_hex(f)
            hexes.append(h)
            print(f"  @{t:5.2f}s  {h}")
        if len(hexes) > 1:
            n = min(len(h) for h in hexes)
            same = "".join("." if len(set(h[i] for h in hexes)) == 1 else "X"
                           for i in range(n))
            print(f"  konstant: {same}")


if __name__ == "__main__":
    paths = sys.argv[1:] or sorted(glob.glob("captures/Kamin-*-2026*.complex16s"))
    main(paths)
