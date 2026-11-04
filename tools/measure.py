#!/usr/bin/env python3
"""Misst Symbolrate und Deviation eines Bursts, statt sie vorzugeben.

Die Symboldauer kommt aus dem Histogramm der Flankenabstaende: der
kleinste haeufige Abstand ist ein Symbol (in der 1010..-Praambel wechselt
das Signal bei jedem Symbol).

Aufruf:  .venv/bin/python3 tools/measure.py captures/<datei>.complex16s
"""
import sys

import numpy as np
from scipy.signal import firwin, lfilter

FS = 1_000_000.0


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
    return sorted(out, key=lambda b: -b[2])


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
    return inst[on:off], fc


def symbol_rate(inst, thr):
    """Kuerzester wiederkehrender Flankenabstand = eine Symboldauer."""
    sign = inst > thr
    edges = np.where(np.diff(sign.astype(int)) != 0)[0]
    if len(edges) < 8:
        return None, None
    gaps = np.diff(edges)
    gaps = gaps[gaps > 1]                     # Einzel-Sample-Zappler raus
    if len(gaps) == 0:
        return None, None
    # kleinster Abstand, der oft genug vorkommt (>=10% aller Flanken)
    hist = np.bincount(gaps)
    cand = [g for g in range(1, len(hist)) if hist[g] >= 0.10*len(gaps)]
    sps = float(min(cand)) if cand else float(np.percentile(gaps, 10))
    # verfeinern: Mittelwert aller Abstaende nahe dieser Symboldauer
    near = gaps[np.abs(gaps - sps) < sps*0.35]
    if len(near):
        sps = float(near.mean())
    return sps, gaps


def slice_bits(inst, thr, sps):
    sign = inst > thr
    edges = np.where(np.diff(sign.astype(int)) != 0)[0]
    start = edges[0] if len(edges) else 0
    n = int((len(inst)-start)/sps)
    bits = []
    for i in range(n):
        c = start + (i+0.5)*sps
        a, b = int(c-sps*0.35), int(c+sps*0.35)
        bits.append(1 if inst[a:b].mean() > thr else 0)
    return np.array(bits, dtype=np.int8)


def to_hex(bits):
    n = (len(bits)//8)*8
    return bytes(int("".join(map(str, bits[i:i+8])), 2)
                 for i in range(0, n, 8)).hex()


def main(path):
    iq = load(path)
    bursts = find_bursts(iq)
    print(f"{len(bursts)} Bursts in {path.split('/')[-1]}\n")
    for k, (s, e, pk) in enumerate(bursts):
        inst, fc = demod(iq, s, e)
        hi, lo = np.percentile(inst, 85), np.percentile(inst, 15)
        thr = (hi+lo)/2
        dev = (hi-lo)/2
        sps, gaps = symbol_rate(inst, thr)
        if sps is None:
            print(f" #{k} @{s/FS*1e3:8.2f}ms  zu wenige Flanken")
            continue
        baud = FS/sps
        bits = slice_bits(inst, thr, sps)
        bstr = "".join(map(str, bits))
        print(f" #{k} @{s/FS*1e3:8.2f}ms peak{10*np.log10(pk):5.1f}dB "
              f"fc{fc/1e3:+7.0f}k dev+-{dev/1e3:4.1f}k")
        print(f"     SPS={sps:.2f} -> Baud={baud:,.0f} bit/s   {len(bits)} Bits")
        print(f"     bits: {bstr[:96]}...")
        print(f"     hex : {to_hex(bits)}")
        print()


if __name__ == "__main__":
    main(sys.argv[1])
