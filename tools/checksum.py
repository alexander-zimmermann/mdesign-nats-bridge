#!/usr/bin/env python3
"""Check and compute the checksum of a fireplace telegram.

The formula:

    CRC16(sync + telegram + 1 bit, poly=0x8005, init=0x2e81) == KONST

computed over the *whole* telegram including sync word and checksum.
Polynomial and init come from the reverse engineering of a related device
(github.com/erdebee/mertik-868-protocol).

Three pitfalls:

  * The telegram types have DIFFERENT LENGTHS. Because of PKTLEN the firmware
    always reads 20 bytes; computing over the full read length never matches
    for the short types.
  * The CRC runs over the whole telegram including sync and checksum and
    then yields a constant - not over the data alone.
  * NO telegram ends on a byte boundary. Exactly ONE check bit follows the
    last byte (the top bit of the trailing byte, easily mistaken for noise).
    Without this bit the checksum has two valid solutions, 0xc002 apart -
    with it exactly one.

    Type   Meaning             Length*         KONST
    f7b9   set value/status    21 bytes + 1 bit 0x4993
    f939   switch command      18 bytes + 1 bit 0xeddd
    fa39   status reply        16 bytes + 1 bit 0x3aae
    (* from the sync word, i.e. payload + 2)

The self-test checks every frame in the recorded logs.

Sending must include the 17th bit - otherwise the sender leaves noise there
and the receiver drops about half of the telegrams.

Usage:  python3 tools/checksum.py               self-test against the recordings
        python3 tools/checksum.py <hex>         checksum for one telegram
"""
import re
import sys

POLY, INIT = 0x8005, 0x2E81
LOGS = ["captures/stufen.log", "captures/nacht.log"]

# Type -> (total length from sync in bytes, residue)
# The length ALWAYS includes a 17th bit after the last byte.
TYPEN = {(0xF7, 0xB9): (21, 0x4993),
         (0xF9, 0x39): (18, 0xEDDD),
         (0xFA, 0x39): (16, 0x3AAE)}


def crc16(daten, extrabit=None):
    """CRC over the bytes; extrabit appends a single bit at the end."""
    c = INIT
    for byte in daten:
        c ^= byte << 8
        for _ in range(8):
            c = ((c << 1) ^ POLY) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    if extrabit is not None:
        top = (c >> 15) & 1
        c = (c << 1) & 0xFFFF
        if top ^ extrabit:
            c ^= POLY
    return c


def rest(daten, extrabit=0):
    return crc16(daten, extrabit)


def typ(f):
    """(length, constant) from the address bytes; None for an unknown type."""
    return TYPEN.get((f[2], f[3])) if len(f) > 3 else None


def gueltig(f):
    """True if checksum and data match (f = frame from sync).

    The 17th check bit is read from the trailing byte; if it is missing, both
    values are accepted (the check is then one bit weaker).
    """
    t = typ(f)
    if not t:
        return False
    laenge, konst = t
    if len(f) < laenge:
        return False
    if len(f) > laenge:
        return rest(f[:laenge], (f[laenge] >> 7) & 1) == konst
    return any(rest(f[:laenge], e) == konst for e in (0, 1))


def berechne(daten, extrabit=0):
    """Find the two checksum bytes for a telegram without checksum.

    The appended check bit is free to choose - both variants are valid as long
    as the bit is sent too. The firmware uses 0.
    """
    t = typ(daten)
    if not t:
        return None
    laenge, konst = t
    if len(daten) != laenge - 2:
        return None
    for hi in range(256):
        for lo in range(256):
            if rest(list(daten) + [hi, lo], extrabit) == konst:
                return hi, lo
    return None


def selbsttest():
    frames = set()
    for pfad in LOGS:
        try:
            for zeile in open(pfad, errors="replace"):
                m = re.search(r"dBm\s+([0-9a-f]{40,})", zeile)
                if not m:
                    continue
                h = m.group(1)
                b = tuple(int(h[i:i+2], 16) for i in range(0, len(h), 2))
                if len(b) == 22 and (b[2], b[3]) in TYPEN:
                    frames.add(b)
        except FileNotFoundError:
            pass
    if not frames:
        sys.exit("no recordings found")
    for adr, (laenge, konst) in TYPEN.items():
        f = [x for x in frames if (x[2], x[3]) == adr]
        ok = sum(1 for x in f if gueltig(list(x)))
        print(f"{adr[0]:02x}{adr[1]:02x}  {len(f):4d} frames, {ok} valid "
              f"(length {laenge}, constant {konst:#06x})")


def main():
    if len(sys.argv) > 1:
        h = sys.argv[1].replace(" ", "")
        b = [int(h[i:i+2], 16) for i in range(0, len(h), 2)]
        t = typ(b)
        if not t:
            print("unknown telegram type - expected the frame from sync 69c8")
            return
        laenge, _ = t
        if len(b) >= laenge:
            print("valid" if gueltig(b) else "CHECKSUM WRONG")
        elif len(b) == laenge - 2:
            hi, lo = berechne(b)
            print(f"checksum: {hi:02x}{lo:02x}")
            print(f"complete: {h}{hi:02x}{lo:02x}")
        else:
            print(f"expected {laenge-2} bytes (to compute) or {laenge} (to check)")
        return
    selbsttest()


if __name__ == "__main__":
    main()
