#!/usr/bin/env python3
"""Check and compute the checksum of a fireplace telegram.

The formula:

    CRC16(sync + telegram + 1 bit, poly=0x8005, init=0x2e81) == CONSTANT

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

    Type   Meaning             Length*          CONSTANT
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
TYPES = {(0xF7, 0xB9): (21, 0x4993),
         (0xF9, 0x39): (18, 0xEDDD),
         (0xFA, 0x39): (16, 0x3AAE)}


def crc16(data, extrabit=None):
    """CRC over the bytes; extrabit appends a single bit at the end."""
    c = INIT
    for byte in data:
        c ^= byte << 8
        for _ in range(8):
            c = ((c << 1) ^ POLY) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    if extrabit is not None:
        top = (c >> 15) & 1
        c = (c << 1) & 0xFFFF
        if top ^ extrabit:
            c ^= POLY
    return c


def residue(data, extrabit=0):
    return crc16(data, extrabit)


def telegram_type(f):
    """(length, constant) from the address bytes; None for an unknown type."""
    return TYPES.get((f[2], f[3])) if len(f) > 3 else None


def is_valid(f):
    """True if checksum and data match (f = frame from sync).

    The 17th check bit is read from the trailing byte; if it is missing, both
    values are accepted (the check is then one bit weaker).
    """
    t = telegram_type(f)
    if not t:
        return False
    length, constant = t
    if len(f) < length:
        return False
    if len(f) > length:
        return residue(f[:length], (f[length] >> 7) & 1) == constant
    return any(residue(f[:length], e) == constant for e in (0, 1))


def compute(data, extrabit=0):
    """Find the two checksum bytes for a telegram without checksum.

    The appended check bit is free to choose - both variants are valid as long
    as the bit is sent too. The firmware uses 0.
    """
    t = telegram_type(data)
    if not t:
        return None
    length, constant = t
    if len(data) != length - 2:
        return None
    for hi in range(256):
        for lo in range(256):
            if residue(list(data) + [hi, lo], extrabit) == constant:
                return hi, lo
    return None


def self_test():
    frames = set()
    for path in LOGS:
        try:
            for line in open(path, errors="replace"):
                m = re.search(r"dBm\s+([0-9a-f]{40,})", line)
                if not m:
                    continue
                h = m.group(1)
                b = tuple(int(h[i:i+2], 16) for i in range(0, len(h), 2))
                if len(b) == 22 and (b[2], b[3]) in TYPES:
                    frames.add(b)
        except FileNotFoundError:
            pass
    if not frames:
        sys.exit("no recordings found")
    for addr, (length, constant) in TYPES.items():
        f = [x for x in frames if (x[2], x[3]) == addr]
        ok = sum(1 for x in f if is_valid(list(x)))
        print(f"{addr[0]:02x}{addr[1]:02x}  {len(f):4d} frames, {ok} valid "
              f"(length {length}, constant {constant:#06x})")


def main():
    if len(sys.argv) > 1:
        h = sys.argv[1].replace(" ", "")
        b = [int(h[i:i+2], 16) for i in range(0, len(h), 2)]
        t = telegram_type(b)
        if not t:
            print("unknown telegram type - expected the frame from sync 69c8")
            return
        length, _ = t
        if len(b) >= length:
            print("valid" if is_valid(b) else "CHECKSUM WRONG")
        elif len(b) == length - 2:
            hi, lo = compute(b)
            print(f"checksum: {hi:02x}{lo:02x}")
            print(f"complete: {h}{hi:02x}{lo:02x}")
        else:
            print(f"expected {length-2} bytes (to compute) or {length} (to check)")
        return
    self_test()


if __name__ == "__main__":
    main()
