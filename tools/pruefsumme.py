#!/usr/bin/env python3
"""Prueft und berechnet die Pruefsumme eines Kamin-Telegramms.

Die Formel:

    CRC16(Sync + Telegramm + 1 Bit, poly=0x8005, init=0x2e81) == KONST

gerechnet ueber das *gesamte* Telegramm einschliesslich Sync-Wort und
Pruefsumme. Polynom und Startwert stammen aus dem Reverse Engineering eines
verwandten Geraets (github.com/erdebee/mertik-868-protocol).

Drei Fallstricke, an denen frueherer Versuche scheiterten:

  * Die Telegrammtypen sind UNTERSCHIEDLICH LANG. Die Firmware liest wegen
    PKTLEN immer 20 Byte; wer ueber die volle Leselaenge rechnet, bekommt bei
    den kurzen Typen nie ein Ergebnis.
  * Der CRC laeuft ueber das gesamte Telegramm inklusive Sync und Pruefsumme
    und ergibt dann eine Konstante - nicht ueber die Daten allein.
  * KEIN Telegramm endet auf einer Bytegrenze. Hinter dem letzten Byte steht
    noch genau EIN Pruefbit (das oberste Bit des Folgebytes, das wir lange
    fuer Rauschen hielten). Ohne dieses Bit hat die Pruefsumme zwei gueltige
    Loesungen, die sich um 0xc002 unterscheiden - mit ihm genau eine.

    Typ    Bedeutung           Laenge*         KONST
    f7b9   Stellwert/Status    21 Byte + 1 Bit 0x4993
    f939   Schaltbefehl        18 Byte + 1 Bit 0xeddd
    fa39   Statusantwort       16 Byte + 1 Bit 0x3aae
    (* ab Sync-Wort, also Nutzlast + 2)

Verifiziert an 879 Frames aus zwei Mitschnitten: 100 % in allen drei Typen.

Beim Senden muss das 17. Bit mitgeschickt werden - sonst liefert der Sender
dort Rauschen ab und wird in etwa der Haelfte der Faelle verworfen.

Aufruf:  python3 tools/pruefsumme.py               Selbsttest an den Mitschnitten
         python3 tools/pruefsumme.py <hex>         Pruefsumme fuer ein Telegramm
"""
import re
import sys

POLY, INIT = 0x8005, 0x2E81
LOGS = ["captures/stufen.log", "captures/nacht.log"]

# Typ -> (Gesamtlaenge ab Sync in Byte, Restklasse)
# Zur Laenge gehoert IMMER noch ein 17. Bit hinter dem letzten Byte.
TYPEN = {(0xF7, 0xB9): (21, 0x4993),
         (0xF9, 0x39): (18, 0xEDDD),
         (0xFA, 0x39): (16, 0x3AAE)}


def crc16(daten, extrabit=None):
    """CRC ueber die Bytes; extrabit haengt ein einzelnes Bit hinten an."""
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
    """(Laenge, Konstante) anhand der Adressbytes; None bei unbekanntem Typ."""
    return TYPEN.get((f[2], f[3])) if len(f) > 3 else None


def gueltig(f):
    """True, wenn Pruefsumme und Daten zusammenpassen (f = Frame ab Sync).

    Das 17. Pruefbit wird aus dem Folgebyte gelesen; fehlt es, werden beide
    Moeglichkeiten akzeptiert (dann ist die Pruefung um ein Bit schwaecher).
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
    """Ermittelt die zwei Pruefsummenbytes zu einem Telegramm ohne Pruefsumme.

    Das angehaengte Pruefbit ist frei waehlbar - beide Varianten sind gueltig,
    solange das Bit auch gesendet wird. Die Firmware nimmt 0.
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
        sys.exit("keine Mitschnitte gefunden")
    for adr, (laenge, konst) in TYPEN.items():
        f = [x for x in frames if (x[2], x[3]) == adr]
        ok = sum(1 for x in f if gueltig(list(x)))
        print(f"{adr[0]:02x}{adr[1]:02x}  {len(f):4d} Frames, {ok} gueltig "
              f"(Laenge {laenge}, Konstante {konst:#06x})")


def main():
    if len(sys.argv) > 1:
        h = sys.argv[1].replace(" ", "")
        b = [int(h[i:i+2], 16) for i in range(0, len(h), 2)]
        t = typ(b)
        if not t:
            print("unbekannter Telegrammtyp - erwartet wird der Frame ab Sync 69c8")
            return
        laenge, _ = t
        if len(b) >= laenge:
            print("gueltig" if gueltig(b) else "PRUEFSUMME FALSCH")
        elif len(b) == laenge - 2:
            hi, lo = berechne(b)
            print(f"Pruefsumme: {hi:02x}{lo:02x}")
            print(f"vollstaendig: {h}{hi:02x}{lo:02x}")
        else:
            print(f"{laenge-2} Byte erwartet (zum Berechnen) oder {laenge} (zum Pruefen)")
        return
    selbsttest()


if __name__ == "__main__":
    main()
