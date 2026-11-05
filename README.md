# mdesign-nats-bridge

ESP32 firmware that speaks the 868 MHz radio protocol of an M-Design gas
fireplace, in place of its Honeywell DF-RC handheld remote. The goal is a NATS
bridge like the other Lares sidecars.

## Status

Protocol work. The firmware is a serial tool: it prints received frames and
replays recorded on/off and power-level sequences with a fresh sequence number
and checksum. There is no WiFi and no NATS yet, and nothing on the Lares side.

## Hardware

LILYGO T96 (ESP32) plus an 868 MHz CC1101 board with SMA antenna, wired to VSPI:

| CC1101 | ESP32 |
|---|---|
| VCC | 3V3 (**not** 5V) |
| GND | GND |
| SCK | GPIO 18 |
| MISO | GPIO 19 |
| MOSI | GPIO 23 |
| CSN | GPIO 5 |
| GDO0 | GPIO 17 |
| GDO2 | GPIO 16 (unused, tri-state) |

GDO2 collides with the onboard OLED on some T96 revisions; move it if the OLED
comes into use.

## Radio

The fireplace runs Honeywell's decorative-fire control: the DF-RC remote and a
DFGT receiver in the fire. M-Design publishes the
[DF-RC manual](https://m-design.be/downloads/ManualDFRC_English.pdf).

| Parameter | Value |
|---|---|
| Frequency | 868.3 MHz |
| Modulation | 2-FSK, ±19 kHz deviation |
| Data rate | 38.4 kbit/s |
| Preamble | 32 bit |
| Sync word | `0x69C8` |

The payload is scrambled, but not with the CC1101's PN9 default; replay does not
need to undo it. Three telegram types, told apart by the first two bytes after
the sync word:

| Type | Meaning | Payload after sync |
|---|---|---|
| `f7b9` | power level / status | 19 bytes + 1 bit |
| `f939` | switch command | 16 bytes + 1 bit |
| `fa39` | status reply | 14 bytes + 1 bit |

The checksum is a CRC-16 (poly `0x8005`, init `0x2E81`) over sync word, payload,
checksum and the one trailing bit; the result is a constant per type. Polynomial
and init come from the reverse engineering of the related Mertik Maxitrol
868 MHz protocol ([erdebee/mertik-868-protocol](https://github.com/erdebee/mertik-868-protocol)).
[tools/checksum.py](tools/checksum.py) is the reference and verifies it
against the recorded logs.

## Serial commands

115200 baud.

| Key | Action |
|---|---|
| `r` | receive and print frames |
| `1` / `2` | on / off |
| `3` | off, replaying the recorded remote frames unchanged |
| `a` … `o` | power level 1 … 15 |
| `t` | level test 15-1-14-1-… |
| `v` | replay the last received remote frame |
| `x` | toggle copying the live bytes 12–14 into sent `f7b9` frames (on at boot) |
| `?` | register dump |

## Build and flash

```sh
pio run              # build
pio run -t upload    # flash over USB
pio device monitor   # serial log at 115200
```

## Tools

Python helpers, run from the repo root. The SDR scripts need `rtl_sdr`, numpy and
scipy; the serial scripts need pyserial.

```sh
uv venv && uv pip install numpy scipy pyserial
```

| Script | Purpose |
|---|---|
| `capture.py` | record one remote key press with an RTL-SDR into `captures/` |
| `scan.py` | wideband scan around 868.2 MHz to find the carrier |
| `measure.py` | measure symbol rate and deviation of a burst |
| `compare.py` | decode all recordings at the fixed rate, aligned on the sync word |
| `logger.py` | write the ESP32's serial output with timestamps to a file |
| `send.py` | send one firmware command and show what comes back |
| `status.py` | follow status telegrams and show only the bytes that change |
| `stufen.py` | show the remote's command frames with their level byte |
| `checksum.py` | check or compute a checksum; self-test against the logs |

Recordings and logs are not in the repo. The tools expect them in `captures/`,
which is ignored.

## License

GPL-2.0 — see [LICENSE](LICENSE).
