/*
 * Fireplace radio bridge: ESP32 + CC1101 on 868.3 MHz.
 *
 * Serial commands:
 *   r     receive: print every packet as hex. The CC1101 recovers clock and
 *         bits itself, so the remote's frames come out bit-exact.
 *   1/2   ON / OFF (switch commands on the second address f939)
 *   3     OFF, replaying recorded remote frames unchanged
 *   a..o  set power level 1..15 (a = level 1, o = level 15)
 *   t     level test 15-1-14-1-...: every level against level 1 so the step
 *         is visible. Any key aborts.
 *   v     replay the last received remote set command unchanged
 *   x     toggle copying the live reading into sent f7b9 frames (on at boot)
 *   ?     register dump
 *
 * Measured radio parameters: 868.3 MHz, 2-FSK, 38.4 kbit/s, +-19 kHz
 * deviation, sync 0x69C8.
 */
#include <Arduino.h>
#include <SPI.h>

// ---- Wiring (VSPI, see README) ---------------------------------------------
constexpr int PIN_SCK = 18;
constexpr int PIN_MISO = 19;
constexpr int PIN_MOSI = 23;
constexpr int PIN_CS = 5;
constexpr int PIN_GDO0 = 17;
constexpr int PIN_GDO2 = 16;

// ---- CC1101 register addresses ---------------------------------------------
// clang-format off
enum : uint8_t {
  IOCFG2 = 0x00, IOCFG0 = 0x02, FIFOTHR = 0x03, SYNC1 = 0x04, SYNC0 = 0x05,
  PKTLEN = 0x06, PKTCTRL1 = 0x07, PKTCTRL0 = 0x08, CHANNR = 0x0A,
  FSCTRL1 = 0x0B, FSCTRL0 = 0x0C, FREQ2 = 0x0D, FREQ1 = 0x0E, FREQ0 = 0x0F,
  MDMCFG4 = 0x10, MDMCFG3 = 0x11, MDMCFG2 = 0x12, MDMCFG1 = 0x13,
  MDMCFG0 = 0x14, DEVIATN = 0x15, MCSM1 = 0x17, MCSM0 = 0x18,
  FOCCFG = 0x19, BSCFG = 0x1A, AGCCTRL2 = 0x1B, AGCCTRL1 = 0x1C,
  AGCCTRL0 = 0x1D, FREND1 = 0x21, FREND0 = 0x22, FSCAL3 = 0x23,
  FSCAL2 = 0x24, FSCAL1 = 0x25, FSCAL0 = 0x26, TEST2 = 0x2C, TEST1 = 0x2D,
  TEST0 = 0x2E, PATABLE = 0x3E, TXFIFO = 0x3F, RXFIFO = 0x3F,
};
// Command strobes
enum : uint8_t {
  SRES = 0x30, SCAL = 0x33, SRX = 0x34, STX = 0x35, SIDLE = 0x36,
  SFRX = 0x3A, SFTX = 0x3B, SNOP = 0x3D,
};
// Status registers (need the burst bit)
enum : uint8_t {
  PARTNUM = 0x30, VERSION = 0x31, RSSI = 0x34, MARCSTATE = 0x35,
  RXBYTES = 0x3B,
};
// clang-format on

SPISettings spiCfg(4000000, MSBFIRST, SPI_MODE0);

// ---- Register configuration -------------------------------------------------
// Computed from the measured parameters (26 MHz crystal). No whitening, no CRC
// and a fixed length: the chip hands over raw bits, and its CRC filter would
// drop every packet, since the telegrams use a different checksum.
// clang-format off
const uint8_t REGS[][2] = {
  {IOCFG2,   0x2E},  // GDO2 unused (tri-state)
  {IOCFG0,   0x06},  // GDO0: high on sync, low at packet end
  {FIFOTHR,  0x47},
  {SYNC1,    0x69}, {SYNC0, 0xC8},   // measured sync word
  {PKTLEN,   0x14},  // fixed 20-byte payload
  {PKTCTRL1, 0x00},  // no address filter, no status bytes appended
  {PKTCTRL0, 0x00},  // fixed length, no whitening, no CRC
  {CHANNR,   0x00},
  {FSCTRL1,  0x06}, {FSCTRL0, 0x00},
  {FREQ2,    0x21}, {FREQ1, 0x65}, {FREQ0, 0x6A},   // 868.2999 MHz
  {MDMCFG4,  0xCA},  // channel bandwidth 102 kHz, DRATE_E=10
  {MDMCFG3,  0x83},  // 38383 bit/s
  {MDMCFG2,  0x02},  // 2-FSK, no Manchester, 16/16 sync
  {MDMCFG1,  0x22},  // 4-byte preamble (32 bit, as measured)
  {MDMCFG0,  0xF8},
  {DEVIATN,  0x34},  // +-19.04 kHz
  {MCSM1,    0x30},  // back to IDLE after RX/TX
  {MCSM0,    0x18},  // calibrate when leaving IDLE
  {FOCCFG,   0x16}, {BSCFG, 0x6C},
  {AGCCTRL2, 0x43}, {AGCCTRL1, 0x40}, {AGCCTRL0, 0x91},
  {FREND1,   0x56}, {FREND0, 0x10},
  {FSCAL3,   0xE9}, {FSCAL2, 0x2A}, {FSCAL1, 0x00}, {FSCAL0, 0x1F},
  {TEST2,    0x81}, {TEST1, 0x35}, {TEST0, 0x09},
};
// clang-format on

// ---- Recorded frames (payload AFTER the sync word) --------------------------
// Recorded bit-exact in receive mode. Layout of a command:
//   [0..1]   f7b9            system address
//   [2]      8c..8f          sequence counter, cycles c->d->e->f
//   [3]      cc/4c           toggle bit (bit 7): position within a sequence slot
//   [4..10]  780999ef3c2d69  command body (status telegrams: 0b8b989cbe2d69)
//   [11..14] set value, changes with the power level
//   [17..18] checksum
// The payload is scrambled (not CC1101 PN9 with the default seed); replay
// sends it back unchanged.
//
// Telegram lengths (see tools/checksum.py) differ, and none ends on a byte
// boundary: each carries ONE check bit after its last byte.
//   f7b9 set value/status  19 bytes + 1 bit
//   f939 switch command    16 bytes + 1 bit
//   fa39 status reply      14 bytes + 1 bit
// Without this 17th check bit the checksum has two valid solutions, with it
// exactly one. Sending therefore appends one byte whose top bit carries the
// check bit; the receiver ignores the rest. Sending only the whole bytes
// leaves noise there, and the receiver drops about half of the telegrams.
struct Telegram {
  uint8_t len;
  uint8_t slot;
  uint8_t b[19];
};

// ---- Switch sequences -------------------------------------------------------
// Switch commands use a SECOND address (f939 instead of f7b9), and the remote
// sends fixed sequences rather than single telegrams. Per the manufacturer's
// manual, switching off is a burner cascade: side burner, then main burner,
// then pilot, hence three switch commands.
//
// The remote assigns ONE sequence number per slot and sends two telegrams in
// it (switch command + set value, told apart by the toggle bit). The number
// cycles 8c->8d->8e->8f. The sequence numbers below are placeholders: sending
// sets them from the running counter and recomputes the checksum.
// Template: a recorded remote sequence that switches the fire on.
// clang-format off
const Telegram SEQ_ON[] = {
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xC2,0xAB,0xFA,0xAA,0x3F,0x5E}}, // idle
  {19,0,{0xF7,0xB9,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xC2,0xAB,0xB7,0x67,0x93,0x75}}, // level 15
  {16,1,{0xF9,0x39,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xE8,0xE2,0xB5,0x0E,0x52,0x6A}},                // ON
  {19,1,{0xF7,0xB9,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xC2,0xAB,0xB7,0x67,0x90,0x7F}}, // level 15
};
// Switching off: an idle telegram before EVERY switch command, as the remote
// does. An f939 switch command is only 16 bytes and has no reading field, so
// only the preceding f7b9 telegram, which carries the live reading, shows that
// the sender is in sync. With a single idle telegram up front, only the first
// switch command takes effect.
const Telegram SEQ_OFF[] = {
  {19,0,{0xF7,0xB9,0x8E,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0x8B,0x2B,0xFA,0xAA,0x40,0x44}}, // idle
  {16,0,{0xF9,0x39,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xE9,0xE3,0xAC,0xBA,0x33,0x06}},                // side burner off
  {19,1,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0x8B,0x2B,0xFA,0xAA,0x41,0x42}}, // idle
  {16,1,{0xF9,0x39,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xED,0x67,0x46,0xB0,0xD5,0x1B}},                // main burner off
  {19,2,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0x8B,0x2B,0xFA,0xAA,0x42,0x48}}, // idle
  {16,2,{0xF9,0x39,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xEC,0x66,0x5F,0x04,0xD4,0x36}},                // pilot off
};
// clang-format on
constexpr uint8_t N_SEQ_ON = sizeof(SEQ_ON) / sizeof(SEQ_ON[0]);
constexpr uint8_t N_SEQ_OFF = sizeof(SEQ_OFF) / sizeof(SEQ_OFF[0]);

// Power levels 1..15. Stepping down and up yields the same values, so the
// encoding is absolute. Index 0 = level 1. Byte 17 carries the set value.
// clang-format off
const Telegram LEVELS[] = {
  {19,0,{0xF7,0xB9,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x44,0xAB,0x85,0x55,0x86,0x99}},  // level  1
  {19,0,{0xF7,0xB9,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x44,0xAB,0x86,0xD6,0x0D,0x9C}},  // level  2
  {19,0,{0xF7,0xB9,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x53,0x2B,0x82,0x52,0xB2,0x8D}},  // level  3
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x44,0xAB,0x8F,0xDF,0x38,0x23}},  // level  4
  {19,0,{0xF7,0xB9,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x4D,0x2B,0x8B,0x5B,0x9E,0xB2}},  // level  5
  {19,0,{0xF7,0xB9,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x69,0xAB,0x94,0xC4,0x46,0xF7}},  // level  6
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x4D,0x2B,0x90,0x40,0xC7,0x61}},  // level  7
  {19,0,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xB5,0x2B,0x9C,0x4C,0x8C,0x13}},  // level  8
  {19,0,{0xF7,0xB9,0x8E,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xB5,0x2B,0x99,0xC9,0x13,0x02}},  // level  9
  {19,0,{0xF7,0xB9,0x8D,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xB5,0x2B,0xA5,0x75,0xDB,0x02}},  // level 10
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xBC,0x2B,0xA6,0xF6,0x24,0x86}},  // level 11
  {19,0,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x66,0xAB,0xA2,0x72,0xBE,0xC9}},  // level 12
  {19,0,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xA5,0xAB,0xAF,0xFF,0x6F,0xBF}},  // level 13
  {19,0,{0xF7,0xB9,0x8E,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xA5,0xAB,0xAB,0x7B,0xF6,0xA8}},  // level 14
  {19,0,{0xF7,0xB9,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xA2,0xAB,0xB7,0x67,0x92,0x62}},  // level 15
};
// clang-format on
constexpr uint8_t N_LEVELS = sizeof(LEVELS) / sizeof(LEVELS[0]);

// ---- SPI basics -------------------------------------------------------------
static void csLow() {
  digitalWrite(PIN_CS, LOW);
  uint32_t t0 = micros();
  // wait until the chip is ready
  while (digitalRead(PIN_MISO) && micros() - t0 < 2000) {
  }
}
static void csHigh() {
  digitalWrite(PIN_CS, HIGH);
}

static void strobe(uint8_t cmd) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(cmd);
  csHigh();
  SPI.endTransaction();
}
static void writeReg(uint8_t addr, uint8_t val) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(addr);
  SPI.transfer(val);
  csHigh();
  SPI.endTransaction();
}
static uint8_t readReg(uint8_t addr) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(addr | 0x80);
  uint8_t v = SPI.transfer(0);
  csHigh();
  SPI.endTransaction();
  return v;
}
static uint8_t readStatus(uint8_t addr) {  // status registers need 0xC0
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(addr | 0xC0);
  uint8_t v = SPI.transfer(0);
  csHigh();
  SPI.endTransaction();
  return v;
}
static void burstWrite(uint8_t addr, const uint8_t *buf, uint8_t n) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(addr | 0x40);
  for (uint8_t i = 0; i < n; i++) SPI.transfer(buf[i]);
  csHigh();
  SPI.endTransaction();
}
static void burstRead(uint8_t addr, uint8_t *buf, uint8_t n) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(addr | 0xC0);
  for (uint8_t i = 0; i < n; i++) buf[i] = SPI.transfer(0);
  csHigh();
  SPI.endTransaction();
}

static void configure() {
  strobe(SRES);
  delay(10);
  for (auto &r : REGS) writeReg(r[0], r[1]);
  uint8_t pa[] = {0xC0};  // maximum TX power at 868 MHz
  burstWrite(PATABLE, pa, 1);
  strobe(SCAL);
  delay(5);
}

// ---- Runtime ----------------------------------------------------------------
constexpr int RSSI_MIN = -90;  // dBm; anything weaker is noise
uint32_t rxNoise = 0;          // dropped false triggers (too weak)
uint32_t rxCorrupt = 0;        // dropped for a bad checksum
bool rxMode = false;

static void startRx() {
  strobe(SIDLE);
  strobe(SFRX);
  strobe(SRX);
  rxMode = true;
  Serial.printf("RX on (packets above %d dBm only) - press a key on the remote.\n", RSSI_MIN);
}

// ---- Checksum ---------------------------------------------------------------
// CRC-16, poly 0x8005, init 0x2e81, over the whole telegram from the sync word,
// including the checksum AND the appended 17th bit; the result is a
// length-dependent constant. Without the 17th bit the equation has two
// solutions (0xc002 apart), with it the checksum is unique.
constexpr uint16_t CRC_POLY = 0x8005, CRC_INIT = 0x2E81;
static uint16_t crcTab[256];

static void crcTabInit() {
  for (uint16_t i = 0; i < 256; i++) {
    uint16_t c = i << 8;
    for (uint8_t j = 0; j < 8; j++) c = (c & 0x8000) ? ((c << 1) ^ CRC_POLY) : (c << 1);
    crcTab[i] = c;
  }
}
static inline uint16_t crcByte(uint16_t c, uint8_t b) {
  return (uint16_t)(c << 8) ^ crcTab[((c >> 8) ^ b) & 0xFF];
}
static inline uint16_t crcBit(uint16_t c, uint8_t bit) {
  uint8_t top = (c >> 15) & 1;
  c <<= 1;
  return (top ^ bit) ? (c ^ CRC_POLY) : c;
}
static uint16_t checksumConstant(uint8_t len) {  // len = payload without sync
  return len == 19 ? 0x4993 : len == 16 ? 0xEDDD : 0x3AAE;
}

// Sets the two checksum bytes so the telegram with an appended 0 bit checks
// out. Returns false if there is no solution; then nothing is sent rather than
// an invalid telegram.
static bool setChecksum(uint8_t *p, uint8_t len) {
  uint16_t prefix = CRC_INIT;
  prefix = crcByte(prefix, 0x69);
  prefix = crcByte(prefix, 0xC8);
  for (uint8_t i = 0; i + 2 < len; i++) prefix = crcByte(prefix, p[i]);
  const uint16_t target = checksumConstant(len);
  for (uint16_t hi = 0; hi < 256; hi++) {
    uint16_t c1 = crcByte(prefix, (uint8_t)hi);
    for (uint16_t lo = 0; lo < 256; lo++) {
      if (crcBit(crcByte(c1, (uint8_t)lo), 0) == target) {
        p[len - 2] = (uint8_t)hi;
        p[len - 1] = (uint8_t)lo;
        return true;
      }
    }
  }
  return false;
}

// Checks a received telegram. Returns the payload length, or 0 for a bad
// checksum or an unknown address. Required: a single bit error in the reading
// field would poison the fire's state through the live patch.
static uint8_t checksumOk(const uint8_t *buf) {
  uint8_t len;
  uint16_t target;
  if (buf[0] == 0xF7 && buf[1] == 0xB9) {
    len = 19;
    target = 0x4993;
  } else if (buf[0] == 0xF9 && buf[1] == 0x39) {
    len = 16;
    target = 0xEDDD;
  } else if (buf[0] == 0xFA && buf[1] == 0x39) {
    len = 14;
    target = 0x3AAE;
  } else
    return 0;
  uint16_t c = CRC_INIT;
  c = crcByte(c, 0x69);
  c = crcByte(c, 0xC8);
  for (uint8_t i = 0; i < len; i++) c = crcByte(c, buf[i]);
  c = crcBit(c, (buf[len] >> 7) & 1);
  return (c == target) ? len : 0;
}

// ---- Sending ----------------------------------------------------------------
// The remote assigns one sequence number per slot (8c..8f, cyclic) and never
// repeats it back to back. The same counter runs here, so our telegrams do not
// look like repeats.
uint8_t seqCounter = 0x8C;

// PKTLEN applies to sending and receiving alike. The telegrams differ in
// length, so it is set before each send and restored to the receive value
// afterwards. len+1 bytes go out: bit 7 of the last carries the check bit
// (0 here), the receiver ignores the rest.
// One telegram per sequence position, like the remote: the toggle bit marks
// the position within a slot, not a repeat.
constexpr uint8_t REPEATS = 1;          // sends per telegram
constexpr uint32_t REPEAT_PAUSE = 150;  // ms between repeats

// Last overheard remote set command, verbatim including the trailing byte.
// Replaying it unchanged ('v') tests the send chain without any transcription.
uint8_t stored[20];
bool storedOk = false;
uint8_t storedValue = 0;

// Bytes [12..14] of an f7b9 telegram carry the remote's reading, and the fire
// mirrors them unchanged in its status telegrams. Sent f7b9 frames get the
// CURRENT value instead of a stale one from a recording. Presumably the fire
// executes safety-relevant commands only from a sender in sync with it: it
// acknowledges OFF commands with a stale reading but does not carry them out.
uint8_t live[3];
bool liveOk = false;
bool livePatch = true;  // toggled with 'x' for A/B tests

// Raw telegram: a recording including the trailing byte, sent completely
// unchanged (no sequence counter, no recomputed checksum). A control run: if
// this works, any fault lies in what the firmware changes.
struct RawTelegram {
  uint8_t len;
  uint8_t b[20];
};
// clang-format off
const RawTelegram OFF_RAW[] = {
  {19,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xF6,0xAB,0xFA,0xAA,0x2F,0x57,0xF7}},
  {16,{0xF9,0x39,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xE9,0xE3,0xAC,0xBA,0x73,0x78,0x02}},
  {16,{0xF9,0x39,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xED,0x67,0x46,0xB0,0x35,0xA6,0x2D}},
  {16,{0xF9,0x39,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xEC,0x66,0x5F,0x04,0x14,0x34,0xCD}},
};
// clang-format on
constexpr uint8_t N_OFF_RAW = sizeof(OFF_RAW) / sizeof(OFF_RAW[0]);

static void sendFrame(const Telegram &t, uint8_t seq, const char *name, uint8_t repeats = REPEATS) {
  for (uint8_t w = 0; w < repeats; w++) {
    uint8_t f[20];
    memcpy(f, t.b, t.len);
    f[2] = seq;
    if (w & 1) f[3] ^= 0x80;  // flip the toggle bit like the remote
    if (livePatch && liveOk && f[0] == 0xF7) {
      f[12] = live[0];
      f[13] = live[1];
      f[14] = live[2];
    }
    if (!setChecksum(f, t.len)) {
      Serial.printf("%s NOT sent - no valid checksum\n", name);
      return;
    }
    f[t.len] = 0x00;  // check bit 0 + padding

    rxMode = false;
    strobe(SIDLE);
    strobe(SFTX);
    writeReg(PKTLEN, t.len + 1);
    burstWrite(TXFIFO, f, t.len + 1);
    strobe(STX);
    uint32_t t0 = millis();
    while ((readStatus(MARCSTATE) & 0x1F) != 0x01 && millis() - t0 < 200) {
    }
    strobe(SIDLE);
    writeReg(PKTLEN, 0x14);
    if (w + 1 < repeats) delay(REPEAT_PAUSE);
  }
  Serial.printf("%s sent (seq %02x, %u bytes + 1 bit, %ux)\n", name, seq, t.len, repeats);
}

static void pollRx();  // defined below

// Sends a raw telegram completely unchanged, trailing byte included.
static void sendRoh(const RawTelegram &t, const char *name) {
  rxMode = false;
  strobe(SIDLE);
  strobe(SFTX);
  writeReg(PKTLEN, t.len + 1);
  burstWrite(TXFIFO, t.b, t.len + 1);
  strobe(STX);
  uint32_t t0 = millis();
  while ((readStatus(MARCSTATE) & 0x1F) != 0x01 && millis() - t0 < 200) {
  }
  strobe(SIDLE);
  writeReg(PKTLEN, 0x14);
  Serial.printf("%s sent verbatim (%u bytes + 1 bit)\n", name, t.len);
}

// Keeps receiving during the pause, so the fire's reply telegrams (address
// fa39) show up between sends.
static void receiveFor(uint32_t ms) {
  strobe(SIDLE);
  strobe(SFRX);
  strobe(SRX);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) pollRx();
  strobe(SIDLE);
}

static void sendSeq(const Telegram *seq, uint8_t n, const char *name) {
  const uint8_t base = seqCounter;
  uint8_t maxSlot = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (seq[i].slot > maxSlot) maxSlot = seq[i].slot;
    uint8_t s = 0x8C | ((base + seq[i].slot) & 0x03);
    char txt[28];
    snprintf(txt, sizeof(txt), "%s %u/%u", name, i + 1, n);
    sendFrame(seq[i], s, txt);
    if (i + 1 < n) receiveFor(300);
  }
  seqCounter = 0x8C | ((base + maxSlot + 1) & 0x03);
}

// A 16-bit sync matches random noise about every 2^16 bits, roughly every
// 1.7 s at 38.4 kbit/s. Such false triggers sit at the sensitivity limit
// (~-105 dBm), real packets well above it.
static void pollRx() {
  uint8_t n = readStatus(RXBYTES) & 0x7F;
  if (n < 20) return;
  uint8_t buf[20];
  burstRead(RXFIFO, buf, 20);
  int rssi = ((int8_t)readStatus(RSSI) / 2) - 74;
  strobe(SIDLE);
  strobe(SFRX);
  strobe(SRX);  // for the next packet

  if (rssi < RSSI_MIN) {
    rxNoise++;
    return;
  }

  // Only process telegrams with a valid checksum.
  bool known = (buf[0] == 0xF7 && buf[1] == 0xB9) || (buf[0] == 0xF9 && buf[1] == 0x39) ||
               (buf[0] == 0xFA && buf[1] == 0x39);
  if (known && !checksumOk(buf)) {
    rxCorrupt++;
    return;
  }

  Serial.printf("RX %4d dBm  69c8", rssi);  // the sync word is not in the FIFO
  for (uint8_t i = 0; i < 20; i++) Serial.printf("%02x", buf[i]);
  if (rxNoise) Serial.printf("   (%lu false triggers", rxNoise);
  if (rxCorrupt) Serial.printf("%s%lu with bit errors", rxNoise ? ", " : "   (", rxCorrupt);
  if (rxNoise || rxCorrupt) Serial.print(" dropped)");

  // Remember the remote's last set command for 'v'.
  if (buf[0] == 0xF7 && buf[4] == 0x78 && buf[15] != 0xFA) {
    memcpy(stored, buf, 20);
    storedOk = true;
    storedValue = buf[15];
    Serial.printf("   << stored: remote set command, value %02x", buf[15]);
  }

  // Fire status telegram: bytes 10/11 (without sync) are a BIT FIELD, not a
  // state code. Three bits have been seen, set in this order while lighting.
  // Their meaning follows the Modbus register map of a related Mertik device
  // (status register 40203: pilot, main burner, wave active), see
  // forum.domoticz.com/viewtopic.php?t=29060.
  if (buf[0] == 0xF7 && buf[4] == 0x0B) {
    uint16_t st = ((uint16_t)buf[10] << 8) | buf[11];
    uint16_t d = st ^ 0x691A;  // 0x691a = all off
    Serial.print("   [");
    if (!d)
      Serial.print("off");
    else {
      if (d & 0x0040) Serial.print("pilot ");
      if (d & 0x0200) Serial.print("main ");
      if (d & 0x0002) Serial.print("Wave ");
      if (d & ~0x0242) Serial.printf("unknown:%04x ", d & ~0x0242);
    }
    Serial.print("]");

    // Byte [12] is only ever bd or bc across 800+ recorded frames. Anything else
    // is corrupt: keep sending the last good value so the fire settles on it.
    if (buf[12] == 0xBD || buf[12] == 0xBC) {
      live[0] = buf[12];
      live[1] = buf[13];
      live[2] = buf[14];
      liveOk = true;
    } else if (liveOk) {
      Serial.printf("   !! reading [12]=%02x implausible, keeping %02x%02x%02x", buf[12], live[0],
                    live[1], live[2]);
    }

    // Change tracker: every byte that moved since the previous status telegram.
    // Sequence, toggle and checksum change with every packet and are skipped.
    static uint8_t previous[19];
    static bool havePrevious = false;
    if (havePrevious) {
      bool first = true;
      for (uint8_t i = 0; i < 19; i++) {
        if (i == 2 || i == 3 || i == 17 || i == 18) continue;
        if (buf[i] == previous[i]) continue;
        if (first) {
          Serial.print("   CHANGE:");
          first = false;
        }
        Serial.printf(" [%u] %02x->%02x", i, previous[i], buf[i]);
      }
    }
    memcpy(previous, buf, 19);
    havePrevious = true;
  }
  Serial.println();
  rxNoise = 0;
  rxCorrupt = 0;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_GDO2, INPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  strobe(SRES);
  delay(10);
  uint8_t part = readStatus(PARTNUM), ver = readStatus(VERSION);
  Serial.printf("\nCC1101 PARTNUM=0x%02X VERSION=0x%02X\n", part, ver);
  if (ver == 0x00 || ver == 0xFF) {
    Serial.println("No answer - check the wiring (VCC to 3V3, not 5V!).");
    return;
  }

  crcTabInit();
  configure();
  Serial.println("868.3 MHz | 38.4 kbit/s | +-19 kHz | sync 0x69C8");
  Serial.printf(
      "r=receive  1=ON  2=OFF  3=OFF verbatim  a..%c=level 1..%d\n"
      "t=level test  v=replay stored  x=live reading  ?=registers\n",
      'a' + N_LEVELS - 1, N_LEVELS);
  startRx();
}

// Alternating level test 15-1-14-1-13-1...: each level against level 1 rather
// than its neighbour, so the jump is big enough to see whether a level takes
// effect at all; between neighbours the valve travel blurs it. The valve
// takes up to 5 s from level 1 to 15, hence the pause.
constexpr uint32_t SETTLE_TIME = 7000;

static bool aborted() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  Serial.println("--- aborted ---");
  return true;
}

static void setLevel(uint8_t level) {
  char name[16];
  snprintf(name, sizeof(name), "level %u", level);
  sendFrame(LEVELS[level - 1], seqCounter, name);
  seqCounter = 0x8C | ((seqCounter + 1) & 0x03);
}

static void levelTest() {
  Serial.printf(
      "\n=== Level test 15-1-14-1-..., %lu ms per step ===\n"
      "Watch: does the flame jump visibly at EVERY level?\n"
      "Any key aborts.\n",
      SETTLE_TIME);
  for (uint8_t s = N_LEVELS; s >= 1; s--) {
    Serial.printf("\n-- level %u --\n", s);
    setLevel(s);
    receiveFor(SETTLE_TIME);
    if (aborted()) return;
    if (s == 1) break;  // level 1 is the resting point
    Serial.println("-- back to 1 --");
    setLevel(1);
    receiveFor(SETTLE_TIME);
    if (aborted()) return;
  }
  Serial.println("\n=== Level test done ===");
}

void loop() {
  if (Serial.available()) {
    char k = Serial.read();
    switch (k) {
      case 'r':
        startRx();
        break;
      case '1':
        sendSeq(SEQ_ON, N_SEQ_ON, "ON");
        startRx();
        break;
      case '2':
        sendSeq(SEQ_OFF, N_SEQ_OFF, "OFF");
        startRx();
        break;
      case 't':
        levelTest();
        startRx();
        break;
      case '3':  // control run: unchanged recording
        Serial.println("\n--- OFF verbatim (recorded remote frames) ---");
        for (uint8_t i = 0; i < N_OFF_RAW; i++) {
          char s2[24];
          snprintf(s2, sizeof(s2), "OFF-raw %u/%u", i + 1, N_OFF_RAW);
          sendRoh(OFF_RAW[i], s2);
          if (i + 1 < N_OFF_RAW) delay(120);
        }
        startRx();
        break;
      case 'v':
        if (!storedOk) {
          Serial.println("nothing stored yet");
          break;
        }
        Serial.printf(
            "\n--- Replaying the stored remote telegram "
            "(value %02x) ---\n",
            storedValue);
        {
          RawTelegram r;
          r.len = 19;
          memcpy(r.b, stored, 20);
          sendRoh(r, "recording");
        }
        startRx();
        break;
      case 'x':
        livePatch = !livePatch;
        Serial.printf("live reading %s\n", livePatch ? "ON" : "OFF");
        break;
      default:
        if (k >= 'a' && k < char('a' + N_LEVELS)) {  // a = level 1 ... o = level 15
          char name[16];
          snprintf(name, sizeof(name), "level %d", k - 'a' + 1);
          sendFrame(LEVELS[k - 'a'], seqCounter, name);
          seqCounter = 0x8C | ((seqCounter + 1) & 0x03);
          startRx();
        }
        break;
      case '?':
        Serial.printf(
            "MARCSTATE=0x%02X RXBYTES=0x%02X FREQ=%02X%02X%02X "
            "MDMCFG4=%02X MDMCFG3=%02X DEVIATN=%02X\n",
            readStatus(MARCSTATE), readStatus(RXBYTES), readReg(FREQ2), readReg(FREQ1),
            readReg(FREQ0), readReg(MDMCFG4), readReg(MDMCFG3), readReg(DEVIATN));
        break;
    }
  }
  if (rxMode) pollRx();
}
