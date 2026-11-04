/*
 * Kamin-Funkbruecke: ESP32 + CC1101 auf 868,3 MHz.
 *
 * Zwei Betriebsarten, per Serial umschaltbar:
 *   r  - Empfangen: jedes Paket wird als Hex ausgegeben. Damit lassen sich die
 *        Frames des Handsenders bitgenau mitschneiden - deutlich sauberer als
 *        die SDR-Demodulation, weil der CC1101 Takt und Bitentscheidung selbst
 *        macht.
 *   1/2  - AN / AUS (laufen ueber die zweite Adresse f939)
 *   a..o - Leistungsstufe 1..15 setzen (a = Stufe 1, o = Stufe 15)
 *   t  - Stufentest 15-1-14-1-... : jede Stufe gegen Stufe 1 gestellt,
 *        damit der Sprung sichtbar ist. Beliebige Taste bricht ab.
 *   ?  - Registerdump zur Diagnose
 *
 * Die Funkparameter stammen aus eigenen Messungen (2026-07-28):
 * 868,3 MHz, 2-FSK, 38,4 kbit/s, Deviation +-19 kHz, Sync 0x69C8.
 */
#include <Arduino.h>
#include <SPI.h>

// ---- Verdrahtung (VSPI, siehe README) --------------------------------------
constexpr int PIN_SCK = 18;
constexpr int PIN_MISO = 19;
constexpr int PIN_MOSI = 23;
constexpr int PIN_CS = 5;
constexpr int PIN_GDO0 = 17;
constexpr int PIN_GDO2 = 16;

// ---- CC1101 Registeradressen -----------------------------------------------
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
// Command Strobes
enum : uint8_t {
  SRES = 0x30, SCAL = 0x33, SRX = 0x34, STX = 0x35, SIDLE = 0x36,
  SFRX = 0x3A, SFTX = 0x3B, SNOP = 0x3D,
};
// Statusregister (Burst-Bit noetig)
enum : uint8_t {
  PARTNUM = 0x30, VERSION = 0x31, RSSI = 0x34, MARCSTATE = 0x35,
  RXBYTES = 0x3B,
};
// clang-format on

SPISettings spiCfg(4000000, MSBFIRST, SPI_MODE0);

// ---- Registerkonfiguration --------------------------------------------------
// Aus den gemessenen Parametern berechnet (26-MHz-Quarz).
// Bewusst OHNE Whitening, OHNE CRC und mit fester Laenge: wir wollen die
// Rohbits sehen, nicht die Interpretation des Chips. Ein CRC-Filter wuerde
// Pakete verwerfen, deren Pruefsummenverfahren wir noch nicht kennen.
// clang-format off
const uint8_t REGS[][2] = {
  {IOCFG2,   0x2E},  // GDO2 unbenutzt (Tri-State)
  {IOCFG0,   0x06},  // GDO0: HIGH bei Sync, LOW am Paketende
  {FIFOTHR,  0x47},
  {SYNC1,    0x69}, {SYNC0, 0xC8},   // gemessenes Sync-Wort
  {PKTLEN,   0x14},  // 20 Byte feste Nutzlast
  {PKTCTRL1, 0x00},  // kein Adressfilter, kein Status-Anhang
  {PKTCTRL0, 0x00},  // feste Laenge, kein Whitening, kein CRC
  {CHANNR,   0x00},
  {FSCTRL1,  0x06}, {FSCTRL0, 0x00},
  {FREQ2,    0x21}, {FREQ1, 0x65}, {FREQ0, 0x6A},   // 868,2999 MHz
  {MDMCFG4,  0xCA},  // Kanalbandbreite 102 kHz, DRATE_E=10
  {MDMCFG3,  0x83},  // 38.383 bit/s
  {MDMCFG2,  0x02},  // 2-FSK, kein Manchester, 16/16 Sync
  {MDMCFG1,  0x22},  // 4 Byte Praeambel (= 32 bit, wie gemessen)
  {MDMCFG0,  0xF8},
  {DEVIATN,  0x34},  // +-19,04 kHz
  {MCSM1,    0x30},  // nach RX/TX zurueck in IDLE
  {MCSM0,    0x18},  // Autokalibrierung beim Wechsel aus IDLE
  {FOCCFG,   0x16}, {BSCFG, 0x6C},
  {AGCCTRL2, 0x43}, {AGCCTRL1, 0x40}, {AGCCTRL0, 0x91},
  {FREND1,   0x56}, {FREND0, 0x10},
  {FSCAL3,   0xE9}, {FSCAL2, 0x2A}, {FSCAL1, 0x00}, {FSCAL0, 0x1F},
  {TEST2,    0x81}, {TEST1, 0x35}, {TEST0, 0x09},
};
// clang-format on

// ---- Aufgezeichnete Frames (Nutzlast NACH dem Sync-Wort) --------------------
// Bitgenau per Empfangsmodus mitgeschnitten (2026-08-12), nicht mehr aus der
// SDR-Demodulation geschaetzt. Aufbau eines Kommandos:
//   [0..1] f7b9  Systemadresse
//   [2]    8c..8f  Sequenzzaehler, laeuft zyklisch c->d->e->f
//   [3]    cc/4c   Toggle-Bit (Bit 7): jedes Telegramm wird zweimal gesendet
//   [4..10] 780999ef3c2d69  Kommando-Rumpf (bei Statustelegrammen 0b8b989cbe2d69)
//   [11..14] Stellwert, aendert sich mit der Leistungsstufe
//   [17..18] Pruefsumme
// Die Nutzdaten sind verwuerfelt (kein CC1101-PN9 mit Standardstartwert);
// fuer den Replay ist das unerheblich, wir senden sie unveraendert zurueck.
//
// Telegrammlaengen (2026-08-24 bestimmt, siehe tools/pruefsumme.py):
// NICHT alle Telegramme sind gleich lang, und keines endet auf einer
// Bytegrenze - jedes traegt hinter dem letzten Byte noch EIN Pruefbit:
//   f7b9 Stellwert/Status  19 Byte + 1 Bit
//   f939 Schaltbefehl      16 Byte + 1 Bit
//   fa39 Statusantwort     14 Byte + 1 Bit
// Dieses 17. Pruefbit ist unverzichtbar: ohne es hat die Pruefsumme zwei
// gueltige Loesungen, mit ihm genau eine. Beim Senden wird deshalb ein
// zusaetzliches Byte angehaengt, dessen oberstes Bit das Pruefbit traegt;
// den Rest ignoriert der Empfaenger. Wer nur die glatten Bytes sendet,
// liefert dort Rauschen ab und wird in etwa der Haelfte der Faelle verworfen.
struct Telegramm {
  uint8_t len;
  uint8_t slot;
  uint8_t b[19];
};

// ---- Schaltsequenzen --------------------------------------------------------
// Schaltbefehle laufen ueber eine ZWEITE Adresse (f939 statt f7b9). Der
// Handsender sendet dabei keine Einzeltelegramme, sondern feste Folgen.
// Laut Herstellerhandbuch ist Ausschalten eine Brennerkaskade: Nebenbrenner,
// dann Hauptbrenner, dann Zuendflamme - daher drei Schaltbefehle.
//
// Der Handsender vergibt je "Slot" EINE Sequenznummer und sendet darin zwei
// Telegramme (Schaltbefehl + Stellwert, unterschieden am Toggle-Bit). Die
// Nummer laeuft zyklisch 8c->8d->8e->8f. Die Sequenznummern hier sind nur
// Platzhalter - sie werden beim Senden aus dem laufenden Zaehler gesetzt und
// die Pruefsumme danach neu gerechnet.
// Vorlage: Handsender-Mitschnitt vom 2026-08-24 (nachweislich funktionierend).
// clang-format off
const Telegramm SEQ_AN[] = {
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xC2,0xAB,0xFA,0xAA,0x3F,0x5E}}, // Ruhewert
  {19,0,{0xF7,0xB9,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xC2,0xAB,0xB7,0x67,0x93,0x75}}, // Stufe 15
  {16,1,{0xF9,0x39,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xE8,0xE2,0xB5,0x0E,0x52,0x6A}},                // AN
  {19,1,{0xF7,0xB9,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xC2,0xAB,0xB7,0x67,0x90,0x7F}}, // Stufe 15
};
// Ausschalten: VOR JEDEM Schaltbefehl ein Ruhewert-Telegramm, so wie es der
// Handsender im Mitschnitt vom 2026-08-12 macht. Der Grund liegt an der
// Telegrammlaenge: ein f939-Schaltbefehl ist nur 16 Byte lang und hat gar kein
// Messwertfeld. Der Nachweis, dass der Absender synchron ist, kann also nur
// aus dem vorangehenden f7b9-Telegramm kommen - und der wird beim Senden mit
// dem Live-Messwert versehen. Eine Fassung mit nur EINEM Ruhewert vorweg
// brauchte zwei Durchlaeufe: pro Durchlauf kam nur der erste Schaltbefehl an.
const Telegramm SEQ_AUS[] = {
  {19,0,{0xF7,0xB9,0x8E,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0x8B,0x2B,0xFA,0xAA,0x40,0x44}}, // Ruhewert
  {16,0,{0xF9,0x39,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xE9,0xE3,0xAC,0xBA,0x33,0x06}},                // Nebenbrenner aus
  {19,1,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0x8B,0x2B,0xFA,0xAA,0x41,0x42}}, // Ruhewert
  {16,1,{0xF9,0x39,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xED,0x67,0x46,0xB0,0xD5,0x1B}},                // Hauptbrenner aus
  {19,2,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0x8B,0x2B,0xFA,0xAA,0x42,0x48}}, // Ruhewert
  {16,2,{0xF9,0x39,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xEC,0x66,0x5F,0x04,0xD4,0x36}},                // Zuendflamme aus
};
// clang-format on
constexpr uint8_t N_SEQ_AN = sizeof(SEQ_AN) / sizeof(SEQ_AN[0]);
constexpr uint8_t N_SEQ_AUS = sizeof(SEQ_AUS) / sizeof(SEQ_AUS[0]);

// Leistungsstufen 1..15, verifiziert durch Runter- UND Hochlauf (2026-08-12):
// beide Richtungen liefern dieselben Werte, die Kodierung ist also absolut.
// Index 0 = Stufe 1. Byte 17 traegt den Stellwert.
// clang-format off
const Telegramm STUFEN[] = {
  {19,0,{0xF7,0xB9,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x44,0xAB,0x85,0x55,0x86,0x99}},  // Stufe  1
  {19,0,{0xF7,0xB9,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x44,0xAB,0x86,0xD6,0x0D,0x9C}},  // Stufe  2
  {19,0,{0xF7,0xB9,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x53,0x2B,0x82,0x52,0xB2,0x8D}},  // Stufe  3
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x44,0xAB,0x8F,0xDF,0x38,0x23}},  // Stufe  4
  {19,0,{0xF7,0xB9,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x4D,0x2B,0x8B,0x5B,0x9E,0xB2}},  // Stufe  5
  {19,0,{0xF7,0xB9,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x69,0xAB,0x94,0xC4,0x46,0xF7}},  // Stufe  6
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x4D,0x2B,0x90,0x40,0xC7,0x61}},  // Stufe  7
  {19,0,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xB5,0x2B,0x9C,0x4C,0x8C,0x13}},  // Stufe  8
  {19,0,{0xF7,0xB9,0x8E,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xB5,0x2B,0x99,0xC9,0x13,0x02}},  // Stufe  9
  {19,0,{0xF7,0xB9,0x8D,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xB5,0x2B,0xA5,0x75,0xDB,0x02}},  // Stufe 10
  {19,0,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xBC,0x2B,0xA6,0xF6,0x24,0x86}},  // Stufe 11
  {19,0,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBD,0x66,0xAB,0xA2,0x72,0xBE,0xC9}},  // Stufe 12
  {19,0,{0xF7,0xB9,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xA5,0xAB,0xAF,0xFF,0x6F,0xBF}},  // Stufe 13
  {19,0,{0xF7,0xB9,0x8E,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xA5,0xAB,0xAB,0x7B,0xF6,0xA8}},  // Stufe 14
  {19,0,{0xF7,0xB9,0x8E,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xA2,0xAB,0xB7,0x67,0x92,0x62}},  // Stufe 15
};
// clang-format on
constexpr uint8_t N_STUFEN = sizeof(STUFEN) / sizeof(STUFEN[0]);

// ---- SPI-Grundfunktionen ----------------------------------------------------
static void csLow() {
  digitalWrite(PIN_CS, LOW);
  uint32_t t0 = micros();
  // auf Bereitschaft warten
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
static uint8_t readStatus(uint8_t addr) {  // Statusregister brauchen 0xC0
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
  uint8_t pa[] = {0xC0};  // maximale Sendeleistung fuer 868 MHz
  burstWrite(PATABLE, pa, 1);
  strobe(SCAL);
  delay(5);
}

// ---- Betrieb ----------------------------------------------------------------
constexpr int RSSI_MIN = -90;  // dBm; alles Schwaechere ist Rauschen
uint32_t rxNoise = 0;          // verworfene Fehltrigger (zu schwach)
uint32_t rxKaputt = 0;         // verworfen wegen falscher Pruefsumme
bool rxMode = false;

static void startRx() {
  strobe(SIDLE);
  strobe(SFRX);
  strobe(SRX);
  rxMode = true;
  Serial.printf("RX aktiv (nur Pakete ueber %d dBm) - Taste am Handsender druecken.\n", RSSI_MIN);
}

// ---- Pruefsumme -------------------------------------------------------------
// CRC-16, Polynom 0x8005, Startwert 0x2e81, gerechnet ueber das gesamte
// Telegramm ab Sync-Wort einschliesslich Pruefsumme UND des angehaengten
// 17. Bits. Das Ergebnis ist dann eine laengenabhaengige Konstante.
// Ohne das 17. Bit haette die Gleichung zwei Loesungen (sie unterscheiden
// sich um 0xc002) - erst mit ihm ist die Pruefsumme eindeutig.
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
static uint16_t konstante(uint8_t len) {  // len = Nutzlast ohne Sync
  return len == 19 ? 0x4993 : len == 16 ? 0xEDDD : 0x3AAE;
}

// Setzt die beiden Pruefsummenbytes so, dass das Telegramm mit angehaengtem
// Bit 0 aufgeht. Rueckgabe false, falls keine Loesung gefunden wird - dann
// wird nicht gesendet, statt etwas Ungueltiges auf die Anlage loszulassen.
static bool pruefsummeSetzen(uint8_t *p, uint8_t len) {
  uint16_t vorne = CRC_INIT;
  vorne = crcByte(vorne, 0x69);
  vorne = crcByte(vorne, 0xC8);
  for (uint8_t i = 0; i + 2 < len; i++) vorne = crcByte(vorne, p[i]);
  const uint16_t ziel = konstante(len);
  for (uint16_t hi = 0; hi < 256; hi++) {
    uint16_t c1 = crcByte(vorne, (uint8_t)hi);
    for (uint16_t lo = 0; lo < 256; lo++) {
      if (crcBit(crcByte(c1, (uint8_t)lo), 0) == ziel) {
        p[len - 2] = (uint8_t)hi;
        p[len - 1] = (uint8_t)lo;
        return true;
      }
    }
  }
  return false;
}

// Prueft ein empfangenes Telegramm. Rueckgabe: Nutzlaenge, oder 0 wenn die
// Pruefsumme nicht passt bzw. die Adresse unbekannt ist. Unverzichtbar: ein
// einziger Bitfehler im Messwertfeld reicht, um ueber den Live-Patch den
// Zustand des Kamins zu vergiften - genau das ist am 2026-08-28 passiert.
static uint8_t pruefsummeOk(const uint8_t *buf) {
  uint8_t len;
  uint16_t ziel;
  if (buf[0] == 0xF7 && buf[1] == 0xB9) {
    len = 19;
    ziel = 0x4993;
  } else if (buf[0] == 0xF9 && buf[1] == 0x39) {
    len = 16;
    ziel = 0xEDDD;
  } else if (buf[0] == 0xFA && buf[1] == 0x39) {
    len = 14;
    ziel = 0x3AAE;
  } else
    return 0;
  uint16_t c = CRC_INIT;
  c = crcByte(c, 0x69);
  c = crcByte(c, 0xC8);
  for (uint8_t i = 0; i < len; i++) c = crcByte(c, buf[i]);
  c = crcBit(c, (buf[len] >> 7) & 1);
  return (c == ziel) ? len : 0;
}

// ---- Senden -----------------------------------------------------------------
// Der Handsender vergibt je Slot eine Sequenznummer (8c..8f, zyklisch) und
// wiederholt sie nie unmittelbar. Wir fuehren denselben Zaehler mit, damit
// unsere Telegramme nicht wie Wiederholungen aussehen.
uint8_t seqZaehler = 0x8C;

// PKTLEN gilt fuer Senden und Empfangen gleichermassen. Weil die Telegramme
// unterschiedlich lang sind, wird es vor jedem Senden gesetzt und danach wieder
// auf den Empfangswert zurueckgestellt. Gesendet wird len+1 Byte: das letzte
// traegt in Bit 7 das Pruefbit (hier 0), der Rest ist fuer den Empfaenger egal.
// Jedes Telegramm wird MEHRFACH gesendet, mit wechselndem Toggle-Bit - so macht
// es der Handsender auch. Der Grund ist handfest: ein einzeln gesendetes
// Telegramm geht gelegentlich verloren, und wir merken es nicht. Bei einer
// Stufe faellt das kaum auf (ein Treffer genuegt), beim Ausschalten dagegen
// muessen drei Befehle nacheinander ankommen - faellt einer aus, passiert gar
// nichts. Genau dieses Muster zeigte der Test vom 2026-08-28: 11 von 15 Stufen
// gingen, Ausschalten nie.
constexpr uint8_t WDH = 1;  // Sendungen je Telegramm
// 3x mit kippendem Toggle-Bit gesendet zu haben, half nachweislich nicht
// (Test 2026-08-28, identisches Ergebnis). Das Toggle-Bit markiert die
// Position innerhalb eines Sequenz-Slots, nicht eine Wiederholung -
// deshalb wieder genau ein Telegramm je Position, wie der Handsender.
constexpr uint32_t WDH_PAUSE = 150;  // ms zwischen den Wiederholungen

// Bytes [12..14] eines f7b9-Telegramms tragen den Messwert des Handsenders.
// Der Kamin spiegelt sie unveraendert in seine Statustelegramme - wir koennen
// den AKTUELLEN Wert also mitlesen, statt einen eingefrorenen aus einem alten
// Mitschnitt zu senden. Verdacht: sicherheitsrelevante Befehle werden nur
// angenommen, wenn der Absender mit dem Systemzustand synchron ist. Der Kamin
// quittiert unsere Ausschaltbefehle naemlich genau wie die des Handsenders,
// fuehrt sie aber nicht aus - und dieses Feld ist der einzige inhaltliche
// Unterschied, der bleibt.
// Letzter mitgehoerter Stellbefehl des Handsenders, wortgetreu inklusive
// Folgebyte. Damit laesst sich ohne jede Abschrift pruefen, ob unsere
// Sendekette taugt: dasselbe Telegramm, das eben gewirkt hat, unveraendert
// zurueckspielen.
uint8_t mitschnitt[20];
bool mitschnittOk = false;
uint8_t mitschnittStufe = 0;

uint8_t lebend[3];
bool lebendOk = false;
bool lebendPatch = true;  // mit 'x' umschaltbar, fuer A/B-Tests

// Rohtelegramm: Mitschnitt inklusive Folgebyte, wird voellig unveraendert
// gesendet - kein Sequenzzaehler, keine neu gerechnete Pruefsumme. Dient als
// Kontrollversuch: geht es damit, liegt der Fehler in dem, was wir aendern.
struct RohTelegramm {
  uint8_t len;
  uint8_t b[20];
};
// clang-format off
const RohTelegramm AUS_ROH[] = {
  {19,{0xF7,0xB9,0x8F,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0x69,0x08,0xBC,0xF6,0xAB,0xFA,0xAA,0x2F,0x57,0xF7}},
  {16,{0xF9,0x39,0x8F,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xE9,0xE3,0xAC,0xBA,0x73,0x78,0x02}},
  {16,{0xF9,0x39,0x8C,0xCC,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xED,0x67,0x46,0xB0,0x35,0xA6,0x2D}},
  {16,{0xF9,0x39,0x8C,0x4C,0x78,0x09,0x99,0xEF,0x3C,0x2D,0xEC,0x66,0x5F,0x04,0x14,0x34,0xCD}},
};
// clang-format on
constexpr uint8_t N_AUS_ROH = sizeof(AUS_ROH) / sizeof(AUS_ROH[0]);

static void sendFrame(const Telegramm &t, uint8_t seq, const char *name, uint8_t wdh = WDH) {
  for (uint8_t w = 0; w < wdh; w++) {
    uint8_t f[20];
    memcpy(f, t.b, t.len);
    f[2] = seq;
    if (w & 1) f[3] ^= 0x80;  // Toggle-Bit kippen wie der Handsender
    if (lebendPatch && lebendOk && f[0] == 0xF7) {
      f[12] = lebend[0];
      f[13] = lebend[1];
      f[14] = lebend[2];
    }
    if (!pruefsummeSetzen(f, t.len)) {
      Serial.printf("%s NICHT gesendet - keine gueltige Pruefsumme\n", name);
      return;
    }
    f[t.len] = 0x00;  // Pruefbit 0 + Fuellbits

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
    if (w + 1 < wdh) delay(WDH_PAUSE);
  }
  Serial.printf("%s gesendet (seq %02x, %u Byte + 1 Bit, %ux)\n", name, seq, t.len, wdh);
}

static void pollRx();  // weiter unten definiert

// Sendet ein Rohtelegramm voellig unveraendert, einschliesslich Folgebyte.
static void sendRoh(const RohTelegramm &t, const char *name) {
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
  Serial.printf("%s wortgetreu gesendet (%u Byte + 1 Bit)\n", name, t.len);
}

// Horcht waehrend der Pause weiter mit - so werden die Antworttelegramme des
// Kamins (Adresse fa39) sichtbar, die bisher in den Sendepausen verlorengingen.
static void lauschen(uint32_t ms) {
  strobe(SIDLE);
  strobe(SFRX);
  strobe(SRX);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) pollRx();
  strobe(SIDLE);
}

static void sendSeq(const Telegramm *seq, uint8_t n, const char *name) {
  const uint8_t basis = seqZaehler;
  uint8_t maxSlot = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (seq[i].slot > maxSlot) maxSlot = seq[i].slot;
    uint8_t s = 0x8C | ((basis + seq[i].slot) & 0x03);
    char txt[28];
    snprintf(txt, sizeof(txt), "%s %u/%u", name, i + 1, n);
    sendFrame(seq[i], s, txt);
    if (i + 1 < n) lauschen(300);
  }
  seqZaehler = 0x8C | ((basis + maxSlot + 1) & 0x03);
}

// Ein 16-Bit-Sync trifft im Rauschen rein zufaellig etwa alle 2^16 Bit zu,
// bei 38,4 kbit/s also ca. jede 1,7 s. Diese Fehltrigger liegen am
// Empfindlichkeitslimit (~-105 dBm), echte Pakete deutlich darueber.
static void pollRx() {
  uint8_t n = readStatus(RXBYTES) & 0x7F;
  if (n < 20) return;
  uint8_t buf[20];
  burstRead(RXFIFO, buf, 20);
  int rssi = ((int8_t)readStatus(RSSI) / 2) - 74;
  strobe(SIDLE);
  strobe(SFRX);
  strobe(SRX);  // fuer das naechste Paket

  if (rssi < RSSI_MIN) {
    rxNoise++;
    return;
  }

  // Nur pruefsummenreine Telegramme weiterverarbeiten.
  bool bekannt = (buf[0] == 0xF7 && buf[1] == 0xB9) || (buf[0] == 0xF9 && buf[1] == 0x39) ||
                 (buf[0] == 0xFA && buf[1] == 0x39);
  if (bekannt && !pruefsummeOk(buf)) {
    rxKaputt++;
    return;
  }

  Serial.printf("RX %4d dBm  69c8", rssi);  // Sync steht nicht im FIFO
  for (uint8_t i = 0; i < 20; i++) Serial.printf("%02x", buf[i]);
  if (rxNoise) Serial.printf("   (%lu Fehltrigger", rxNoise);
  if (rxKaputt) Serial.printf("%s%lu mit Bitfehlern", rxNoise ? ", " : "   (", rxKaputt);
  if (rxNoise || rxKaputt) Serial.print(" verworfen)");

  // Statustelegramm des Kamins: Byte 10/11 (ohne Sync) sind ein BITFELD, kein
  // Zustandscode. Drei Bits wurden beobachtet, sie kommen beim Anfeuern in
  // dieser Reihenfolge. Die Deutung folgt der Modbus-Registerbelegung eines
  // baugleichen Mertik-Geraets (Statusregister 40203: Zuendflamme, Hauptbrenner,
  // Wave aktiv) - siehe forum.domoticz.com/viewtopic.php?t=29060.
  if (buf[0] == 0xF7 && buf[4] == 0x78 && buf[15] != 0xFA) {
    memcpy(mitschnitt, buf, 20);
    mitschnittOk = true;
    mitschnittStufe = buf[15];
    Serial.printf("   << gemerkt: Handsender-Stellbefehl, Stellwert %02x", buf[15]);
  }

  if (buf[0] == 0xF7 && buf[4] == 0x0B) {
    uint16_t st = ((uint16_t)buf[10] << 8) | buf[11];
    uint16_t d = st ^ 0x691A;  // 0x691a = alles aus
    Serial.print("   [");
    if (!d)
      Serial.print("aus");
    else {
      if (d & 0x0040) Serial.print("Zuendflamme ");
      if (d & 0x0200) Serial.print("Hauptbrenner ");
      if (d & 0x0002) Serial.print("Wave ");
      if (d & ~0x0242) Serial.printf("unbekannt:%04x ", d & ~0x0242);
    }
    Serial.print("]");

    // Aenderungsmelder: zeigt jede Bewegung gegenueber dem vorigen
    // Statustelegramm. Sequenz, Toggle und Pruefsumme aendern sich ohnehin
    // bei jedem Paket und wuerden die Ausgabe nur zumuellen.
    // [12] nimmt in ueber 800 Mitschnitt-Frames ausschliesslich bd oder bc an.
    // Alles andere ist verdorben - dann lieber den letzten guten Wert behalten
    // und weitersenden, damit der Kamin sich wieder darauf einschwingt.
    if (buf[12] == 0xBD || buf[12] == 0xBC) {
      lebend[0] = buf[12];
      lebend[1] = buf[13];
      lebend[2] = buf[14];
      lebendOk = true;
    } else if (lebendOk) {
      Serial.printf("   !! Messwert [12]=%02x unplausibel, halte %02x%02x%02x", buf[12], lebend[0],
                    lebend[1], lebend[2]);
    }

    static uint8_t vorher[19];
    static bool haveVorher = false;
    if (haveVorher) {
      bool erste = true;
      for (uint8_t i = 0; i < 19; i++) {
        if (i == 2 || i == 3 || i == 17 || i == 18) continue;
        if (buf[i] == vorher[i]) continue;
        if (erste) {
          Serial.print("   AENDERUNG:");
          erste = false;
        }
        Serial.printf(" [%u] %02x->%02x", i, vorher[i], buf[i]);
      }
    }
    memcpy(vorher, buf, 19);
    haveVorher = true;
  }
  Serial.println();
  rxNoise = 0;
  rxKaputt = 0;
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
    Serial.println("Keine Antwort - Verdrahtung pruefen (VCC an 3V3, nicht 5V!).");
    return;
  }

  crcTabInit();
  configure();
  Serial.println("868,3 MHz | 38,4 kbit/s | +-19 kHz | Sync 0x69C8");
  Serial.printf(
      "r=Empfangen  1=AN  2=AUS  3=AUS wortgetreu  a..%c=Stufe 1..%d\n"
      "t=Stufentest  v=Mitschnitt wiedergeben  x=Live-Messwert  ?=Register\n",
      'a' + N_STUFEN - 1, N_STUFEN);
  startRx();
}

// Stufentest im Wechsel: 15-1-14-1-13-1... Jede Stufe wird gegen Stufe 1
// gestellt statt gegen ihren Nachbarn. Der Sprung ist dadurch gross genug,
// um mit blossem Auge zu erkennen, ob eine Stufe ueberhaupt greift - bei
// 15-14-13 ist der Unterschied zu klein und die Stellzeit verwischt ihn.
// Ein Ventilweg von Stufe 1 bis 15 dauert bis zu 5 s, daher die Pause.
constexpr uint32_t STELLZEIT = 7000;

static bool abgebrochen() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  Serial.println("--- abgebrochen ---");
  return true;
}

static void stufeStellen(uint8_t stufe) {
  char name[16];
  snprintf(name, sizeof(name), "Stufe %u", stufe);
  sendFrame(STUFEN[stufe - 1], seqZaehler, name);
  seqZaehler = 0x8C | ((seqZaehler + 1) & 0x03);
}

static void stufentest() {
  Serial.printf(
      "\n=== Stufentest 15-1-14-1-... , %lu ms je Schritt ===\n"
      "Beobachten: springt die Flamme bei JEDER Stufe sichtbar hoch?\n"
      "Beliebige Taste bricht ab.\n",
      STELLZEIT);
  for (uint8_t s = N_STUFEN; s >= 1; s--) {
    Serial.printf("\n-- Stufe %u --\n", s);
    stufeStellen(s);
    lauschen(STELLZEIT);
    if (abgebrochen()) return;
    if (s == 1) break;  // Stufe 1 ist schon der Ruhepunkt
    Serial.println("-- zurueck auf 1 --");
    stufeStellen(1);
    lauschen(STELLZEIT);
    if (abgebrochen()) return;
  }
  Serial.println("\n=== Stufentest beendet ===");
}

void loop() {
  if (Serial.available()) {
    char k = Serial.read();
    switch (k) {
      case 'r':
        startRx();
        break;
      case '1':
        sendSeq(SEQ_AN, N_SEQ_AN, "AN");
        startRx();
        break;
      case '2':
        sendSeq(SEQ_AUS, N_SEQ_AUS, "AUS");
        startRx();
        break;
      case 't':
        stufentest();
        startRx();
        break;
      case '3':  // Kontrollversuch: unveraenderter Mitschnitt
        Serial.println("\n--- AUS wortgetreu (Handsender-Mitschnitt 28.08.) ---");
        for (uint8_t i = 0; i < N_AUS_ROH; i++) {
          char s2[24];
          snprintf(s2, sizeof(s2), "AUS-roh %u/%u", i + 1, N_AUS_ROH);
          sendRoh(AUS_ROH[i], s2);
          if (i + 1 < N_AUS_ROH) delay(120);
        }
        startRx();
        break;
      case 'v':
        if (!mitschnittOk) {
          Serial.println("noch nichts gemerkt");
          break;
        }
        Serial.printf(
            "\n--- Wiedergabe des gemerkten Handsender-Telegramms "
            "(Stellwert %02x) ---\n",
            mitschnittStufe);
        {
          RohTelegramm r;
          r.len = 19;
          memcpy(r.b, mitschnitt, 20);
          sendRoh(r, "Mitschnitt");
        }
        startRx();
        break;
      case 'x':
        lebendPatch = !lebendPatch;
        Serial.printf("Live-Messwert %s\n", lebendPatch ? "EIN" : "AUS");
        break;
      default:
        if (k >= 'a' && k < char('a' + N_STUFEN)) {  // a=Stufe 1 ... o=Stufe 15
          char name[16];
          snprintf(name, sizeof(name), "Stufe %d", k - 'a' + 1);
          sendFrame(STUFEN[k - 'a'], seqZaehler, name);
          seqZaehler = 0x8C | ((seqZaehler + 1) & 0x03);
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
