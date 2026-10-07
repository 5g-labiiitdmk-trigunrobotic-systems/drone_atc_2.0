/*
  PIXHAWK 2.4.8 <-> ESP8266 CONNECTION CHECK  (simple, no libraries to install)

  Wiring (3.3 V logic):
    Pixhawk TELEM TX  -> NodeMCU D7
    Pixhawk TELEM RX  -> NodeMCU D8
    Pixhawk GND       -> NodeMCU GND
    (power the NodeMCU from USB; do NOT use the Pixhawk 5V pin)

  Pixhawk (ArduPilot) settings for the TELEM port you used (TELEM1 = SERIAL1, TELEM2 = SERIAL2):
    SERIALn_PROTOCOL = 2   and   SERIALn_BAUD = 57   (57600), then reboot the Pixhawk.

  Open Tools > Serial Monitor at 115200 baud. It prints ALL OK only if data flows both ways.
*/
#include <SoftwareSerial.h>

#define BAUD 57600
SoftwareSerial fc(13, 15);                 // RX = D7 (GPIO13), TX = D8 (GPIO15)

uint32_t rxBytes = 0, goodFrames = 0, heartbeats = 0, replies = 0, rounds = 0, echo = 0;
unsigned long lastRound = 0, lastReport = 0;
uint8_t seqTx = 0;

// CRC_EXTRA of the MAVLink messages an ArduPilot sends (needed to check each frame)
static int crcExtra(uint32_t id) {
  switch (id) {
    case 0: return 50;    case 1: return 124;   case 2: return 137;   case 20: return 214;
    case 22: return 220;  case 24: return 24;   case 26: return 170;  case 27: return 144;
    case 28: return 67;   case 29: return 115;  case 30: return 39;   case 31: return 246;
    case 32: return 185;  case 33: return 104;  case 35: return 244;  case 36: return 222;
    case 42: return 28;   case 46: return 11;   case 62: return 183;  case 65: return 118;
    case 74: return 20;   case 76: return 152;  case 77: return 143;  case 111: return 34;
    case 116: return 76;  case 125: return 203; case 129: return 46;  case 147: return 154;
    case 148: return 178; case 152: return 208; case 163: return 127; case 165: return 21;
    case 178: return 47;  case 241: return 90;  case 242: return 104; case 253: return 83;
    default: return -1;
  }
}
static void crcAdd(uint8_t b, uint16_t &crc) {
  uint8_t t = b ^ (uint8_t)(crc & 0xFF);
  t ^= (t << 4);
  crc = (crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4);
}

// ---- ESP -> Pixhawk: ask for the parameter SYSID_THISMAV; the Pixhawk MUST answer ----
static void sendRequest(bool mavlink1) {
  uint8_t pl[20] = {0};
  pl[0] = 0xFF; pl[1] = 0xFF;                       // param index -1 = "find by name"
  pl[2] = 1; pl[3] = 1;                             // Pixhawk system 1, component 1
  memcpy(pl + 4, "SYSID_THISMAV", 13);
  uint8_t f[40]; size_t n = 0;
  uint16_t crc = 0xFFFF;
  if (mavlink1) { f[0] = 0xFE; f[1] = 20; f[2] = seqTx++; f[3] = 255; f[4] = 200; f[5] = 20; n = 6; }
  else          { f[0] = 0xFD; f[1] = 20; f[2] = 0; f[3] = 0; f[4] = seqTx++; f[5] = 255; f[6] = 200; f[7] = 20; f[8] = 0; f[9] = 0; n = 10; }
  memcpy(f + n, pl, 20); n += 20;
  for (size_t i = 1; i < n; i++) crcAdd(f[i], crc);
  crcAdd(214, crc);                                 // CRC_EXTRA of PARAM_REQUEST_READ
  f[n++] = crc & 0xFF; f[n++] = crc >> 8;
  fc.write(f, n);
}

// ---- Pixhawk -> ESP: find valid MAVLink frames in the byte stream ----
static uint8_t buf[300]; static size_t len = 0;
static void dropOne() { memmove(buf, buf + 1, --len); }
static void feed(uint8_t b) {
  if (len >= sizeof buf) dropOne();
  buf[len++] = b;
  while (len > 0) {
    if (buf[0] != 0xFD && buf[0] != 0xFE) { dropOne(); continue; }
    bool v2 = (buf[0] == 0xFD);
    if (len < (v2 ? 10u : 6u)) return;
    uint32_t id = v2 ? (buf[7] | (buf[8] << 8) | ((uint32_t)buf[9] << 16)) : buf[5];
    int ex = crcExtra(id);
    if (ex < 0 || (v2 && (buf[2] & 0xFE))) { dropOne(); continue; }
    size_t crcPos = (v2 ? 10u : 6u) + buf[1];
    size_t need = crcPos + 2 + ((v2 && (buf[2] & 1)) ? 13 : 0);
    if (len < need) return;
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < crcPos; i++) crcAdd(buf[i], crc);
    crcAdd((uint8_t)ex, crc);
    if (buf[crcPos] == (crc & 0xFF) && buf[crcPos + 1] == (crc >> 8)) {
      uint8_t sys = v2 ? buf[5] : buf[3], comp = v2 ? buf[6] : buf[4];
      const uint8_t *p = buf + (v2 ? 10 : 6);
      if (sys == 255 && comp == 200) echo++;                    // our own frame came back (TX wired to RX)
      else {
        goodFrames++;
        if (id == 0 && p[5] != 8 && p[4] != 6) heartbeats++;    // heartbeat of a real autopilot
        if (id == 22 && !strncmp((const char *)p + 8, "SYSID_THISMAV", 13)) replies++;   // answer to our request
      }
      memmove(buf, buf + need, len - need); len -= need;
    } else dropOne();
  }
}

void setup() {
  Serial.begin(115200);
  fc.begin(BAUD);
  Serial.println("\nPixhawk <-> ESP8266 connection check started...");
}

void loop() {
  while (fc.available()) { feed(fc.read()); rxBytes++; }

  if (millis() - lastRound >= 1500) {               // ask the Pixhawk something (alternate MAVLink 1 / 2)
    lastRound = millis();
    sendRequest(rounds & 1);
    rounds++;
  }

  if (millis() - lastReport >= 3000) {              // report
    lastReport = millis();
    bool rxOk = heartbeats > 0;                      // Pixhawk -> ESP
    bool txOk = replies > 0;                         // ESP -> Pixhawk (the Pixhawk answered us)
    Serial.println("\n===== Pixhawk <-> ESP8266 =====");
    Serial.printf("Pixhawk -> ESP : %s   (bytes %lu, heartbeats %lu)\n", rxOk ? "OK" : "NOT OK", (unsigned long)rxBytes, (unsigned long)heartbeats);
    Serial.printf("ESP -> Pixhawk : %s   (requests %lu, answers %lu)\n", txOk ? "OK" : "NOT OK", (unsigned long)rounds, (unsigned long)replies);
    if (rxOk && txOk)        Serial.println(">>> ALL OK - connection is good in BOTH directions");
    else if (rxOk && rounds >= 4) Serial.println(">>> PROBLEM: Pixhawk -> ESP works but ESP -> Pixhawk does NOT. Check the wire D8 -> Pixhawk RX, GND, and SERIALn_PROTOCOL = 2.");
    else if (rxOk)           Serial.println(">>> Pixhawk -> ESP works, still testing ESP -> Pixhawk...");
    else if (echo > 0)       Serial.println(">>> PROBLEM: the ESP is hearing itself (D7 and D8 are connected together). Connect the Pixhawk instead.");
    else if (rxBytes > 0)    Serial.println(">>> PROBLEM: data arrives but is not valid MAVLink. Check SERIALn_BAUD = 57 (57600), SERIALn_PROTOCOL = 2 and the GND wire.");
    else                     Serial.println(">>> PROBLEM: NO data from the Pixhawk. Check Pixhawk TX -> D7, GND, that the Pixhawk is powered, and which TELEM port you used.");
  }
}
