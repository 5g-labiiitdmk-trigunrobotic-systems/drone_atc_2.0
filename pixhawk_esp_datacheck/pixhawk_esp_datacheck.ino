/**
 * PIXHAWK <-> ESP8266 DATA CHECK  (standalone diagnostic, NO libraries needed)
 * ===========================================================================
 * Tells you, separately, whether
 *     Pixhawk -> ESP   (bytes / valid MAVLink frames arrive)   and
 *     ESP -> Pixhawk   (the Pixhawk ANSWERS what the ESP sends)
 * is working. It contains its own tiny MAVLink parser/encoder, so it does not
 * need the MAVLink headers (nothing to copy, nothing to install).
 *
 * HOW TO USE
 *   1. Board: NodeMCU 1.0 (ESP8266). Flash this sketch.
 *   2. Wire:  Pixhawk TELEM TX -> D7 (GPIO13)   Pixhawk TELEM RX -> D8 (GPIO15)
 *             Pixhawk GND -> NodeMCU GND.  Power the NodeMCU separately (USB).
 *   3. Join the WiFi "PixhawkCheck" (password 12345678) on a phone/PC.
 *   4. Open  http://192.168.4.1   and read the two big lights.
 *
 * Pixhawk (ArduPilot) must have, for the TELEM port you wired (n = 1 or 2):
 *   SERIALn_PROTOCOL = 2   and   SERIALn_BAUD = 57   (57600)   then reboot.
 * The page auto-scans other baud rates if bytes arrive but nothing decodes.
 *
 * ECHO TEST (checks the ESP alone): unplug the Pixhawk, jumper D7 to D8.
 * The ESP then hears its own frames -> "ECHO OK" = both ESP pins work.
 */
#if !defined(ESP8266)
#error "Select an ESP8266 board (NodeMCU)."
#endif
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define USE_SWAPPED_UART 1               // 1: D7/D8.  0: the RX/TX pins (shared with USB)
#define TEST_PERIOD_MS   2000

static const uint32_t BAUDS[] = { 57600, 115200, 921600, 38400, 19200, 9600, 230400, 460800 };
static const uint8_t  NBAUDS  = sizeof(BAUDS) / sizeof(BAUDS[0]);

ESP8266WebServer web(80);

// ---------------------------------------------------------------- counters
uint32_t rxBytes = 0, txBytes = 0;
uint32_t validFrames = 0, badFrames = 0, hbCount = 0, ownEcho = 0;
uint32_t paramReplies = 0, acks = 0, txRounds = 0;
unsigned long lastRxMs = 0, lastTestMs = 0, baudSetMs = 0, lastHbMs = 0;
uint8_t  fcSys = 0, fcComp = 0, fcType = 0, fcAutopilot = 0, fcBase = 0;
uint32_t fcCustom = 0;
float    sysidValue = -1;
uint8_t  baudIdx = 0, ackResult = 255;
bool     autoScan = true, autoTest = true, baudLocked = false;
uint32_t bytesAtBaudStart = 0;
uint8_t  tail[24]; uint8_t tailN = 0;                    // last raw bytes
struct Seen { uint16_t id; uint32_t n; } seen[24]; uint8_t seenN = 0;
char     lastText[56] = "";
uint8_t  seqTx = 0;

// ---------------------------------------------------------------- MAVLink bits (no library)
static uint8_t crcExtra(uint32_t id, bool* known) {
  *known = true;
  switch (id) {
    case 0:   return 50;   // HEARTBEAT
    case 1:   return 124;  // SYS_STATUS
    case 20:  return 214;  // PARAM_REQUEST_READ
    case 22:  return 220;  // PARAM_VALUE
    case 24:  return 24;   // GPS_RAW_INT
    case 30:  return 39;   // ATTITUDE
    case 33:  return 104;  // GLOBAL_POSITION_INT
    case 35:  return 244;  // RC_CHANNELS_RAW
    case 36:  return 222;  // SERVO_OUTPUT_RAW
    case 42:  return 28;   // MISSION_CURRENT
    case 62:  return 183;  // NAV_CONTROLLER_OUTPUT
    case 65:  return 118;  // RC_CHANNELS
    case 74:  return 20;   // VFR_HUD
    case 76:  return 152;  // COMMAND_LONG
    case 77:  return 143;  // COMMAND_ACK
    case 125: return 203;  // POWER_STATUS
    case 253: return 83;   // STATUSTEXT
    case 2:   return 137;  // SYSTEM_TIME
    case 26:  return 170;  // SCALED_IMU
    case 27:  return 144;  // RAW_IMU
    case 28:  return 67;   // RAW_PRESSURE
    case 29:  return 115;  // SCALED_PRESSURE
    case 31:  return 246;  // ATTITUDE_QUATERNION
    case 32:  return 185;  // LOCAL_POSITION_NED
    case 46:  return 11;   // MISSION_ITEM_REACHED
    case 111: return 34;   // TIMESYNC
    case 116: return 76;   // SCALED_IMU2
    case 129: return 46;   // SCALED_IMU3
    case 147: return 154;  // BATTERY_STATUS
    case 148: return 178;  // AUTOPILOT_VERSION
    case 152: return 208;  // MEMINFO
    case 163: return 127;  // AHRS
    case 165: return 21;   // HWSTATUS
    case 178: return 47;   // AHRS2
    case 241: return 90;   // VIBRATION
    case 242: return 104;  // HOME_POSITION
    default:  *known = false; return 0;
  }
}
static void crcAcc(uint8_t b, uint16_t& crc) {
  uint8_t t = b ^ (uint8_t)(crc & 0xFF);
  t ^= (t << 4);
  crc = (crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4);
}

// MAVLink v2 frame (no signing, full payload)
static size_t buildV2(uint8_t* out, uint32_t msgid, const uint8_t* pl, uint8_t len) {
  bool k; uint8_t ex = crcExtra(msgid, &k);
  out[0] = 0xFD; out[1] = len; out[2] = 0; out[3] = 0; out[4] = seqTx++; out[5] = 255; out[6] = 200;
  out[7] = msgid & 0xFF; out[8] = (msgid >> 8) & 0xFF; out[9] = (msgid >> 16) & 0xFF;
  memcpy(out + 10, pl, len);
  uint16_t crc = 0xFFFF;
  for (size_t i = 1; i < 10u + len; i++) crcAcc(out[i], crc);
  crcAcc(ex, crc);
  out[10 + len] = crc & 0xFF; out[11 + len] = crc >> 8;
  return 12u + len;
}
// MAVLink v1 frame
static size_t buildV1(uint8_t* out, uint8_t msgid, const uint8_t* pl, uint8_t len) {
  bool k; uint8_t ex = crcExtra(msgid, &k);
  out[0] = 0xFE; out[1] = len; out[2] = seqTx++; out[3] = 255; out[4] = 200; out[5] = msgid;
  memcpy(out + 6, pl, len);
  uint16_t crc = 0xFFFF;
  for (size_t i = 1; i < 6u + len; i++) crcAcc(out[i], crc);
  crcAcc(ex, crc);
  out[6 + len] = crc & 0xFF; out[7 + len] = crc >> 8;
  return 8u + len;
}

static void txFrame(const uint8_t* f, size_t n) { Serial.write(f, n); txBytes += n; }

static void sendTestRound() {
  uint8_t f[64], pl[40];
  const bool v1 = (txRounds & 1);                   // alternate MAVLink1 / MAVLink2
  // PARAM_REQUEST_READ: param_index(int16) | target_system | target_component | param_id[16]
  memset(pl, 0, sizeof pl);
  pl[0] = 0xFF; pl[1] = 0xFF;                       // index -1 => look up by name
  pl[2] = 1; pl[3] = 1;                             // target sys 1 comp 1 (Pixhawk default)
  memcpy(pl + 4, "SYSID_THISMAV", 13);
  txFrame(f, v1 ? buildV1(f, 20, pl, 20) : buildV2(f, 20, pl, 20));
  // COMMAND_LONG REQUEST_MESSAGE(512) of AUTOPILOT_VERSION(148): 7 floats | command | tsys | tcomp | confirm
  memset(pl, 0, sizeof pl);
  float p1 = 148.0f; memcpy(pl, &p1, 4);
  pl[28] = 512 & 0xFF; pl[29] = 512 >> 8; pl[30] = 1; pl[31] = 1; pl[32] = 0;
  txFrame(f, v1 ? buildV1(f, 76, pl, 33) : buildV2(f, 76, pl, 33));
  // our own heartbeat (what the echo test looks for)
  memset(pl, 0, sizeof pl);
  pl[4] = 18; pl[5] = 8; pl[7] = 4; pl[8] = 3;      // type ONBOARD_CONTROLLER, autopilot INVALID, state ACTIVE, mavlink 3
  txFrame(f, v1 ? buildV1(f, 0, pl, 9) : buildV2(f, 0, pl, 9));
  txRounds++;
}

static void noteSeen(uint16_t id) {
  for (uint8_t i = 0; i < seenN; i++) if (seen[i].id == id) { seen[i].n++; return; }
  if (seenN < 24) { seen[seenN].id = id; seen[seenN].n = 1; seenN++; }
}

static void handleFrame(uint8_t sys, uint8_t comp, uint32_t msgid, const uint8_t* payload, uint8_t len) {
  uint8_t p[64]; memset(p, 0, sizeof p); memcpy(p, payload, len < 64 ? len : 64);   // v2 trims trailing zeros
  validFrames++;
  if (sys == 255 && comp == 200) { ownEcho++; return; }                          // our own frame came back
  noteSeen(msgid);
  if (msgid == 0) {                                                               // HEARTBEAT
    uint8_t type = p[4], ap = p[5];
    if (ap != 8 && type != 6) {                                                   // a real autopilot
      fcSys = sys; fcComp = comp; fcType = type; fcAutopilot = ap; fcBase = p[6];
      fcCustom = p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
      hbCount++; lastHbMs = millis();
    }
  } else if (msgid == 22) {                                                       // PARAM_VALUE
    char name[17]; memcpy(name, p + 8, 16); name[16] = 0;
    if (!strncmp(name, "SYSID_THISMAV", 13)) { float v; memcpy(&v, p, 4); sysidValue = v; paramReplies++; }
  } else if (msgid == 77) {                                                       // COMMAND_ACK
    uint16_t cmd = p[0] | (p[1] << 8);
    if (cmd == 512) { acks++; ackResult = p[2]; }
  } else if (msgid == 253) {                                                      // STATUSTEXT
    strncpy(lastText, (const char*)p + 1, 50); lastText[50] = 0;
    for (char* c = lastText; *c; c++) if (*c == '"' || *c == '\\' || (unsigned char)*c < 32) *c = ' ';
  }
}

// byte-stream framer with CRC check
static uint8_t fb[320]; static size_t fl = 0;
static void dropOne() { memmove(fb, fb + 1, --fl); }
static void framerPush(uint8_t b) {
  if (fl >= sizeof fb) dropOne();
  fb[fl++] = b;
  while (fl > 0) {
    if (fb[0] != 0xFD && fb[0] != 0xFE) { dropOne(); continue; }
    const bool v2 = fb[0] == 0xFD;
    if (fl < (v2 ? 10u : 6u)) return;
    // Reject junk as early as the header allows, so a stray 0xFD/0xFE can't make us
    // wait for a bogus long frame and swallow the real frames behind it.
    {
      const uint32_t mid = v2 ? (fb[7] | (fb[8] << 8) | ((uint32_t)fb[9] << 16)) : fb[5];
      bool k; crcExtra(mid, &k);
      if (!k || (v2 && (fb[2] & 0xFE))) { badFrames++; dropOne(); continue; }
    }
    const uint8_t len = fb[1];
    const size_t crcPos = (v2 ? 10u : 6u) + len;
    const size_t need = crcPos + 2 + ((v2 && (fb[2] & 1)) ? 13 : 0);
    if (fl < need) return;
    const uint32_t msgid = v2 ? (fb[7] | (fb[8] << 8) | ((uint32_t)fb[9] << 16)) : fb[5];
    bool known; const uint8_t ex = crcExtra(msgid, &known);
    bool ok = false;
    if (known) {
      uint16_t crc = 0xFFFF;
      for (size_t i = 1; i < crcPos; i++) crcAcc(fb[i], crc);
      crcAcc(ex, crc);
      ok = (fb[crcPos] == (crc & 0xFF)) && (fb[crcPos + 1] == (crc >> 8));
    }
    if (ok) {
      handleFrame(v2 ? fb[5] : fb[3], v2 ? fb[6] : fb[4], msgid, fb + (v2 ? 10 : 6), len);
      memmove(fb, fb + need, fl - need); fl -= need;
    } else {                                    // false start or unknown message: resync
      badFrames++; dropOne();
    }
  }
}

static void pumpSerial() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    rxBytes++; lastRxMs = millis();
    if (tailN == sizeof tail) { memmove(tail, tail + 1, tailN - 1); tailN--; }
    tail[tailN++] = b;
    framerPush(b);
  }
}

// ---------------------------------------------------------------- baud auto-scan
static void setBaud(uint8_t idx) {
  baudIdx = idx % NBAUDS;
  Serial.updateBaudRate(BAUDS[baudIdx]);          // keeps the swapped pins
  baudSetMs = millis(); bytesAtBaudStart = rxBytes; fl = 0;
}
static void scanTick() {
  if (!autoScan || baudLocked) return;
  if (validFrames - ownEcho > 0) { baudLocked = true; return; }       // found it
  if (rxBytes - bytesAtBaudStart > 40 && millis() - baudSetMs > 5000) setBaud(baudIdx + 1);
}

// ---------------------------------------------------------------- verdict
static const char* rxVerdict() {
  if (hbCount > 0 && millis() - lastHbMs < 5000) return "OK";
  if (validFrames - ownEcho > 0) return "FRAMES";
  if (rxBytes > 0 && millis() - lastRxMs < 5000) return "GARBAGE";
  return "NONE";
}
static const char* txVerdict() {
  if (paramReplies + acks > 0) return "OK";
  if (txRounds >= 4) return "NO REPLY";
  return "TESTING";
}
static const char* summary() {
  const bool rxOk = !strcmp(rxVerdict(), "OK") || !strcmp(rxVerdict(), "FRAMES");
  const bool txOk = !strcmp(txVerdict(), "OK");
  if (txOk)  return "BOTH WAYS WORK: the Pixhawk hears the ESP, and the ESP hears the Pixhawk.";
  if (rxOk && txVerdict()[0] == 'N') return "ONE WAY ONLY: Pixhawk -> ESP works, ESP -> Pixhawk does NOT. Check the D8 -> Pixhawk RX wire (TX/RX crossed? loose?), common GND, and the Pixhawk port protocol (2) and baud (57). Then try the echo test.";
  if (rxOk)  return "Pixhawk -> ESP works. Testing ESP -> Pixhawk...";
  if (ownEcho > 0) return "ECHO OK: the ESP's TX and RX pins both work. Remove the jumper and connect the Pixhawk.";
  if (!strcmp(rxVerdict(), "GARBAGE")) return "Bytes arrive but nothing decodes: wrong baud (set SERIALn_BAUD=57; the page is scanning baud rates) or wrong protocol (SERIALn_PROTOCOL=2) or a loose GND.";
  return "NO DATA from the Pixhawk: check Pixhawk TX -> D7, GND, that the Pixhawk is powered, and that this is the right TELEM port. To test the ESP alone: jumper D7 to D8.";
}

// ---------------------------------------------------------------- web
static void handleStatus() {
  const unsigned long now = millis();
  char hex[24 * 3 + 1]; hex[0] = 0;
  for (uint8_t i = 0; i < tailN; i++) { char t[4]; snprintf(t, sizeof t, "%02X ", tail[i]); strcat(hex, t); }
  char ids[24 * 12 + 4]; ids[0] = 0;
  for (uint8_t i = 0; i < seenN; i++) { char t[16]; snprintf(t, sizeof t, "%s%u:%lu", i ? " " : "", seen[i].id, (unsigned long)seen[i].n); strcat(ids, t); }
  char b[1700];
  snprintf(b, sizeof b,
    "{\"summary\":\"%s\",\"rx\":\"%s\",\"tx\":\"%s\",\"baud\":%lu,\"locked\":%s,"
    "\"rxBytes\":%lu,\"txBytes\":%lu,\"valid\":%lu,\"bad\":%lu,\"hb\":%lu,\"echo\":%lu,"
    "\"params\":%lu,\"acks\":%lu,\"rounds\":%lu,\"sysid\":%.0f,\"ackResult\":%u,"
    "\"fcSys\":%u,\"fcComp\":%u,\"fcType\":%u,\"fcAp\":%u,\"armed\":%s,\"mode\":%lu,"
    "\"msRx\":%ld,\"hex\":\"%s\",\"ids\":\"%s\",\"text\":\"%s\",\"scan\":%s}",
    summary(), rxVerdict(), txVerdict(), (unsigned long)BAUDS[baudIdx], baudLocked ? "true" : "false",
    (unsigned long)rxBytes, (unsigned long)txBytes, (unsigned long)validFrames, (unsigned long)badFrames, (unsigned long)hbCount, (unsigned long)ownEcho,
    (unsigned long)paramReplies, (unsigned long)acks, (unsigned long)txRounds, sysidValue, ackResult,
    fcSys, fcComp, fcType, fcAutopilot, (fcBase & 0x80) ? "true" : "false", (unsigned long)fcCustom,
    lastRxMs ? (long)(now - lastRxMs) : -1L, hex, ids, lastText, autoScan ? "true" : "false");
  web.send(200, "application/json", b);
}
static void handleSend()  { sendTestRound(); web.send(200, "text/plain", "sent"); }
static void handleReset() {
  rxBytes = txBytes = validFrames = badFrames = hbCount = ownEcho = paramReplies = acks = txRounds = 0;
  sysidValue = -1; ackResult = 255; lastText[0] = 0; tailN = 0; seenN = 0; fl = 0; lastRxMs = 0; lastHbMs = 0;
  baudLocked = false; bytesAtBaudStart = 0; baudSetMs = millis();
  web.send(200, "text/plain", "reset");
}
static void handleScan() { autoScan = !autoScan; baudLocked = false; web.send(200, "text/plain", autoScan ? "scan on" : "scan off"); }
static void handleBaud() {
  uint32_t want = web.arg("b").toInt();
  for (uint8_t i = 0; i < NBAUDS; i++) if (BAUDS[i] == want) { autoScan = false; baudLocked = false; setBaud(i); }
  web.send(200, "text/plain", "ok");
}

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>Pixhawk data check</title><style>
body{font-family:system-ui,sans-serif;margin:0;background:#0f1320;color:#e8ecf4}main{max-width:680px;margin:0 auto;padding:14px}
.lights{display:flex;gap:12px}.light{flex:1;border-radius:14px;padding:16px;text-align:center;background:#2a2f45}
.light h2{margin:0;font-size:13px;letter-spacing:.08em;color:#c7d0ea}.light .s{font-size:30px;font-weight:800;margin:8px 0}
.good{background:#14502c}.bad{background:#5a1d1d}.wait{background:#4d4115}
#sum{margin:12px 0;padding:14px;border-radius:12px;background:#171c2e;font-size:16px;line-height:1.4}
.card{background:#171c2e;border-radius:12px;padding:12px;margin-top:10px}.card h3{margin:0 0 6px;font-size:12px;letter-spacing:.08em;color:#9aa6c4}
.dim{color:#9aa6c4;font-size:13px;line-height:1.7;word-break:break-all}code{color:#d7e3ff}
button{background:#2b3358;color:#fff;border:0;border-radius:10px;padding:11px 13px;font-size:14px;margin:10px 8px 0 0}
</style></head><body><main>
<div class=lights>
 <div class="light wait" id=lr><h2>PIXHAWK &rarr; ESP</h2><div class=s id=rs>...</div><div class=dim id=rd></div></div>
 <div class="light wait" id=lt><h2>ESP &rarr; PIXHAWK</h2><div class=s id=ts>...</div><div class=dim id=td></div></div>
</div>
<div id=sum>connecting...</div>
<div class=card><h3>PIXHAWK</h3><div class=dim id=fc>-</div></div>
<div class=card><h3>RAW DATA (last bytes received)</h3><div class=dim><code id=hex>-</code><br>message ids heard (id:count): <code id=ids>-</code></div></div>
<div class=card><h3>COUNTERS</h3><div class=dim id=cnt>-</div></div>
<button onclick="fetch('/send')">Send test now</button><button onclick="fetch('/reset')">Reset</button><button onclick="fetch('/scan')">Auto baud scan on/off</button>
<br><button onclick="fetch('/baud?b=57600')">57600</button><button onclick="fetch('/baud?b=115200')">115200</button><button onclick="fetch('/baud?b=921600')">921600</button>
</main><script>
function lamp(el,st){el.className='light '+(st=='OK'?'good':(st=='NONE'||st=='NO REPLY'||st=='GARBAGE'?'bad':'wait'))}
async function t(){try{const s=await (await fetch('/status')).json();
 sum.textContent=s.summary;
 lamp(lr,s.rx);rs.textContent=s.rx=='OK'?'WORKING':(s.rx=='FRAMES'?'FRAMES OK':(s.rx=='GARBAGE'?'GARBAGE':'NO DATA'));
 rd.innerHTML=s.rxBytes+' bytes &middot; '+s.valid+' valid frames<br>baud '+s.baud+(s.locked?' (locked)':(s.scan?' (scanning)':''));
 lamp(lt,s.tx);ts.textContent=s.tx=='OK'?'WORKING':(s.tx=='NO REPLY'?'NO REPLY':'TESTING');
 td.innerHTML=s.txBytes+' bytes sent ('+s.rounds+' rounds)<br>replies: '+(s.params+s.acks);
 fc.innerHTML=s.hb>0?('sysid '+s.fcSys+' comp '+s.fcComp+' &middot; vehicle type '+s.fcType+' &middot; autopilot '+s.fcAp+'<br>armed: '+s.armed+' &middot; mode '+s.mode+(s.sysid>=0?'<br>SYSID_THISMAV = '+s.sysid:'')+(s.text?'<br>last text: '+s.text:'')):'no Pixhawk heartbeat heard yet';
 hex.textContent=s.hex||'(nothing received)';ids.textContent=s.ids||'-';
 cnt.innerHTML='heartbeats '+s.hb+' &middot; bad/unrecognised '+s.bad+' &middot; own frames echoed back '+s.echo+'<br>param replies '+s.params+' &middot; command acks '+s.acks+' (result '+s.ackResult+') &middot; ms since last byte '+s.msRx;
}catch(e){sum.textContent='lost connection to the ESP hotspot'}}
setInterval(t,1000);t();</script></body></html>)HTML";

static void handleRoot() { web.send_P(200, "text/html", PAGE); }

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(BAUDS[0]);
#if USE_SWAPPED_UART
  Serial.swap();                                 // UART0 -> D7 (RX) / D8 (TX)
#endif
  baudSetMs = millis();
  WiFi.mode(WIFI_AP);
  WiFi.softAP("PixhawkCheck", "12345678");
  web.on("/", handleRoot);
  web.on("/status", handleStatus);
  web.on("/send", handleSend);
  web.on("/reset", handleReset);
  web.on("/scan", handleScan);
  web.on("/baud", handleBaud);
  web.begin();
}

void loop() {
  pumpSerial();
  scanTick();
  if (autoTest && millis() - lastTestMs >= TEST_PERIOD_MS) { lastTestMs = millis(); sendTestRound(); }
  web.handleClient();
  yield();
}
