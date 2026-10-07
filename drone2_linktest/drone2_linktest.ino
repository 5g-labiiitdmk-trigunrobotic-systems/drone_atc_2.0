/**
 * FC <-> ESP8266 LINK TEST  (standalone diagnostic, not the flight firmware)
 * =========================================================================
 * Answers one question: does MAVLink flow in BOTH directions?
 *
 *   RX test  (FC -> ESP): counts bytes, valid MAVLink frames, FC heartbeats.
 *   TX test  (ESP -> FC): every 2 s the ESP asks the FC two things the FC MUST
 *            answer if it received them:
 *              - PARAM_REQUEST_READ "SYSID_THISMAV"  -> PARAM_VALUE reply
 *              - COMMAND_LONG REQUEST_MESSAGE(AUTOPILOT_VERSION) -> COMMAND_ACK
 *            Any reply proves the FC heard the ESP.
 *   ECHO test: unplug the FC and jumper D7 to D8 (ESP TX -> ESP RX). If the ESP
 *            hears its own frames, both ESP pins and the UART setup work, so a
 *            failure with the FC is the wire/FC side.
 *
 * No router, no USB serial needed: the ESP makes its own WiFi hotspot.
 *   1. Flash. 2. On a phone/PC join WiFi "DroneLinkTest" (password 12345678).
 *   3. Open http://192.168.4.1  - the verdict is shown at the top.
 *
 * Same wiring as the flight firmware:
 *   FC TELEM TX -> D7 (GPIO13)   FC TELEM RX -> D8 (GPIO15)   GND -> GND
 * FC settings: SERIALn_PROTOCOL = 2, SERIALn_BAUD = 57 (57600).
 * Libraries: none besides the ESP8266 core + MAVLink headers copied into
 * drone2_linktest/mavlink/ (so mavlink/common/mavlink.h exists).
 */
#if !defined(ESP8266)
#error "Select an ESP8266 board (NodeMCU)."
#endif
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "mavlink/common/mavlink.h"

#define FC_BAUD          57600
#define USE_SWAPPED_UART 1          // 1: FC on D7/D8.  0: FC on RX/TX pins (shared with USB)
#define TEST_PERIOD_MS   2000

ESP8266WebServer web(80);

// ---- counters ----
uint32_t rxBytes = 0, rxFrames = 0, rxDropped = 0, fcHeartbeats = 0, echoFrames = 0;
uint32_t txRounds = 0, txFrames = 0, paramReplies = 0, acks = 0;
unsigned long lastRxMs = 0, lastFcHbMs = 0, lastReplyMs = 0, lastTestMs = 0;
uint8_t  fcSys = 1, fcComp = 1, fcType = 0, fcAutopilot = 0;
bool     fcFound = false, fcArmed = false;
uint32_t fcCustomMode = 0;
float    sysidValue = -1;
uint8_t  lastAckResult = 255;
char     lastText[56] = "";
bool     autoTest = true;

static void fcSend(const mavlink_message_t& m) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
  Serial.write(buf, n);
  txFrames++;
}

static void sendTestRound() {
  mavlink_message_t m;
  // 1) read a parameter -> PARAM_VALUE
  mavlink_msg_param_request_read_pack(255, 200, &m, fcSys, fcComp, "SYSID_THISMAV", -1);
  fcSend(m);
  // 2) ask for AUTOPILOT_VERSION -> COMMAND_ACK (+ the message)
  mavlink_msg_command_long_pack(255, 200, &m, fcSys, fcComp, MAV_CMD_REQUEST_MESSAGE, 0,
                                MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0);
  fcSend(m);
  // 3) our own heartbeat (also what the echo test looks for)
  mavlink_msg_heartbeat_pack(255, 200, &m, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  fcSend(m);
  txRounds++;
}

static void handleMessage(const mavlink_message_t& m) {
  if (m.sysid == 255 && m.compid == 200) { echoFrames++; return; }   // our own frame came back
  if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_heartbeat_t hb; mavlink_msg_heartbeat_decode(&m, &hb);
    if (hb.autopilot != MAV_AUTOPILOT_INVALID && hb.type != MAV_TYPE_GCS) {
      fcFound = true; fcSys = m.sysid; fcComp = m.compid; fcType = hb.type; fcAutopilot = hb.autopilot;
      fcArmed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED; fcCustomMode = hb.custom_mode;
      fcHeartbeats++; lastFcHbMs = millis();
    }
  } else if (m.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
    mavlink_param_value_t p; mavlink_msg_param_value_decode(&m, &p);
    if (!strncmp(p.param_id, "SYSID_THISMAV", 13)) { paramReplies++; sysidValue = p.param_value; lastReplyMs = millis(); }
  } else if (m.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
    mavlink_command_ack_t a; mavlink_msg_command_ack_decode(&m, &a);
    if (a.command == MAV_CMD_REQUEST_MESSAGE) { acks++; lastAckResult = a.result; lastReplyMs = millis(); }
  } else if (m.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
    mavlink_statustext_t t; mavlink_msg_statustext_decode(&m, &t);
    strncpy(lastText, t.text, sizeof(lastText) - 1); lastText[sizeof(lastText) - 1] = 0;
    for (char* c = lastText; *c; c++) if (*c == '"' || *c == '\\' || (unsigned char)*c < 32) *c = ' ';   // keep the JSON valid
  }
}

static void pumpSerial() {
  static mavlink_message_t msg; static mavlink_status_t st;
  while (Serial.available()) {
    uint8_t c = Serial.read();
    rxBytes++; lastRxMs = millis();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &st)) { rxFrames++; handleMessage(msg); }
    rxDropped = st.packet_rx_drop_count;
  }
}

// ---- the verdict ----
static const char* verdict() {
  const unsigned long now = millis();
  const bool rxAlive = rxBytes > 0 && now - lastRxMs < 3000;
  const bool fromFc  = fcHeartbeats > 0;
  const bool replies = (paramReplies + acks) > 0;
  if (replies)                                return "TWO-WAY OK: the FC hears the ESP and the ESP hears the FC.";   // a reply proves both directions
  if (fromFc && txRounds >= 3 && !replies)    return "ONE-WAY: FC -> ESP works, ESP -> FC does NOT. Check D8 -> FC RX wire (not swapped, solid), common GND, and that the FC port is not receive-blocked. Then run the echo test (jumper D7-D8, FC unplugged).";
  if (fromFc)                                 return "FC -> ESP works. Waiting for the ESP -> FC test replies...";
  if (echoFrames > 0 && !fromFc)              return "ECHO OK: ESP TX pin and RX pin both work (loopback). No FC heard: connect the FC and remove the jumper.";
  if (rxBytes > 0 && rxFrames == 0)           return "Bytes arrive but no valid MAVLink: wrong baud (SERIALn_BAUD must be 57) or wrong protocol (SERIALn_PROTOCOL must be 2), or noise/loose GND.";
  if (rxFrames > 0 && !fromFc)                return "MAVLink frames arrive but none from an autopilot (only a GCS/companion?). Check which FC port you wired.";
  if (rxAlive)                                return "Receiving...";
  return "NO DATA from the FC: check FC TX -> D7, GND, FC port protocol/baud, and that the FC is powered. (To test the ESP alone: jumper D7 to D8.)";
}

static void handleStatus() {
  const unsigned long now = millis();
  char b[900];
  snprintf(b, sizeof b,
    "{\"verdict\":\"%s\",\"rxBytes\":%lu,\"rxFrames\":%lu,\"rxDropped\":%lu,\"fcHeartbeats\":%lu,\"fcFound\":%s,"
    "\"fcSys\":%u,\"fcComp\":%u,\"fcType\":%u,\"fcAutopilot\":%u,\"fcArmed\":%s,\"fcMode\":%lu,"
    "\"txFrames\":%lu,\"txRounds\":%lu,\"paramReplies\":%lu,\"acks\":%lu,\"ackResult\":%u,\"sysid\":%.0f,"
    "\"echoFrames\":%lu,\"msSinceRx\":%ld,\"msSinceHb\":%ld,\"text\":\"%s\",\"auto\":%s}",
    verdict(), (unsigned long)rxBytes, (unsigned long)rxFrames, (unsigned long)rxDropped, (unsigned long)fcHeartbeats,
    fcFound ? "true" : "false", fcSys, fcComp, fcType, fcAutopilot, fcArmed ? "true" : "false", (unsigned long)fcCustomMode,
    (unsigned long)txFrames, (unsigned long)txRounds, (unsigned long)paramReplies, (unsigned long)acks, lastAckResult, sysidValue,
    (unsigned long)echoFrames, lastRxMs ? (long)(now - lastRxMs) : -1L, lastFcHbMs ? (long)(now - lastFcHbMs) : -1L,
    lastText, autoTest ? "true" : "false");
  web.send(200, "application/json", b);
}
static void handleSend()  { sendTestRound(); web.send(200, "text/plain", "sent"); }
static void handleReset() {
  rxBytes = rxFrames = rxDropped = fcHeartbeats = echoFrames = txRounds = txFrames = paramReplies = acks = 0;
  lastRxMs = lastFcHbMs = lastReplyMs = 0; fcFound = false; sysidValue = -1; lastAckResult = 255; lastText[0] = 0;
  web.send(200, "text/plain", "reset");
}
static void handleToggle() { autoTest = !autoTest; web.send(200, "text/plain", autoTest ? "auto on" : "auto off"); }

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>FC link test</title><style>
body{font-family:system-ui,sans-serif;margin:0;background:#0f1320;color:#e8ecf4}
main{max-width:640px;margin:0 auto;padding:16px}
#v{padding:16px;border-radius:12px;font-size:18px;font-weight:600;line-height:1.35;background:#3a2f12}
#v.ok{background:#12351f}#v.bad{background:#401a1a}
.row{display:flex;gap:10px;margin-top:12px}.card{flex:1;background:#171c2e;border-radius:12px;padding:12px}
.card h3{margin:0 0 6px;font-size:13px;letter-spacing:.08em;color:#9aa6c4}
.big{font-size:22px;font-weight:700}.dim{color:#9aa6c4;font-size:13px;line-height:1.6}
button{background:#2b3358;color:#fff;border:0;border-radius:10px;padding:12px 14px;font-size:15px;margin:12px 8px 0 0}
</style></head><body><main>
<div id=v>connecting...</div>
<div class=row>
 <div class=card><h3>FC &rarr; ESP (RX)</h3><div class=big id=rx>-</div><div class=dim id=rxd></div></div>
 <div class=card><h3>ESP &rarr; FC (TX)</h3><div class=big id=tx>-</div><div class=dim id=txd></div></div>
</div>
<div class=card style="margin-top:12px"><h3>FC</h3><div class=dim id=fc>-</div></div>
<button onclick="fetch('/send')">Send test now</button><button onclick="fetch('/reset')">Reset counters</button><button onclick="fetch('/toggle')">Auto test on/off</button>
<p class=dim>Echo test: unplug the FC, jumper D7 to D8. "ECHO OK" means the ESP pins work.</p>
</main><script>
async function t(){try{const s=await (await fetch('/status')).json();
 const v=document.getElementById('v');v.textContent=s.verdict;v.className=s.verdict.startsWith('TWO-WAY')||s.verdict.startsWith('ECHO OK')?'ok':(s.verdict.startsWith('ONE-WAY')||s.verdict.startsWith('NO DATA')?'bad':'');
 rx.textContent=s.fcHeartbeats>0?'OK':(s.rxBytes>0?'bytes only':'none');
 rxd.innerHTML=s.rxBytes+' bytes, '+s.rxFrames+' frames, '+s.fcHeartbeats+' FC heartbeats<br>bad/dropped packets: '+s.rxDropped+'<br>last byte '+(s.msSinceRx<0?'never':s.msSinceRx+' ms ago');
 tx.textContent=(s.paramReplies+s.acks)>0?'OK':(s.txRounds>=3?'NO REPLY':'testing...');
 txd.innerHTML=s.txFrames+' frames sent in '+s.txRounds+' rounds<br>param replies: '+s.paramReplies+' &nbsp; acks: '+s.acks+'<br>echo (own frames back): '+s.echoFrames;
 fc.innerHTML=s.fcFound?('sysid '+s.fcSys+' comp '+s.fcComp+' type '+s.fcType+' autopilot '+s.fcAutopilot+'<br>armed: '+s.fcArmed+' &nbsp; mode '+s.fcMode+'<br>SYSID_THISMAV = '+s.sysid+'<br>last FC text: '+(s.text||'-')):'no autopilot heard yet';
}catch(e){document.getElementById('v').textContent='lost connection to the ESP hotspot'}}
setInterval(t,1000);t();</script></body></html>)HTML";

static void handleRoot() { web.send_P(200, "text/html", PAGE); }

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(FC_BAUD);
#if USE_SWAPPED_UART
  Serial.swap();                         // UART0 -> D7 (RX) / D8 (TX)
#endif
  WiFi.mode(WIFI_AP);
  WiFi.softAP("DroneLinkTest", "12345678");
  web.on("/", handleRoot);
  web.on("/status", handleStatus);
  web.on("/send", handleSend);
  web.on("/reset", handleReset);
  web.on("/toggle", handleToggle);
  web.begin();
}

void loop() {
  pumpSerial();
  if (autoTest && millis() - lastTestMs >= TEST_PERIOD_MS) { lastTestMs = millis(); sendTestRound(); }
  web.handleClient();
  yield();
}
