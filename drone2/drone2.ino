/**
 * IIITDM Kurnool x Trigun Robotics - DRONE ATC BRIDGE FIRMWARE v10 (ESP8266)
 * ==========================================================================
 * Bridges an ArduPilot flight controller (MAVLink over UART) to the Flask
 * ATC server over WiFi.
 *
 *   FC TELEM TX -> NodeMCU D7 (GPIO13, RX)      [USE_SWAPPED_UART 1, default]
 *   FC TELEM RX -> NodeMCU D8 (GPIO15, TX)
 *   FC GND      -> NodeMCU GND                  (power the NodeMCU separately)
 *   Debug text  -> Serial1 TX on D4 (GPIO2)     (USB-TTL adapter, 115200 baud)
 *
 * What it does
 *   - Finds the server by UDP broadcast ("DRONE-ATC:<port>", port 2390) and
 *     follows it if its address changes. Optional fixed fallback URL below.
 *   - Posts telemetry to /update (1 Hz, immediately on an unauthorized arm).
 *   - Arm authority: an arm that the ESP did not allow is force-disarmed and
 *     reported (/drone/arm_request). The ESP can only react AFTER the FC arms;
 *     to prevent arming outright see drone2/README.md (FC-side settings).
 *   - Executes authority commands from /drone/poll_commands: rtl, land, hover,
 *     kill, move, arm (pilot, needs approval), auth_arm (authority override),
 *     disarm.
 *   - Failsafes: velocity stop, low-battery land.
 *
 * Safety rules baked in
 *   - Never force-disarm something it did not SEE go from disarmed to armed
 *     (an ESP reset in flight must not drop the drone out of the sky).
 *   - Never force-disarm while the drone is clearly airborne; alert instead.
 *   - Approval is only trusted while the server can be reached.
 *   - No unauthenticated control endpoints on the ESP (only /test status).
 *
 * LIBRARIES (Arduino IDE / PlatformIO)
 *   WiFiManager (tzapu), ArduinoJson v6 (bblanchon),
 *   MAVLink C headers: already bundled in drone2/mavlink/ (nothing to install).
 * BOARD: NodeMCU 1.0 (ESP-12E) or any ESP8266.
 */

#if !defined(ESP8266)
#error "drone2.ino targets the ESP8266 (NodeMCU). Select an ESP8266 board."
#endif

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "mavlink/common/mavlink.h"

// ============================================================================
//  CONFIG  (the only part you normally edit)
// ============================================================================
const char* DRONE_ID = "Drone-1";          // unique per drone; must match the pilot portal

#define FIRMWARE_VERSION      "v10"
#define SERVER_FALLBACK_URL   ""           // e.g. "http://192.168.1.50:5000"; "" = UDP discovery only
#define UDP_PORT              2390
#define FC_BAUD               57600
#define USE_SWAPPED_UART      1            // 1: FC on D7/D8, debug on Serial1(D4). 0: FC on RX/TX pins

#define HTTP_TIMEOUT_MS       700
#define TELEMETRY_PERIOD_MS   1000
#define APPROVAL_PERIOD_MS    300          // while disarmed
#define COMMAND_PERIOD_MS     500
#define ARM_REQUEST_GAP_MS    5000
#define DISARM_RETRY_MS       250
#define APPROVAL_STALE_MS     15000        // server unreachable this long => approval dropped
#define APPROVAL_HOLDOFF_MS   3000         // ignore server approval right after a flight
#define AUTH_ARM_WINDOW_MS    10000
#define HB_TIMEOUT_MS         4500
#define AIRBORNE_ALT_M        2.0f         // above this we never force-disarm
#define BATT_LAND_V           14.4f
#define BATT_LOW_COUNT        5            // consecutive low readings before landing

// ArduCopter custom modes
#define MODE_STABILIZE 0
#define MODE_ACRO      1
#define MODE_ALT_HOLD  2
#define MODE_AUTO      3
#define MODE_GUIDED    4
#define MODE_LOITER    5
#define MODE_RTL       6
#define MODE_CIRCLE    7
#define MODE_LAND      9

#if USE_SWAPPED_UART
  #define DBG_SERIAL Serial1
#else
  #define DBG_SERIAL Serial
#endif
#define FC_SERIAL Serial
#define DBG(x)    DBG_SERIAL.println(x)
#define DBGf(...) DBG_SERIAL.printf(__VA_ARGS__)

// ============================================================================
//  STATE
// ============================================================================
WiFiUDP            udp;
ESP8266WebServer   localServer(80);
bool               udpStarted = false;

// --- server ---
String             serverBase = "";                 // "http://ip:port"
bool               serverKnown = false;

// --- FC link ---
uint8_t            fcSys = 1, fcComp = 1;
bool               fcLearned = false;               // learned sysid/compid from a real autopilot heartbeat
bool               hbOK = false;
unsigned long      lastHbMs = 0;
uint8_t            lastBaseMode = 0;
String             flightMode = "UNKNOWN";

// --- vehicle data ---
double             lat = 0, lon = 0;
float              alt = 0, batt = 0;
bool               gpsValid = false;
unsigned long      lastPosMs = 0, lastSysMs = 0;

// --- arming state ---
bool               fcArmed = false;
bool               armKnown = false;                // first heartbeat seen
bool               sawDisarmed = false;             // we observed it disarmed (so a later arm is a real edge)
bool               armAllowed = false;              // latched at the moment of arming
bool               approved = false;                // server says flight approved
unsigned long      lastApprovalOkMs = 0;
unsigned long      holdoffUntil = 0;
unsigned long      authArmUntil = 0;
unsigned long      lastDisarmMs = (unsigned long)(-(long)DISARM_RETRY_MS);   // "long ago", so the first one is not rate limited
bool               lastArmCmdWasArm = false;
bool               wantArmRequest = false;
unsigned long      lastArmReqMs = (unsigned long)(-(long)ARM_REQUEST_GAP_MS);
uint16_t           lockHits = 0;                    // diagnostic: forced disarms sent

// --- scheduling ---
unsigned long      lastTelMs = 0, lastApprovalMs = 0, lastCmdMs = 0, lastStreamMs = 0;
bool               telNow = false;

// --- motion / failsafe ---
unsigned long      lastMoveMs = 0, lastGuidedReqMs = 0;
uint8_t            lowBattReadings = 0;
bool               battFailsafeFired = false;

// ============================================================================
//  MAVLink out
// ============================================================================
static void fcSend(const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

static void sendCommandLong(uint16_t cmd, float p1, float p2 = 0, float p3 = 0,
                            float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(255, 200, &msg, fcSys, fcComp, cmd, 0, p1, p2, p3, p4, p5, p6, p7);
  fcSend(msg);
}

static void sendSetMode(uint8_t customMode) {
  mavlink_message_t msg;
  mavlink_msg_set_mode_pack(255, 200, &msg, fcSys,
                            lastBaseMode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, customMode);
  fcSend(msg);
}

static void sendVelocity(float vx, float vy, float vz) {
  mavlink_message_t msg;
  const uint16_t typeMask = 0x0FC7;               // velocity only
  mavlink_msg_set_position_target_local_ned_pack(255, 200, &msg, millis(), fcSys, fcComp,
      MAV_FRAME_LOCAL_NED, typeMask, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0);
  fcSend(msg);
}

static void sendForceDisarm() {                    // 21196 = force, works even when "in flight"
  lastArmCmdWasArm = false;
  sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196);
}
static void sendDisarm()  { lastArmCmdWasArm = false; sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0); }
static void sendArm()     { lastArmCmdWasArm = true;  sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0); }

static void requestDataStreams() {
  const uint8_t streams[] = { MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_EXTRA1 };
  for (uint8_t s : streams) {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(255, 200, &msg, fcSys, fcComp, s, 2, 1);
    fcSend(msg);
  }
  lastStreamMs = millis();
}

// ============================================================================
//  Arming authority (the core safety logic)
// ============================================================================
static bool airborne() {
  return gpsValid && (millis() - lastPosMs < 3000) && alt > AIRBORNE_ALT_M;
}

// Called whenever the FC's armed state is (re)evaluated.
static void onArmState(bool nowArmed) {
  const unsigned long now = millis();

  if (!armKnown) {                                  // very first reading after boot
    armKnown = true;
    fcArmed = nowArmed;
    if (nowArmed) {                                 // armed before we ever saw it disarmed:
      armAllowed = true;                            // adopt it, never force-disarm (ESP reset in flight!)
      wantArmRequest = true;                        // but tell the authority
      DBG("[ARM] FC already armed at boot - adopted, authority notified");
    } else {
      sawDisarmed = true;
    }
    return;
  }

  if (nowArmed && !fcArmed) {                       // observed disarmed -> armed
    fcArmed = true;
    armAllowed = !sawDisarmed || approved || now < authArmUntil;
    DBGf("[ARM] FC armed. allowed=%d (approved=%d authArm=%d)\n", armAllowed, approved, now < authArmUntil);
  } else if (!nowArmed && fcArmed) {                // armed -> disarmed
    fcArmed = false;
    sawDisarmed = true;
    if (armAllowed) {                               // a permitted flight ended: spend the approval
      approved = false;
      holdoffUntil = now + APPROVAL_HOLDOFF_MS;
      DBG("[ARM] Disarmed - approval used up");
    }
    armAllowed = false;
    battFailsafeFired = false;
    lowBattReadings = 0;
  }
  telNow = true;                                    // dashboard sees every arm/disarm at once
}

// Runs every loop pass. Keeps disarming until the FC reports disarmed.
static void enforceArmLock() {
  if (!(fcArmed && !armAllowed)) return;
  const unsigned long now = millis();
  wantArmRequest = true;
  if (airborne()) {                                 // never drop an airborne drone
    if (now - lastDisarmMs >= 2000) { lastDisarmMs = now; telNow = true; DBG("[SAFETY] Unauthorized arm but AIRBORNE - alert only"); }
    return;
  }
  if (now - lastDisarmMs >= DISARM_RETRY_MS) {
    lastDisarmMs = now;
    lockHits++;
    sendForceDisarm();
    telNow = true;
    DBG("[SAFETY] Armed without authority -> force disarm");
  }
}

// ============================================================================
//  MAVLink in
// ============================================================================
static void onHeartbeat(const mavlink_message_t& msg) {
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(&msg, &hb);
  if (hb.autopilot == MAV_AUTOPILOT_INVALID || hb.type == MAV_TYPE_GCS) return;   // not the autopilot
  if (!fcLearned) {
    fcSys = msg.sysid; fcComp = msg.compid; fcLearned = true;
    DBGf("[FC] Autopilot found: sys=%u comp=%u\n", fcSys, fcComp);
    requestDataStreams();
  }
  if (msg.sysid != fcSys) return;

  lastHbMs = millis();
  hbOK = true;
  lastBaseMode = hb.base_mode;
  switch (hb.custom_mode) {
    case MODE_STABILIZE: flightMode = "STABILIZE"; break;
    case MODE_ACRO:      flightMode = "ACRO";      break;
    case MODE_ALT_HOLD:  flightMode = "ALT_HOLD";  break;
    case MODE_AUTO:      flightMode = "AUTO";      break;
    case MODE_GUIDED:    flightMode = "GUIDED";    break;
    case MODE_LOITER:    flightMode = "LOITER";    break;
    case MODE_RTL:       flightMode = "RTL";       break;
    case MODE_CIRCLE:    flightMode = "CIRCLE";    break;
    case MODE_LAND:      flightMode = "LAND";      break;
    default:             flightMode = String("MODE_") + String((unsigned)hb.custom_mode);
  }
  onArmState((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);
}

static void parseMAVLink() {
  mavlink_message_t msg;
  mavlink_status_t  st;
  while (FC_SERIAL.available()) {
    uint8_t c = FC_SERIAL.read();
    if (!mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &st)) continue;

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) { onHeartbeat(msg); continue; }
    if (!fcLearned || msg.sysid != fcSys) continue;

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t p;
        mavlink_msg_global_position_int_decode(&msg, &p);
        alt = p.relative_alt / 1000.0f;
        lastPosMs = millis();
        if (p.lat != 0 || p.lon != 0) { lat = p.lat / 1e7; lon = p.lon / 1e7; gpsValid = true; }
        break;
      }
      case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t s;
        mavlink_msg_sys_status_decode(&msg, &s);
        lastSysMs = millis();
        if (s.voltage_battery != UINT16_MAX) {       // 65535 mV = "unknown"
          batt = s.voltage_battery / 1000.0f;
          if (fcArmed && batt > 1.0f && batt < BATT_LAND_V) { if (lowBattReadings < 255) lowBattReadings++; }
          else lowBattReadings = 0;
        }
        break;
      }
      case MAVLINK_MSG_ID_STATUSTEXT: {
        // ArduPilot announces the arm before the next 1 Hz heartbeat carries it.
        mavlink_statustext_t t;
        mavlink_msg_statustext_decode(&msg, &t);
        if (!fcArmed && armKnown && strstr(t.text, "Arming motors")) onArmState(true);
        break;
      }
      case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t a;
        mavlink_msg_command_ack_decode(&msg, &a);
        if (a.command == MAV_CMD_COMPONENT_ARM_DISARM && a.result == MAV_RESULT_ACCEPTED && armKnown)
          onArmState(lastArmCmdWasArm);              // react now, don't wait for the heartbeat
        break;
      }
    }
  }
  if (lastHbMs && millis() - lastHbMs > HB_TIMEOUT_MS) {
    if (hbOK) DBG("[FC] Heartbeat lost");
    hbOK = false; gpsValid = false;
  }
}

// ============================================================================
//  Server link
// ============================================================================
static void setServer(const String& base) {
  if (base != serverBase) { serverBase = base; DBG(String("[NET] Server: ") + serverBase); }
  serverKnown = true;
}

static void udpTick() {
  if (WiFi.status() != WL_CONNECTED) { udpStarted = false; return; }
  if (!udpStarted) { udp.begin(UDP_PORT); udpStarted = true; }
  int len;
  while ((len = udp.parsePacket()) > 0) {            // keep listening forever: follows server moves
    char buf[48] = {0};
    udp.read(buf, sizeof(buf) - 1);
    if (strncmp(buf, "DRONE-ATC:", 10) == 0) {
      String port = String(buf + 10); port.trim();
      setServer(String("http://") + udp.remoteIP().toString() + ":" + port);
    }
  }
}

// Returns the HTTP status code, or <0 on a local/network failure.
static int httpCall(bool post, const String& path, const String& body, String* resp) {
  if (!serverKnown || WiFi.status() != WL_CONNECTED) return -100;
  WiFiClient client;
  HTTPClient http;
  http.setReuse(false);
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(client, serverBase + path)) return -101;
  int code;
  if (post) { http.addHeader("Content-Type", "application/json"); code = http.POST(body); }
  else      { code = http.GET(); }
  if (code == 200 && resp) *resp = http.getString();
  http.end();
  return code;
}

static void postTelemetry() {
  // Strings must outlive serializeJson (ArduinoJson keeps const char* by pointer).
  const String sLat = String(lat, 6), sLon = String(lon, 6), sAlt = String(alt, 1), sBat = String(batt, 2);
  const String sIp = WiFi.localIP().toString();
  StaticJsonDocument<512> d;
  d["id"] = DRONE_ID;
  d["lat"] = serialized(sLat.c_str());
  d["lon"] = serialized(sLon.c_str());
  d["alt"] = serialized(sAlt.c_str());
  d["batt"] = serialized(sBat.c_str());
  d["mode"] = flightMode.c_str();
  d["mav_hb"] = hbOK;
  d["armed"] = fcArmed;
  d["gps_ok"] = gpsValid;
  d["ip"] = sIp.c_str();
  d["rssi"] = WiFi.RSSI();
  d["fw"] = FIRMWARE_VERSION;
  d["fw_approved"] = approved;
  d["fw_arm_allowed"] = armAllowed;
  d["fw_lock_hits"] = lockHits;
  char buf[512];
  serializeJson(d, buf, sizeof(buf));
  httpCall(true, "/update", String(buf), nullptr);
}

static void postArmRequest() {
  String body = String("{\"drone_id\":\"") + DRONE_ID + "\"}";
  int code = httpCall(true, "/drone/arm_request", body, nullptr);
  DBGf("[ARM] arm request -> HTTP %d\n", code);
}

static void pollApproval() {
  String resp;
  int code = httpCall(false, String("/pilot/status/") + DRONE_ID, "", &resp);
  const unsigned long now = millis();
  if (code == 200) {
    StaticJsonDocument<192> d;
    if (!deserializeJson(d, resp.c_str())) {
      lastApprovalOkMs = now;
      bool serverApproved = d["approved"] | false;
      if (!serverApproved)                      approved = false;
      else if (now >= holdoffUntil)             approved = true;
    }
  }
  if (approved && now - lastApprovalOkMs > APPROVAL_STALE_MS) {   // can't verify -> don't trust
    approved = false;
    DBG("[NET] Approval dropped: server unreachable");
  }
}

static void runCommand(const char* type, JsonObject cmd) {
  const unsigned long now = millis();
  DBGf("[CMD] %s\n", type);
  if      (!strcmp(type, "rtl"))   sendSetMode(MODE_RTL);
  else if (!strcmp(type, "land"))  sendSetMode(MODE_LAND);
  else if (!strcmp(type, "hover")) sendSetMode(MODE_LOITER);
  else if (!strcmp(type, "kill"))  sendForceDisarm();
  else if (!strcmp(type, "disarm")) sendDisarm();                       // FC refuses it in flight
  else if (!strcmp(type, "arm")) {                                      // pilot, needs approval
    if (approved && hbOK && !fcArmed) sendArm(); else DBG("[CMD] arm ignored (not approved / no FC / already armed)");
  }
  else if (!strcmp(type, "auth_arm")) {                                 // authority override
    if (hbOK && !fcArmed) { authArmUntil = now + AUTH_ARM_WINDOW_MS; sendArm(); }
  }
  else if (!strcmp(type, "move")) {
    if (flightMode != "GUIDED" && now - lastGuidedReqMs > 1500) { lastGuidedReqMs = now; sendSetMode(MODE_GUIDED); }
    sendVelocity(cmd["vx"] | 0.0f, cmd["vy"] | 0.0f, cmd["vz"] | 0.0f);
    lastMoveMs = now;
  }
}

static void pollCommands() {
  String resp;
  if (httpCall(false, String("/drone/poll_commands/") + DRONE_ID, "", &resp) != 200) return;
  StaticJsonDocument<768> d;
  if (deserializeJson(d, resp.c_str()) || !d.is<JsonArray>()) return;
  for (JsonObject cmd : d.as<JsonArray>()) {
    const char* type = cmd["cmd"] | "";
    runCommand(type, cmd);
  }
}

// At most ONE blocking HTTP request per loop pass, so MAVLink is parsed in between.
static void netTick() {
  const unsigned long now = millis();
  if (!serverKnown || WiFi.status() != WL_CONNECTED) return;

  if (wantArmRequest && !approved && now - lastArmReqMs >= ARM_REQUEST_GAP_MS) {
    wantArmRequest = false; lastArmReqMs = now; postArmRequest(); return;
  }
  if (wantArmRequest && approved) wantArmRequest = false;
  if (telNow || now - lastTelMs >= TELEMETRY_PERIOD_MS) {
    telNow = false; lastTelMs = now; postTelemetry(); return;
  }
  if (!fcArmed && now - lastApprovalMs >= APPROVAL_PERIOD_MS) {
    lastApprovalMs = now; pollApproval(); return;
  }
  if (now - lastCmdMs >= COMMAND_PERIOD_MS) {
    lastCmdMs = now; pollCommands(); return;
  }
}

// ============================================================================
//  Failsafes
// ============================================================================
static void failsafeTick() {
  const unsigned long now = millis();
  if (lastMoveMs && now - lastMoveMs > 500) { sendVelocity(0, 0, 0); lastMoveMs = 0; }
  if (!battFailsafeFired && fcArmed && lowBattReadings >= BATT_LOW_COUNT) {
    DBGf("[WARN] Low battery %.2f V - landing\n", batt);
    sendVelocity(0, 0, 0); sendSetMode(MODE_LAND);
    battFailsafeFired = true;
  }
  if (fcLearned && now - lastStreamMs > 15000 && now - lastSysMs > 5000) requestDataStreams();   // FC rebooted?
}

// ============================================================================
//  Local status page (read-only; no control endpoints on purpose)
// ============================================================================
static void handleTest() {
  StaticJsonDocument<384> d;
  d["status"] = "ok";
  d["drone"] = DRONE_ID;
  d["fw"] = FIRMWARE_VERSION;
  const String sIp = WiFi.localIP().toString();
  d["ip"] = sIp.c_str();
  d["server"] = serverBase.c_str();
  d["fc_heartbeat"] = hbOK;
  d["armed"] = fcArmed;
  d["approved"] = approved;
  d["lock_hits"] = lockHits;
  char buf[384];
  serializeJson(d, buf, sizeof(buf));
  localServer.send(200, "application/json", buf);
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup() {
  FC_SERIAL.setRxBufferSize(1024);                   // HTTP calls block; don't lose MAVLink meanwhile
  FC_SERIAL.begin(FC_BAUD);
#if USE_SWAPPED_UART
  FC_SERIAL.swap();                                  // UART0 -> GPIO13 (RX/D7), GPIO15 (TX/D8)
  Serial1.begin(115200);                             // debug on D4
#endif
  DBGf("\n[BOOT] Drone ATC firmware %s, id %s\n", FIRMWARE_VERSION, DRONE_ID);

  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  WiFiManager wm;
  wm.setConfigPortalTimeout(300);
  wm.setConnectTimeout(30);
  if (!wm.autoConnect("DroneSetup")) { DBG("[WIFI] failed - restarting"); delay(2000); ESP.restart(); }
  DBGf("[WIFI] connected, IP %s\n", WiFi.localIP().toString().c_str());

  if (strlen(SERVER_FALLBACK_URL) > 0) setServer(String(SERVER_FALLBACK_URL));

  localServer.on("/test", HTTP_GET, handleTest);
  localServer.begin();
}

void loop() {
  parseMAVLink();
  enforceArmLock();
  failsafeTick();
  udpTick();
  localServer.handleClient();
  netTick();
  yield();
}
