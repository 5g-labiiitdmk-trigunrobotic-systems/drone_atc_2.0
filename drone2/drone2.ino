/**
 * IIITDM KURNOOL — DRONE ESP32 / ESP8266 FIRMWARE v9 (ARM HANDSHAKE)
 * =====================================================
 * ZERO CONFIG — no hardcoded IP.
 * ESP listens for UDP broadcast from Flask on boot.
 * Flask broadcasts its IP every 3s → ESP discovers automatically.
 *
 * FIXES:
 *   - Removed automatic flight request on arm attempt.
 *   - Approval check every 1 second (was 2).
 *   - Strict arming prevention without approval.
 *   v9:
 *   - Blocked arm attempts now POST /drone/arm_request so the ATC dashboard
 *     shows an ARM REQUEST card. APPROVE ARM on the dashboard authorizes
 *     the flight; the pilot then arms again and is allowed through.
 *   - No re-request loop: requests are rate limited and never sent while
 *     already approved.
 *
 * LIBRARIES
 *   WiFiManager  by tzapu
 *   ArduinoJson  by bblanchon (v6)
 *   MAVLink      (copy mavlink/ folder next to this .ino)
 */

#if defined(ESP32)
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <WebServer.h>
  typedef WebServer LocalWebServer;
  // ESP32: FC MAVLink on Serial2, USB Serial stays free for debug prints.
  #define FC_SERIAL   Serial2
  #define FC_RX_PIN   16      // <- FC TELEM TX
  #define FC_TX_PIN   17      // -> FC TELEM RX
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <ESP8266WebServer.h>
  typedef ESP8266WebServer LocalWebServer;
  // ESP8266: FC MAVLink shares the hardware UART (RX/TX pins) with USB.
  #define FC_SERIAL   Serial
#endif
#include <WiFiUdp.h>
#include "mavlink/common/mavlink.h"
#include <ArduinoJson.h>
#include <WiFiManager.h>

// !! ONLY CHANGE THIS — unique per drone !!
const char* droneID = "Drone-1";   // Change to your drone ID

#define UDP_PORT        2390
#define MODE_STABILIZE  0
#define MODE_ALT_HOLD   2
#define MODE_AUTO       3
#define MODE_GUIDED     4
#define MODE_LOITER     5
#define MODE_RTL        6
#define MODE_LAND       9

#define ALT_CEILING_M          30.0f
#define ALT_FLOOR_M             0.5f
#define BATT_LAND_V            14.4f
#define HB_TIMEOUT_MS          4500

char serverBase[200] = "";
char serverIP[256]   = "";
bool serverFound     = false;

WiFiUDP udp;
LocalWebServer localServer(80);

float         lat               = 0.0f;
float         lon               = 0.0f;
float         alt               = 0.0f;
float         batt              = 0.0f;
String        flightMode        = "STABILIZE";
bool          armed             = false;
unsigned long lastHeartbeat     = 0;
bool          heartBeatOK       = false;
bool          gpsValid          = false;
bool          battFailsafeFired = false;
uint8_t       lastBaseMode      = 0;
bool          guidedRequested   = false;
unsigned long lastMoveTime      = 0;

bool          prevArmed         = false;
bool          flightApproved    = false;
bool          wasArmedInFlight  = false;
bool          armAllowed        = false;   // latched at the moment of arming
unsigned long approvalHoldoffUntil = 0;    // ignore server approval briefly after a disarm
unsigned long lastArmRequestMs  = 0;
#define ARM_REQUEST_MIN_GAP_MS 5000
#define DISARM_RETRY_MS        250
unsigned long lastDisarmMs      = 0;
bool          telNow            = false;   // push telemetry immediately

void requestArmAuthority();
void enforceArmLock();
void sendTelemetry();

#define DBG(x)    Serial.println(x)
#define DBGf(...) Serial.printf(__VA_ARGS__)

// =============================================================================
//  UDP Server Discovery
// =============================================================================

bool discoverServer() {
  DBG("[UDP] Listening for server broadcast...");
  udp.begin(UDP_PORT);

  unsigned long start = millis();
  while (millis() - start < 10000) {
    int len = udp.parsePacket();
    if (len > 0) {
      char buf[64] = {0};
      udp.read(buf, sizeof(buf) - 1);
      String msg = String(buf);
      msg.trim();
      DBGf("[UDP] Received: %s\n", msg.c_str());

      if (msg.startsWith("DRONE-ATC:")) {
        String port = msg.substring(10);
        port.trim();
        String senderIP = udp.remoteIP().toString();

        snprintf(serverBase, sizeof(serverBase),
                 "http://%s:%s", senderIP.c_str(), port.c_str());
        snprintf(serverIP, sizeof(serverIP),
                 "http://%s:%s/update", senderIP.c_str(), port.c_str());

        serverFound = true;
        udp.stop();
        DBGf("[UDP] ✓ Server found: %s\n", serverBase);
        return true;
      }
    }
    delay(100);
  }
  udp.stop();
  DBG("[UDP] ✗ No server found in 10s — will retry in loop");
  return false;
}

// =============================================================================
//  MAVLink helpers
// =============================================================================

void sendSetMode(uint8_t customMode) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t baseMode = lastBaseMode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  mavlink_msg_set_mode_pack(255, 200, &msg, 1, baseMode, customMode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

void sendCommandLong(uint16_t command,
                     float p1, float p2, float p3,
                     float p4, float p5, float p6, float p7) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(255, 200, &msg,
    1, 1, command, 0, p1, p2, p3, p4, p5, p6, p7);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

void sendVelocity(float vx, float vy, float vz) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t type_mask = 0x0FC7;
  mavlink_msg_set_position_target_local_ned_pack(
    255, 200, &msg, millis(), 1, 1,
    MAV_FRAME_LOCAL_NED, type_mask,
    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

void sendDisarm() {
  sendCommandLong(400, 0, 21196, 0, 0, 0, 0, 0);
  DBG("[SAFETY] Disarm sent");
}

void requestDataStreams() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  mavlink_msg_request_data_stream_pack(255, 200, &msg,
    1, 1, MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
  mavlink_msg_request_data_stream_pack(255, 200, &msg,
    1, 1, MAV_DATA_STREAM_POSITION, 2, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
  mavlink_msg_request_data_stream_pack(255, 200, &msg,
    1, 1, MAV_DATA_STREAM_EXTRA1, 2, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

// =============================================================================
//  MAVLink parser
// =============================================================================

void parseMAVLink() {
  mavlink_message_t msg;
  mavlink_status_t  status;

  while (FC_SERIAL.available()) {
    uint8_t c = FC_SERIAL.read();
    if (!mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) continue;

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        lastHeartbeat = millis();
        heartBeatOK   = true;
        lastBaseMode  = hb.base_mode;
        bool currentArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

        // FIX: Strict arming prevention – only allow if approved
        // Level-triggered: keep forcing disarm on EVERY heartbeat while armed
        // without approval (an edge-only check gave up after one failed try).
        if (currentArmed && !prevArmed) armAllowed = flightApproved;   // decide once, at arming
        if (!currentArmed) armAllowed = false;
        if (currentArmed && !armAllowed) {
          armed = true;
          enforceArmLock();
        }
        if (currentArmed && !prevArmed && flightApproved)
          DBG("[INFO] Arm approved — pilot cleared to fly");
        if (currentArmed && flightApproved) wasArmedInFlight = true;
        if (!currentArmed && wasArmedInFlight) {
          DBG("[INFO] Disarmed — authorization reset");
          flightApproved = false;
          wasArmedInFlight = false;
          approvalHoldoffUntil = millis() + 3000;
        }
        prevArmed = currentArmed;
        armed     = currentArmed;

        switch (hb.custom_mode) {
          case 0:             flightMode = "STABILIZE"; break;
          case 1:             flightMode = "ACRO";      break;
          case MODE_ALT_HOLD: flightMode = "ALT_HOLD";  break;
          case MODE_AUTO:     flightMode = "AUTO";       break;
          case MODE_GUIDED:   flightMode = "GUIDED";     break;
          case MODE_LOITER:   flightMode = "LOITER";     break;
          case MODE_RTL:      flightMode = "RTL";        break;
          case 7:             flightMode = "CIRCLE";     break;
          case MODE_LAND:     flightMode = "LAND";       break;
          default: flightMode = "MODE_" + String(hb.custom_mode);
        }
        break;
      }
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t p;
        mavlink_msg_global_position_int_decode(&msg, &p);
        if (p.lat != 0 || p.lon != 0) {
          lat = p.lat / 10000000.0f;
          lon = p.lon / 10000000.0f;
          alt = p.relative_alt / 1000.0f;
          gpsValid = true;
        }
        break;
      }
      case MAVLINK_MSG_ID_STATUSTEXT: {
        // ArduPilot announces "Arming motors" immediately; the heartbeat that
        // carries the armed flag can lag up to 1 s. React to the text first.
        mavlink_statustext_t st;
        mavlink_msg_statustext_decode(&msg, &st);
        if (strstr(st.text, "Arming motors") && !flightApproved && !armAllowed) {
          DBGf("[SAFETY] FC says '%s' without approval\n", st.text);
          armed = true;
          enforceArmLock();
        }
        break;
      }
      case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t s;
        mavlink_msg_sys_status_decode(&msg, &s);
        batt = s.voltage_battery / 1000.0f;
        break;
      }
    }
  }
  if (lastHeartbeat > 0 && millis() - lastHeartbeat > HB_TIMEOUT_MS) {
    if (heartBeatOK) DBG("[WARN] Heartbeat lost");
    heartBeatOK = false; gpsValid = false;
  }
}

// =============================================================================
//  HTTP functions
// =============================================================================

void sendTelemetry() {
  if (WiFi.status() != WL_CONNECTED || !serverFound) return;
  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverIP);
  http.setTimeout(800);
  http.addHeader("Content-Type", "application/json");
  String json =
    "{\"id\":\""   + String(droneID)                  + "\""
    ",\"lat\":"    + String(lat,  6)                   +
    ",\"lon\":"    + String(lon,  6)                   +
    ",\"alt\":"    + String(alt,  1)                   +
    ",\"batt\":"   + String(batt, 2)                   +
    ",\"mode\":\"" + flightMode                        + "\""
    ",\"mav_hb\":" + (heartBeatOK ? "true" : "false") +
    ",\"armed\":"  + (armed       ? "true" : "false")  +
    ",\"gps_ok\":" + (gpsValid    ? "true" : "false")  +
    ",\"ip\":\""   + WiFi.localIP().toString()         + "\"}";
  int code = http.POST(json);
  if (code == 200) DBG("[TEL] ✓ OK");
  else DBGf("[TEL] ✗ POST failed: HTTP %d\n", code);
  http.end();
}

void sendFlightRequest() {
  if (WiFi.status() != WL_CONNECTED || !serverFound) return;
  WiFiClient client;
  HTTPClient http;
  http.begin(client, String(serverBase) + "/pilot/request");
  http.setTimeout(800);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST("{\"drone_id\":\"" + String(droneID) + "\"}");
  DBGf("[REQ] Flight request → HTTP %d\n", code);
  http.end();
}

// Forces a disarm (retried every DISARM_RETRY_MS while still armed), asks the
// dashboard for authority and pushes telemetry right away so the alert is not
// delayed by the 1 s telemetry timer.
void enforceArmLock() {
  if (millis() - lastDisarmMs >= DISARM_RETRY_MS) {
    lastDisarmMs = millis();
    DBG("[SAFETY] Armed without approval — forcing disarm");
    sendDisarm();
    sendTelemetry();      // armed=true reaches the dashboard immediately
  }
  telNow = true;
  requestArmAuthority();
}

// Asks the ATC dashboard for arm authority (shows an ARM REQUEST card).
// Rate limited; never sent while a request is outstanding or flight is approved.
void requestArmAuthority() {
  if (WiFi.status() != WL_CONNECTED || !serverFound) return;
  if (flightApproved) return;
  if (lastArmRequestMs && millis() - lastArmRequestMs < ARM_REQUEST_MIN_GAP_MS) return;
  lastArmRequestMs = millis();
  WiFiClient client;
  HTTPClient http;
  http.begin(client, String(serverBase) + "/drone/arm_request");
  http.setTimeout(800);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST("{\"drone_id\":\"" + String(droneID) + "\"}");
  DBGf("[ARM] Arm request -> HTTP %d\n", code);
  http.end();
}

void checkApproval() {
  if (WiFi.status() != WL_CONNECTED || !serverFound) return;
  WiFiClient client;
  HTTPClient http;
  http.begin(client, String(serverBase) + "/pilot/status/" + String(droneID));
  http.setTimeout(800);
  if (http.GET() == 200) {
    StaticJsonDocument<128> doc;
    if (!deserializeJson(doc, http.getString())) {
      bool isApproved = doc["approved"] | false;
      String status   = doc["status"]   | "none";
      if (isApproved && !flightApproved && millis() >= approvalHoldoffUntil) {
        flightApproved = true;
        DBG("[INFO] Flight APPROVED");
      }
      if (!isApproved) {
        flightApproved = false;
      }
    }
  }
  http.end();
}

void pollCommands() {
  if (WiFi.status() != WL_CONNECTED || !serverFound) return;
  WiFiClient client;
  HTTPClient http;
  http.begin(client, String(serverBase) + "/drone/poll_commands/" + String(droneID));
  http.setTimeout(800);
  if (http.GET() == 200) {
    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, http.getString()) && doc.is<JsonArray>()) {
      for (JsonObject cmd : doc.as<JsonArray>()) {
        String type = cmd["cmd"] | "";
        DBGf("[CMD] %s\n", type.c_str());
        if      (type == "rtl")   sendSetMode(MODE_RTL);
        else if (type == "land")  sendSetMode(MODE_LAND);
        else if (type == "kill")  sendDisarm();
        else if (type == "hover") sendSetMode(MODE_LOITER);
        else if (type == "move") {
          if (!guidedRequested && flightMode != "GUIDED") {
            sendSetMode(MODE_GUIDED); guidedRequested = true;
          }
          sendVelocity(cmd["vx"]|0.0f, cmd["vy"]|0.0f, cmd["vz"]|0.0f);
          lastMoveTime = millis();
        }
      }
    }
  }
  http.end();
}

// =============================================================================
//  Failsafes
// =============================================================================

void velocityFailsafe() {
  if (lastMoveTime > 0 && millis() - lastMoveTime > 500) {
    sendVelocity(0, 0, 0); lastMoveTime = 0;
  }
}

void batteryFailsafe() {
  if (battFailsafeFired || batt < 1.0f || !armed) return;
  if (batt < BATT_LAND_V) {
    DBGf("[WARN] Low battery %.2fV — landing\n", batt);
    sendVelocity(0, 0, 0); sendSetMode(MODE_LAND);
    battFailsafeFired = true;
  }
}

// =============================================================================
//  Local server handlers
// =============================================================================

void addCORS() {
  localServer.sendHeader("Access-Control-Allow-Origin",  "*");
  localServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  localServer.sendHeader("Access-Control-Allow-Headers", "*");
}
void handleCORS()  { addCORS(); localServer.send(204); }
void handleRTL()   { sendSetMode(MODE_RTL);    addCORS(); localServer.send(200, "text/plain", "RTL_SENT"); }
void handleHover() { sendSetMode(MODE_LOITER); addCORS(); localServer.send(200, "text/plain", "HOVER_SENT"); }
void handleLand()  { sendSetMode(MODE_LAND);   addCORS(); localServer.send(200, "text/plain", "LAND_SENT"); }
void handleKill()  { sendSetMode(MODE_LAND);   addCORS(); localServer.send(200, "text/plain", "KILL_SENT"); }

void handleMove() {
  addCORS();
  if (!gpsValid)       { localServer.send(409, "text/plain", "NO_GPS");       return; }
  if (!flightApproved) { localServer.send(403, "text/plain", "NOT_APPROVED"); return; }
  if (!heartBeatOK)    { localServer.send(409, "text/plain", "NO_HEARTBEAT"); return; }
  if (!armed)          { localServer.send(409, "text/plain", "NOT_ARMED");    return; }
  if (!localServer.hasArg("plain")) { localServer.send(400, "text/plain", "NO_BODY"); return; }
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, localServer.arg("plain"))) {
    localServer.send(400, "text/plain", "JSON_ERR"); return;
  }
  float vx = constrain((float)(doc["vx"]|0.0f), -5.0f, 5.0f);
  float vy = constrain((float)(doc["vy"]|0.0f), -5.0f, 5.0f);
  float vz = constrain((float)(doc["vz"]|0.0f), -3.0f, 3.0f);
  if (alt >= ALT_CEILING_M && vz < 0) vz = 0;
  if (alt <= ALT_FLOOR_M   && vz > 0) vz = 0;
  if (!guidedRequested && flightMode != "GUIDED") {
    sendSetMode(MODE_GUIDED); guidedRequested = true;
  }
  sendVelocity(vx, vy, vz); lastMoveTime = millis();
  localServer.send(200, "text/plain", "MOVE_SENT");
}

void handleTest() {
  addCORS();
  localServer.send(200, "application/json",
    "{\"status\":\"ok\",\"drone\":\"" + String(droneID) +
    "\",\"ip\":\""    + WiFi.localIP().toString() +
    "\",\"batt\":"    + String(batt, 2) +
    ",\"mode\":\""    + flightMode + "\"" +
    ",\"server\":\"" + String(serverBase) + "\"}");
}

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(57600);          // debug (USB); on ESP8266 this is also the FC link
#if defined(ESP32)
  FC_SERIAL.begin(57600, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
#endif
  DBG("\n[BOOT] IIITDM Kurnool Drone Firmware v9 (ARM HANDSHAKE)");
  DBGf("[BOOT] Drone ID: %s\n", droneID);

  WiFiManager wm;
  wm.setConfigPortalTimeout(300);
  wm.setConnectTimeout(60);
  wm.setMinimumSignalQuality(10);
  wm.setSaveConnectTimeout(30);

  if (!wm.autoConnect("DroneSetup")) {
    DBG("[WIFI] Failed — restarting");
    delay(3000); ESP.restart();
  }
  DBGf("[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

  // Discover server via UDP broadcast
  for (int i = 0; i < 3 && !serverFound; i++) {
    DBGf("[UDP] Discovery attempt %d/3...\n", i + 1);
    discoverServer();
  }

  if (serverFound) {
    DBGf("[BOOT] Server: %s\n", serverBase);
  } else {
    DBG("[BOOT] Server not found yet — will keep trying in loop");
  }

  localServer.on("/rtl",   HTTP_GET,     handleRTL);
  localServer.on("/hover", HTTP_GET,     handleHover);
  localServer.on("/land",  HTTP_GET,     handleLand);
  localServer.on("/kill",  HTTP_GET,     handleKill);
  localServer.on("/move",  HTTP_POST,    handleMove);
  localServer.on("/test",  HTTP_GET,     handleTest);
  localServer.on("/rtl",   HTTP_OPTIONS, handleCORS);
  localServer.on("/hover", HTTP_OPTIONS, handleCORS);
  localServer.on("/land",  HTTP_OPTIONS, handleCORS);
  localServer.on("/kill",  HTTP_OPTIONS, handleCORS);
  localServer.on("/move",  HTTP_OPTIONS, handleCORS);
  localServer.begin();
  DBG("[HTTP] Local server on port 80");

  requestDataStreams();
  DBG("[BOOT] Setup complete\n");
}

// =============================================================================
//  LOOP
// =============================================================================

void loop() {
  localServer.handleClient();
  parseMAVLink();
  velocityFailsafe();
  batteryFailsafe();

  // Keep trying to discover server if not found yet
  static unsigned long lastDiscovery = 0;
  if (!serverFound && millis() - lastDiscovery > 15000) {
    discoverServer();
    lastDiscovery = millis();
  }

  // FIX: Check approval every 1 second (was 2) for faster response
  static unsigned long lastApprovalCheck = 0;
  if (!armed && millis() - lastApprovalCheck > 1000) {
    checkApproval();
    lastApprovalCheck = millis();
  }

  static unsigned long lastTel = 0;
  if (telNow || millis() - lastTel > 1000) {
    telNow = false;
    sendTelemetry();
    pollCommands();
    lastTel = millis();
  }
}