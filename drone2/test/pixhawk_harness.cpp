#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "ESP8266WebServer.h"
#include <ArduinoJson.h>
unsigned long g_ms = 0; HardwareSerial Serial, Serial1; WiFiClass WiFi; EspClass ESP; std::string g_lastBody;
#include "../../pixhawk_esp_datacheck/pixhawk_esp_datacheck.ino"
#include "mavlink/common/mavlink.h"

static bool pxHears = true, loopback = false, pxAlive = true, pxV1 = false, pxArmed = false;
static std::vector<mavlink_message_t> got;                     // what the ESP sent, decoded by the REAL library
static void feed(const mavlink_message_t& m) {
  uint8_t b[300]; mavlink_status_t* cs = mavlink_get_channel_status(MAVLINK_COMM_2);
  if (pxV1) cs->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1; else cs->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink_message_t c = m; uint16_t n = mavlink_msg_to_send_buffer(b, &c);
  for (int i = 0; i < n; i++) Serial.rx.push_back(b[i]);
}
static void pxBeat() { mavlink_message_t m; mavlink_msg_heartbeat_pack_chan(1, 1, MAVLINK_COMM_2, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, (pxArmed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, MAV_STATE_STANDBY); feed(m); }
static void drain() {
  static mavlink_message_t m; static mavlink_status_t st;
  for (uint8_t b : Serial.tx) {
    if (loopback) Serial.rx.push_back(b);
    if (mavlink_parse_char(MAVLINK_COMM_1, b, &m, &st)) {
      got.push_back(m);
      if (!pxHears) continue;
      if (m.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) { mavlink_message_t r; mavlink_msg_param_value_pack_chan(1, 1, MAVLINK_COMM_2, &r, "SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 0); feed(r); }
      if (m.msgid == MAVLINK_MSG_ID_COMMAND_LONG) { mavlink_command_long_t c; mavlink_msg_command_long_decode(&m, &c); if (c.command == 512) { mavlink_message_t r; mavlink_msg_command_ack_pack_chan(1, 1, MAVLINK_COMM_2, &r, 512, MAV_RESULT_ACCEPTED, 0, 0, 0, 0); feed(r); } }
    }
  }
  Serial.tx.clear();
}
static void run(unsigned ms) { static unsigned long lb = 0; for (unsigned t = 0; t < ms; t += 10) { g_ms += 10; if (pxAlive && g_ms - lb >= 1000) { lb = g_ms; pxBeat(); } loop(); drain(); } }
static int fails = 0;
#define CHECK(c, msg) do { bool ok_ = (c); printf("  [%s] %s\n", ok_ ? "PASS" : "FAIL", msg); if (!ok_) fails++; } while (0)
static bool sw(const char* s, const char* p) { return strncmp(s, p, strlen(p)) == 0; }
int main(int argc, char** argv) {
  std::string sc = argv[1]; printf("scenario: %s\n", sc.c_str()); setup();
  if (sc == "encoder") {
    pxAlive = false; pxHears = false; autoTest = false; sendTestRound(); sendTestRound(); drain();
    int pr = 0, cl = 0, hb = 0; bool v2 = false, v1 = false, fieldsOk = true;
    for (auto& m : got) {
      if (m.magic == 0xFD) v2 = true; if (m.magic == 0xFE) v1 = true;
      if (m.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) { pr++; mavlink_param_request_read_t p; mavlink_msg_param_request_read_decode(&m, &p); if (p.target_system != 1 || p.target_component != 1 || p.param_index != -1 || strncmp(p.param_id, "SYSID_THISMAV", 13)) fieldsOk = false; }
      if (m.msgid == MAVLINK_MSG_ID_COMMAND_LONG) { cl++; mavlink_command_long_t c; mavlink_msg_command_long_decode(&m, &c); if (c.command != 512 || c.param1 != 148.0f || c.target_system != 1) fieldsOk = false; }
      if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) { hb++; if (m.sysid != 255 || m.compid != 200) fieldsOk = false; }
    }
    CHECK(pr == 2 && cl == 2 && hb == 2, "2 rounds -> 2x param read, 2x command, 2x heartbeat all pass the REAL library's CRC check");
    CHECK(v1 && v2, "rounds alternate MAVLink 1 and MAVLink 2"); CHECK(fieldsOk, "decoded fields are exactly what was intended");
  }
  if (sc == "two_way")  { run(6000); CHECK(!strcmp(txVerdict(), "OK") && !strcmp(rxVerdict(), "OK"), "both lights OK"); CHECK(sw(summary(), "BOTH WAYS WORK"), summary()); CHECK(paramReplies > 0 && acks > 0 && sysidValue == 1.0f, "param value + ack parsed"); }
  if (sc == "two_way_mavlink1") { pxV1 = true; run(6000); CHECK(sw(summary(), "BOTH WAYS WORK"), "works when the Pixhawk speaks MAVLink 1"); CHECK(paramReplies > 0 && hbCount > 0, "v1 frames parsed"); }
  if (sc == "rx_only")  { pxHears = false; run(9000); CHECK(!strcmp(rxVerdict(), "OK") && !strcmp(txVerdict(), "NO REPLY"), "RX light OK, TX light NO REPLY"); CHECK(sw(summary(), "ONE WAY ONLY"), summary()); }
  if (sc == "no_data")  { pxAlive = false; pxHears = false; run(6000); CHECK(!strcmp(rxVerdict(), "NONE") && sw(summary(), "NO DATA"), summary()); }
  if (sc == "echo")     { pxAlive = false; pxHears = false; loopback = true; run(6000); CHECK(sw(summary(), "ECHO OK") && ownEcho > 3, summary()); }
  if (sc == "armed_parse") { pxArmed = true; run(2500); web.call("/status"); StaticJsonDocument<2048> d; CHECK(!deserializeJson(d, g_lastBody.c_str()) && d["armed"] == true && d["mode"] == 4 && d["fcSys"] == 1, "armed flag + mode decoded from a trimmed v2 heartbeat"); }
  if (sc == "baud_scan") {
    pxAlive = false; pxHears = false; CHECK(BAUDS[baudIdx] == 57600, "starts at 57600");
    for (int i = 0; i < 700; i++) { g_ms += 10; if (i % 10 == 0) for (int k = 0; k < 8; k++) Serial.rx.push_back((uint8_t)(i * 7 + k * 31)); loop(); drain(); }   // wrong baud => garbage
    CHECK(BAUDS[baudIdx] != 57600 && Serial.baud == BAUDS[baudIdx], "garbage makes it try another baud rate");
    for (int i = 0; i < 12; i++) { pxAlive = true; run(1000); }
    CHECK(baudLocked, "locks once valid frames decode");
  }
  if (sc == "resync") {
    pxAlive = false; autoScan = false;
    for (int round = 0; round < 5; round++) {
      for (int k = 0; k < 13; k++) Serial.rx.push_back(0xFD);                       // junk that looks like frame starts
      mavlink_message_t m; mavlink_msg_attitude_pack_chan(1, 1, MAVLINK_COMM_2, &m, 0, .1f, .2f, .3f, 0, 0, 0); feed(m);
      mavlink_msg_vfr_hud_pack_chan(1, 1, MAVLINK_COMM_2, &m, 1, 2, 3, 4, 5, 6); feed(m);
      mavlink_msg_global_position_int_pack_chan(1, 1, MAVLINK_COMM_2, &m, 0, 1, 2, 3, 4, 0, 0, 0, 0); feed(m);
      Serial.rx.push_back(0x00); Serial.rx.push_back(0xFE); Serial.rx.push_back(0xAA);
      run(50);
    }
    for (int i = 0; i < 60; i++) { mavlink_message_t m; mavlink_msg_attitude_pack_chan(1, 1, MAVLINK_COMM_2, &m, i, 0, 0, 0, 0, 0, 0); feed(m); } run(200);
    printf("    decoded %u valid frames (75 sent, the last <=5 may still be waiting behind a bogus header)\n", (unsigned)validFrames);
    CHECK(validFrames >= 65, "frames still decode between junk bytes (resync works)");
  }
  if (sc == "json")     { run(5000); for (int i = 0; i < 24; i++) { mavlink_message_t m; mavlink_msg_attitude_pack_chan(1, 1, MAVLINK_COMM_2, &m, 0, 0, 0, 0, 0, 0, 0); feed(m); }
                          web.call("/status"); StaticJsonDocument<2048> d; CHECK(!deserializeJson(d, g_lastBody.c_str()), "status JSON is valid"); CHECK(g_lastBody.size() < 1699, "status JSON fits the buffer"); printf("    json bytes: %zu\n", g_lastBody.size()); }
  if (sc == "page")     { web.call("/"); CHECK(g_lastBody.find("PIXHAWK") != std::string::npos && g_lastBody.find("/status") != std::string::npos, "web page served"); }
  if (sc == "reset")    { run(5000); web.call("/reset"); CHECK(rxBytes == 0 && validFrames == 0 && hbCount == 0, "reset clears counters"); }
  printf(fails ? "RESULT: %d FAILED\n" : "RESULT: all passed\n", fails); return fails ? 1 : 0; }
