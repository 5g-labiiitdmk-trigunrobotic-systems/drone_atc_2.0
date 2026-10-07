#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "ESP8266WebServer.h"
#include <ArduinoJson.h>
unsigned long g_ms = 0; HardwareSerial Serial, Serial1; WiFiClass WiFi; EspClass ESP; std::string g_lastBody;
#include "../../drone2_linktest/drone2_linktest.ino"

static void feed(const mavlink_message_t& m) { uint8_t b[300]; uint16_t n = mavlink_msg_to_send_buffer(b, &m); for (int i = 0; i < n; i++) Serial.rx.push_back(b[i]); }
static bool fcHears = true, fcReplyParam = true, fcReplyAck = true, loopback = false, fcAlive = true;

static void fcBeat() { mavlink_message_t m; mavlink_msg_heartbeat_pack(1, 1, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5, MAV_STATE_STANDBY); feed(m); }
static void drain() {
  static mavlink_message_t m; static mavlink_status_t st;
  for (uint8_t b : Serial.tx) {
    if (loopback) Serial.rx.push_back(b);
    if (mavlink_parse_char(MAVLINK_COMM_1, b, &m, &st) && fcHears) {
      if (m.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ && fcReplyParam) { mavlink_message_t r; mavlink_msg_param_value_pack(1, 1, &r, "SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 0); feed(r); }
      if (m.msgid == MAVLINK_MSG_ID_COMMAND_LONG && fcReplyAck) { mavlink_command_long_t c; mavlink_msg_command_long_decode(&m, &c); if (c.command == MAV_CMD_REQUEST_MESSAGE) { mavlink_message_t r; mavlink_msg_command_ack_pack(1, 1, &r, c.command, MAV_RESULT_ACCEPTED, 0, 0, 0, 0); feed(r); } }
    }
  }
  Serial.tx.clear();
}
static void run(unsigned ms) { static unsigned long lastBeat = 0; for (unsigned t = 0; t < ms; t += 10) { g_ms += 10; if (fcAlive && g_ms - lastBeat >= 1000) { lastBeat = g_ms; fcBeat(); } loop(); drain(); } }
static int fails = 0;
#define CHECK(c, msg) do { bool ok_ = (c); printf("  [%s] %s\n", ok_ ? "PASS" : "FAIL", msg); if (!ok_) fails++; } while (0)
static bool startsWith(const char* s, const char* p) { return strncmp(s, p, strlen(p)) == 0; }
int main(int argc, char** argv) {
  std::string sc = argv[1]; printf("scenario: %s\n", sc.c_str()); setup();
  if (sc == "no_data")   { fcAlive = false; fcHears = false; run(6000); CHECK(startsWith(verdict(), "NO DATA"), verdict()); }
  if (sc == "noise")     { fcAlive = false; for (int i = 0; i < 200; i++) Serial.rx.push_back((uint8_t)(i * 37 % 200)); fcHears = false; run(3000); CHECK(startsWith(verdict(), "Bytes arrive but no valid MAVLink") || startsWith(verdict(), "Receiving"), verdict()); }
  if (sc == "rx_only")   { fcHears = false; run(8000); CHECK(startsWith(verdict(), "ONE-WAY"), verdict()); CHECK(paramReplies + acks == 0 && fcHeartbeats > 3, "rx counted, no replies"); }
  if (sc == "two_way")   { run(5000); CHECK(startsWith(verdict(), "TWO-WAY OK"), verdict()); CHECK(paramReplies > 0 && acks > 0 && sysidValue == 1.0f, "param value and ack both received"); 
                           web.call("/status"); StaticJsonDocument<1024> d; auto e = deserializeJson(d, g_lastBody.c_str()); CHECK(!e && d["fcFound"] == true && d["acks"] > 0, "status JSON valid"); printf("  json size %zu\n", g_lastBody.size()); CHECK(g_lastBody.size() < 899, "json fits the buffer"); }
  if (sc == "ack_only")  { fcReplyParam = false; run(5000); CHECK(startsWith(verdict(), "TWO-WAY OK"), "a single reply type is enough"); }
  if (sc == "echo")      { fcAlive = false; fcHears = false; loopback = true; run(5000); CHECK(startsWith(verdict(), "ECHO OK"), verdict()); CHECK(echoFrames > 0, "own frames came back"); }
  if (sc == "text")      { run(1500); mavlink_message_t m; mavlink_msg_statustext_pack(1, 1, &m, MAV_SEVERITY_INFO, "PreArm: \"bad\" \\ x", 0, 0); feed(m); run(100); web.call("/status"); StaticJsonDocument<1024> d; CHECK(!deserializeJson(d, g_lastBody.c_str()), "quotes in FC text cannot break the JSON"); }
  if (sc == "reset")     { run(5000); web.call("/reset"); CHECK(rxBytes == 0 && paramReplies == 0 && !fcFound, "reset clears counters"); }
  if (sc == "replies_no_heartbeat") { fcAlive = false; run(500); mavlink_message_t m; mavlink_msg_param_value_pack(1, 1, &m, "SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 0); feed(m); run(100); CHECK(startsWith(verdict(), "TWO-WAY OK"), "a reply alone proves two-way"); }
  if (sc == "page")      { web.call("/"); CHECK(g_lastBody.find("TWO-WAY") != std::string::npos && g_lastBody.find("/status") != std::string::npos, "web page served"); }
  printf(fails ? "RESULT: %d FAILED\n" : "RESULT: all passed\n", fails); return fails ? 1 : 0; }

