// Host test for pixhawk_esp_simple_check.ino (mocks millis/Serial/SoftwareSerial; real MAVLink library for the "Pixhawk").
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <cstdlib>
#include <string>
#include <vector>
#include <deque>
unsigned long g_ms = 0;
unsigned long millis() { return g_ms; }
std::string g_out;
struct MockSerial {
  void begin(unsigned long) {}
  void println(const char* s) { g_out += s; g_out += "\n"; }
  void printf(const char* f, ...) { char b[400]; va_list a; va_start(a, f); vsnprintf(b, sizeof b, f, a); va_end(a); g_out += b; }
} Serial;
struct SoftwareSerial {
  std::deque<uint8_t> rx; std::vector<uint8_t> tx;
  SoftwareSerial(int, int) {}
  void begin(unsigned long) {}
  int available() { return (int)rx.size(); }
  int read() { uint8_t b = rx.front(); rx.pop_front(); return b; }
  size_t write(const uint8_t* b, size_t n) { tx.insert(tx.end(), b, b + n); return n; }
};
#define SoftwareSerial_h_included
#include "../../pixhawk_esp_simple_check/pixhawk_esp_simple_check.ino"
#include "mavlink/common/mavlink.h"

static bool pxHears = true, pxAlive = true, loopback = false, pxV1 = false;
static std::vector<mavlink_message_t> got;
static void feedMsg(const mavlink_message_t& m) {
  mavlink_status_t* cs = mavlink_get_channel_status(MAVLINK_COMM_2);
  if (pxV1) cs->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1; else cs->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  uint8_t b[300]; mavlink_message_t c = m; uint16_t n = mavlink_msg_to_send_buffer(b, &c);
  for (int i = 0; i < n; i++) fc.rx.push_back(b[i]);
}
static void pixBeat() { mavlink_message_t m; mavlink_msg_heartbeat_pack_chan(1, 1, MAVLINK_COMM_2, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY); feedMsg(m); }
static void drain() {
  static mavlink_message_t m; static mavlink_status_t st;
  for (uint8_t b : fc.tx) {
    if (loopback) fc.rx.push_back(b);
    if (mavlink_parse_char(MAVLINK_COMM_1, b, &m, &st)) {
      got.push_back(m);
      if (pxHears && m.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) { mavlink_message_t r; mavlink_msg_param_value_pack_chan(1, 1, MAVLINK_COMM_2, &r, "SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 0); feedMsg(r); }
    }
  }
  fc.tx.clear();
}
static void run(unsigned ms) { static unsigned long lb = 0; for (unsigned t = 0; t < ms; t += 10) { g_ms += 10; if (pxAlive && g_ms - lb >= 1000) { lb = g_ms; pixBeat(); } loop(); drain(); } }
static std::string reportText() { size_t p = g_out.rfind("=====  Pixhawk"); (void)p; size_t q = g_out.rfind("\n===== Pixhawk"); return q == std::string::npos ? "" : g_out.substr(q); }
static int fails = 0;
#define CHECK(c, msg) do { bool ok_ = (c); printf("  [%s] %s\n", ok_ ? "PASS" : "FAIL", msg); if (!ok_) fails++; } while (0)
static bool has(const std::string& s, const char* t) { return s.find(t) != std::string::npos; }

int main(int argc, char** argv) {
  std::string sc = argv[1]; printf("scenario: %s\n", sc.c_str()); setup();
  if (sc == "all_ok")   { run(8000); auto r = reportText(); CHECK(has(r, "ALL OK"), "prints ALL OK when both directions work"); CHECK(has(r, "Pixhawk -> ESP : OK") && has(r, "ESP -> Pixhawk : OK"), "both lines say OK"); }
  if (sc == "all_ok_mavlink1") { pxV1 = true; run(8000); CHECK(has(reportText(), "ALL OK"), "ALL OK also when the Pixhawk uses MAVLink 1"); }
  if (sc == "rx_only")  { pxHears = false; run(10000); auto r = reportText(); CHECK(!has(r, "ALL OK"), "no ALL OK"); CHECK(has(r, "Pixhawk -> ESP : OK") && has(r, "ESP -> Pixhawk : NOT OK") && has(r, "ESP -> Pixhawk does NOT"), "says exactly: ESP -> Pixhawk is the broken direction"); }
  if (sc == "no_data")  { pxAlive = false; pxHears = false; run(10000); auto r = reportText(); CHECK(!has(r, "ALL OK") && has(r, "NO data from the Pixhawk"), "no ALL OK, says there is no data"); }
  if (sc == "garbage")  { pxAlive = false; pxHears = false; for (int i = 0; i < 9000; i += 10) { g_ms += 10; for (int k = 0; k < 3; k++) fc.rx.push_back((uint8_t)(i * 13 + k * 71)); loop(); drain(); } auto r = reportText(); CHECK(!has(r, "ALL OK") && has(r, "not valid MAVLink"), "garbage bytes: no ALL OK, points at baud/protocol"); }
  if (sc == "echo")     { pxAlive = false; pxHears = false; loopback = true; run(10000); auto r = reportText(); CHECK(!has(r, "ALL OK") && has(r, "hearing itself"), "TX wired to RX: no ALL OK, says so"); }
  if (sc == "requests") {
    pxAlive = false; pxHears = false; run(3200); bool v1 = false, v2 = false, ok = true; int n = 0;
    for (auto& m : got) { if (m.msgid != MAVLINK_MSG_ID_PARAM_REQUEST_READ) continue; n++; if (m.magic == 0xFE) v1 = true; if (m.magic == 0xFD) v2 = true;
      mavlink_param_request_read_t p; mavlink_msg_param_request_read_decode(&m, &p); if (p.target_system != 1 || p.target_component != 1 || p.param_index != -1 || strncmp(p.param_id, "SYSID_THISMAV", 13)) ok = false; }
    CHECK(n >= 2 && v1 && v2, "requests pass the REAL library's CRC check, in MAVLink 1 and 2"); CHECK(ok, "target 1/1, index -1, name SYSID_THISMAV");
  }
  if (sc == "flaky_start") { run(2000); pxAlive = true; pxHears = true; run(8000); CHECK(has(reportText(), "ALL OK"), "becomes ALL OK once the link works"); }
  printf(fails ? "RESULT: %d FAILED\n" : "RESULT: all passed\n", fails); return fails ? 1 : 0; }
