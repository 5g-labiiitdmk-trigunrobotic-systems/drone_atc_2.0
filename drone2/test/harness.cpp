#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "WiFiUdp.h"
#include "ESP8266HTTPClient.h"
#include <map>
#include <functional>
#include <algorithm>
unsigned long g_ms = 0;
HardwareSerial Serial, Serial1;
WiFiClass WiFi; EspClass ESP;
int (*g_http)(const char*, const std::string&, const std::string&, std::string&);
#include "../drone2.ino"

// ------------------------------------------------------------------ fake server
struct Srv {
  bool approved = false; bool reachable = true;
  std::vector<std::string> posts;      // "path body"
  std::string cmds = "[]";
  int telemetry = 0, armReq = 0, statusPolls = 0;
  std::string lastTel;
} srv;
static int fakeHttp(const char* m, const std::string& url, const std::string& body, std::string& resp) {
  if (!srv.reachable) return -1;
  std::string path = url.substr(url.find('/', 8));
  srv.posts.push_back(std::string(m) + " " + path + " " + body);
  if (path == "/update") { srv.telemetry++; srv.lastTel = body; return 200; }
  if (path == "/drone/arm_request") { srv.armReq++; return 200; }
  if (path.rfind("/pilot/status/", 0) == 0) { srv.statusPolls++; resp = std::string("{\"approved\":") + (srv.approved ? "true" : "false") + ",\"status\":\"x\"}"; return 200; }
  if (path.rfind("/drone/poll_commands/", 0) == 0) { resp = srv.cmds; srv.cmds = "[]"; return 200; }
  return 404;
}

// ------------------------------------------------------------------ fake FC
struct FC { bool armed = false; bool obeys = true; uint8_t sys = 1, comp = 1; uint8_t type = MAV_TYPE_QUADROTOR; uint32_t mode = 0; } fc;
static void feed(const mavlink_message_t& m) { uint8_t b[300]; uint16_t n = mavlink_msg_to_send_buffer(b, &m); for (int i = 0; i < n; i++) Serial.rx.push_back(b[i]); }
static void fcHeartbeat() { mavlink_message_t m; mavlink_msg_heartbeat_pack(fc.sys, fc.comp, &m, fc.type, MAV_AUTOPILOT_ARDUPILOTMEGA,
  (fc.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, fc.mode, MAV_STATE_ACTIVE); feed(m); }
static float simAlt = -1; static unsigned simMv = 0;
static void fcPosRaw(float altM) { mavlink_message_t m; mavlink_msg_global_position_int_pack(fc.sys, fc.comp, &m, 0, 285548620, 770446360, (int32_t)(altM*1000), (int32_t)(altM*1000), 0,0,0,0); feed(m); }
static void fcBattRaw(unsigned mv) { mavlink_message_t m; mavlink_msg_sys_status_pack(fc.sys, fc.comp, &m, 0,0,0,0, mv, -1, -1, 0,0,0,0,0,0, 0,0,0); feed(m); }
static void fcPos(float a) { simAlt = a; fcPosRaw(a); }
static void fcBatt(unsigned mv) { simMv = mv; fcBattRaw(mv); }
static void fcText(const char* t) { mavlink_message_t m; mavlink_msg_statustext_pack(fc.sys, fc.comp, &m, MAV_SEVERITY_INFO, t, 0, 0); feed(m); }
static void fcAck(uint16_t cmd) { mavlink_message_t m; mavlink_msg_command_ack_pack(fc.sys, fc.comp, &m, cmd, MAV_RESULT_ACCEPTED, 0,0,0,0); feed(m); }

// ------------------------------------------------------------------ decode what the ESP sent to the FC
struct Sent { uint8_t id; uint16_t cmd=0; float p1=0,p2=0; uint8_t tsys=0,tcomp=0; uint32_t custom=0; float vx=0,vy=0,vz=0; };
static std::vector<Sent> sent;
static void drainTx() {
  static mavlink_message_t m; static mavlink_status_t st;
  for (uint8_t b : Serial.tx) if (mavlink_parse_char(MAVLINK_COMM_1, b, &m, &st)) {
    Sent s; s.id = m.msgid;
    if (m.msgid == MAVLINK_MSG_ID_COMMAND_LONG) { mavlink_command_long_t c; mavlink_msg_command_long_decode(&m, &c); s.cmd=c.command; s.p1=c.param1; s.p2=c.param2; s.tsys=c.target_system; s.tcomp=c.target_component;
      // the FC obeys disarm/arm instantly
      if (c.command == MAV_CMD_COMPONENT_ARM_DISARM && fc.obeys) { fc.armed = c.param1 == 1; fcAck(c.command); } }
    if (m.msgid == MAVLINK_MSG_ID_SET_MODE) { mavlink_set_mode_t c; mavlink_msg_set_mode_decode(&m, &c); s.custom=c.custom_mode; s.tsys=c.target_system; }
    if (m.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) { mavlink_set_position_target_local_ned_t c; mavlink_msg_set_position_target_local_ned_decode(&m, &c); s.vx=c.vx; s.vy=c.vy; s.vz=c.vz; }
    sent.push_back(s);
  }
  Serial.tx.clear();
}
static void run(unsigned ms, unsigned hbEveryMs = 1000, unsigned step = 5) {
  static unsigned long lastHb = 0;
  for (unsigned t = 0; t < ms; t += step) { g_ms += step; if (hbEveryMs && g_ms - lastHb >= hbEveryMs) { lastHb = g_ms; fcHeartbeat(); if (simAlt >= 0) fcPosRaw(simAlt); if (simMv) fcBattRaw(simMv); } loop(); drainTx(); }
}
static int countCmd(float p1, float p2) { int n=0; for (auto& s: sent) if (s.id==MAVLINK_MSG_ID_COMMAND_LONG && s.cmd==MAV_CMD_COMPONENT_ARM_DISARM && s.p1==p1 && s.p2==p2) n++; return n; }
static int countMode(uint32_t c) { int n=0; for (auto& s: sent) if (s.id==MAVLINK_MSG_ID_SET_MODE && s.custom==c) n++; return n; }

static int fails = 0;
#define CHECK(c, msg) do { bool ok_ = (c); printf("  [%s] %s\n", ok_ ? "PASS" : "FAIL", msg); if (!ok_) fails++; } while (0)

static void boot() {
  g_http = fakeHttp; setup();
  udp.pk.push_back({String("DRONE-ATC:5000"), String("10.0.0.5")});
  run(50);
  srv.posts.clear(); sent.clear();
}
static bool telHas(const char* needle) { return srv.lastTel.find(needle) != std::string::npos; }

int main(int argc, char** argv) {
  std::string sc = argc > 1 ? argv[1] : "all";
  printf("scenario: %s\n", sc.c_str());

  if (sc == "discovery") {
    g_http = fakeHttp; setup();
    CHECK(!serverKnown, "no server before broadcast");
    udp.pk.push_back({String("DRONE-ATC:5000"), String("10.0.0.5")}); run(20);
    CHECK(serverBase == "http://10.0.0.5:5000", "server discovered from UDP");
    udp.pk.push_back({String("DRONE-ATC:5000"), String("10.0.0.8")}); run(20);
    CHECK(serverBase == "http://10.0.0.8:5000", "follows the server to a new IP");
  }
  if (sc == "telemetry") {
    boot(); fc.mode = 5; fcPos(1.5f); fcBatt(22200); run(3200);
    CHECK(srv.telemetry >= 3, "telemetry ~1 Hz");
    CHECK(telHas("\"id\":\"Drone-1\"") && telHas("\"armed\":false") && telHas("\"mode\":\"LOITER\"") && telHas("\"mav_hb\":true"), "telemetry fields (id/armed/mode/hb)");
    CHECK(telHas("\"lat\":28.554862") && telHas("\"lon\":77.044636"), "6-decimal lat/lon preserved");
    CHECK(telHas("\"batt\":22.20"), "battery volts");
  }
  if (sc == "unauth_arm") {
    boot(); run(1500); fcPos(0.1f);
    fc.armed = true; fcHeartbeat(); size_t before = sent.size(); run(10, 0);
    int disarms = countCmd(0, 21196);
    CHECK(disarms >= 1, "force-disarm sent immediately on unauthorized arm");
    CHECK(!fc.armed, "FC ended up disarmed");
    run(2000); 
    CHECK(srv.armReq == 1, "exactly one arm request (rate-limited, no loop)");
    CHECK(countCmd(0, 21196) <= 2, "no endless disarm spam once FC disarmed");
    CHECK(telHas("fw_lock_hits"), "diagnostics in telemetry");
    // first HTTP after the arm must not precede the disarm
    (void)before;
  }
  if (sc == "disarm_retry") {
    boot(); run(1500); fc.obeys = false; fc.armed = true; fcHeartbeat(); run(1200);
    int n = countCmd(0, 21196);
    CHECK(n >= 4 && n <= 6, "retries disarm every 250 ms while FC ignores it");
  }
  if (sc == "statustext") {
    boot(); run(1500, 1000); fc.armed = true; fcText("Arming motors"); run(10, 0);
    CHECK(countCmd(0, 21196) >= 1, "reacts to 'Arming motors' before the heartbeat");
  }
  if (sc == "approved_flow") {
    boot(); run(1500); srv.approved = true; run(700);
    CHECK(telHas("\"fw_approved\":true") || approved, "approval picked up within ~0.7 s");
    fc.armed = true; fcHeartbeat(); run(1500);
    CHECK(countCmd(0, 21196) == 0, "approved arm is NOT disarmed");
    CHECK(fc.armed && armAllowed, "stays armed, arm latched allowed");
    srv.approved = false; run(2000);                                  // server revoked mid flight
    CHECK(fc.armed && countCmd(0, 21196) == 0, "revoke mid-flight does not disarm (latched)");
    fc.armed = false; fcHeartbeat(); run(20, 0); 
    CHECK(!approved, "approval spent after the flight");
    srv.approved = true; run(2000);                                   // stale server state right after flight
    CHECK(!approved, "stale server approval ignored during holdoff");
    srv.approved = false; run(4000);
    fc.armed = true; fcHeartbeat(); run(30, 0);
    CHECK(countCmd(0, 21196) >= 1, "next arm is blocked again");
  }
  if (sc == "boot_armed") {
    g_http = fakeHttp; fc.armed = true; setup(); udp.pk.push_back({String("DRONE-ATC:5000"), String("10.0.0.5")});
    run(3000);
    CHECK(countCmd(0, 21196) == 0, "armed-at-boot is adopted, never force-disarmed");
    CHECK(srv.armReq >= 1, "authority is still notified");
  }
  if (sc == "airborne") {
    boot(); run(1500); fcPos(12.0f); run(100); fc.armed = true; fcHeartbeat(); run(1500);
    CHECK(countCmd(0, 21196) == 0, "unauthorized arm while airborne: alert only, no force-disarm");
    CHECK(srv.armReq >= 1, "authority notified");
  }
  if (sc == "commands") {
    boot(); run(1500);
    srv.cmds = "[{\"cmd\":\"arm\"}]"; run(1200);
    CHECK(countCmd(1, 0) == 0, "pilot arm ignored when not approved");
    srv.approved = true; run(700); srv.cmds = "[{\"cmd\":\"arm\"}]"; run(1200);
    CHECK(countCmd(1, 0) >= 1, "pilot arm sent when approved");
    sent.clear(); fc.armed = false; fcHeartbeat(); run(100); srv.approved = false; run(4000);
    srv.cmds = "[{\"cmd\":\"auth_arm\"}]"; run(1200);
    CHECK(countCmd(1, 0) >= 1, "authority arm sent without approval");
    CHECK(fc.armed && countCmd(0, 21196) == 0, "authority arm is not disarmed by the lock");
    srv.cmds = "[{\"cmd\":\"disarm\"}]"; run(1200);
    CHECK(countCmd(0, 0) >= 1, "authority disarm is a normal (non-forced) disarm");
    srv.cmds = "[{\"cmd\":\"kill\"}]"; run(1200);
    CHECK(countCmd(0, 21196) >= 1, "kill is a forced disarm");
    srv.cmds = "[{\"cmd\":\"rtl\"},{\"cmd\":\"land\"},{\"cmd\":\"hover\"}]"; run(1200);
    CHECK(countMode(MODE_RTL) >= 1 && countMode(MODE_LAND) >= 1 && countMode(MODE_LOITER) >= 1, "rtl/land/hover set the right modes");
    sent.clear(); srv.cmds = "[{\"cmd\":\"move\",\"vx\":1.5,\"vy\":0.0,\"vz\":-0.5}]"; run(600);
    bool vel = false, zero = false; for (auto& s : sent) if (s.id == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) { if (s.vx == 1.5f && s.vz == -0.5f) vel = true; if (s.vx == 0 && s.vy == 0 && s.vz == 0) zero = true; }
    CHECK(countMode(MODE_GUIDED) >= 1 && vel, "move switches to GUIDED and sends the velocity");
    run(600); for (auto& s : sent) if (s.id == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED && s.vx == 0 && s.vz == 0) zero = true;
    CHECK(zero, "velocity zeroed 500 ms after the last move");
  }
  if (sc == "targets") {
    boot(); fc.sys = 7; fc.comp = 3; run(1500);
    CHECK(fcLearned && fcSys == 7 && fcComp == 3, "learns the autopilot's sysid/compid");
    fc.armed = true; fcHeartbeat(); run(30, 0);
    bool ok = false; for (auto& s : sent) if (s.id == MAVLINK_MSG_ID_COMMAND_LONG && s.tsys == 7 && s.tcomp == 3) ok = true;
    CHECK(ok, "commands are addressed to the learned sysid/compid");
  }
  if (sc == "gcs_ignored") {
    boot(); fc.type = MAV_TYPE_GCS; run(2500);
    CHECK(!hbOK, "a GCS heartbeat is not mistaken for the autopilot");
  }
  if (sc == "stale_approval") {
    boot(); run(1500); srv.approved = true; run(700); CHECK(approved, "approved");
    srv.reachable = false; run(16500);
    CHECK(!approved, "approval dropped after 15 s without the server");
  }
  if (sc == "battery") {
    boot(); run(1500); srv.approved = true; run(700); fc.armed = true; fcHeartbeat(); run(50, 0);
    fcBatt(65535); for (int i = 0; i < 8; i++) { run(100, 0); fcBatt(65535); } run(100, 0);
    CHECK(countMode(MODE_LAND) == 0, "'unknown voltage' (65535 mV) never triggers a landing");
    for (int i = 0; i < 7; i++) { fcBatt(13900); run(100, 0); }
    CHECK(countMode(MODE_LAND) == 1, "5 consecutive low readings -> exactly one LAND");
  }
  if (getenv("SHOWTEL")) printf("lastTel=%s\narmReq=%d posts=%zu\n", srv.lastTel.c_str(), srv.armReq, srv.posts.size());
  printf(fails ? "RESULT: %d FAILED\n" : "RESULT: all passed\n", fails);
  return fails ? 1 : 0;
}
