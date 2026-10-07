#pragma once
#include "ESP8266WiFi.h"
// returns status; fills resp. Implemented by the test harness.
extern int (*g_http)(const char* method, const std::string& url, const std::string& body, std::string& resp);
struct HTTPClient { std::string url, body, resp; int code = 0;
  void setReuse(bool) {} void setTimeout(int) {}
  bool begin(WiFiClient&, const String& u) { url = u.s; return true; }
  void addHeader(const char*, const char*) {}
  int POST(const String& b) { body = b.s; code = g_http("POST", url, body, resp); return code; }
  int GET() { code = g_http("GET", url, "", resp); return code; }
  String getString() { return String(resp); }
  void end() {} };
