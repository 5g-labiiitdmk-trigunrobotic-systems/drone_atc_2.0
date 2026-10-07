#pragma once
#include "ESP8266WiFi.h"
#include <map>
enum { HTTP_GET, HTTP_POST };
extern std::string g_lastBody;
struct ESP8266WebServer { std::map<std::string, void (*)()> routes; ESP8266WebServer(int) {}
  void on(const char* p, void (*h)()) { routes[p] = h; }
  void on(const char* p, int, void (*h)()) { routes[p] = h; }
  void begin() {} void handleClient() {}
  void send(int, const char*, const String& b) { g_lastBody = b.s; }
  void send(int, const char*, const char* b) { g_lastBody = b; }
  void send_P(int, const char*, const char* b) { g_lastBody = b; }
  void call(const char* p) { routes[p](); } };
