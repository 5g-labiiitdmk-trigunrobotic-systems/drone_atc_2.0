#pragma once
#include "ESP8266WiFi.h"
enum { HTTP_GET, HTTP_POST };
struct ESP8266WebServer { ESP8266WebServer(int) {} void on(const char*, int, void (*)()) {} void begin() {} void handleClient() {}
  void send(int, const char*, const String&) {} };
