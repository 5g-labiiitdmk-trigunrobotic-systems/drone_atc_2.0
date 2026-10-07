#pragma once
#include "Arduino.h"
#define WL_CONNECTED 3
struct IPAddress { String ip; String toString() const { return ip; } };
struct WiFiClient {};
struct WiFiClass { int status() { return connected ? WL_CONNECTED : 0; } bool connected = true;
  IPAddress localIP() { return IPAddress{String("10.0.0.99")}; } int RSSI() { return -55; }
  void persistent(bool) {} void setAutoReconnect(bool) {} };
extern WiFiClass WiFi;
struct EspClass { void restart() {} } ;
extern EspClass ESP;
