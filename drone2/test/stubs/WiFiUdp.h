#pragma once
#include "ESP8266WiFi.h"
struct WiFiUDP { std::deque<std::pair<String,String>> pk; std::string cur; String from; bool started=false;
  void begin(int) { started = true; }
  int parsePacket() { if (pk.empty()) return 0; cur = pk.front().first.s; from = pk.front().second; pk.pop_front(); return cur.size(); }
  int read(char* b, size_t n) { size_t k = std::min(n, cur.size()); memcpy(b, cur.data(), k); return k; }
  IPAddress remoteIP() { return IPAddress{from}; } };
