#pragma once
#define PROGMEM
#include <string>
#include <vector>
#include <deque>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <cstdlib>
extern unsigned long g_ms;
inline unsigned long millis() { return g_ms; }
inline void delay(unsigned long ms) { g_ms += ms; }
inline void yield() {}
class String {
 public:
  std::string s;
  String() {}
  String(const char* c) : s(c ? c : "") {}
  String(const std::string& x) : s(x) {}
  String(int v) : s(std::to_string(v)) {}
  String(unsigned v) : s(std::to_string(v)) {}
  String(long v) : s(std::to_string(v)) {}
  String(unsigned long v) : s(std::to_string(v)) {}
  String(double v, int d) { char b[64]; snprintf(b, sizeof b, "%.*f", d, v); s = b; }
  const char* c_str() const { return s.c_str(); }
  long toInt() const { return atol(s.c_str()); }
  unsigned length() const { return s.size(); }
  void trim() { while (!s.empty() && isspace((unsigned char)s.back())) s.pop_back(); size_t i = 0; while (i < s.size() && isspace((unsigned char)s[i])) i++; s = s.substr(i); }
  String& operator+=(const String& o) { s += o.s; return *this; }
  bool operator==(const String& o) const { return s == o.s; }
  bool operator!=(const String& o) const { return s != o.s; }
  bool operator==(const char* o) const { return s == o; }
  bool operator!=(const char* o) const { return s != o; }
};
inline String operator+(const String& a, const String& b) { return String(a.s + b.s); }
inline String operator+(const String& a, const char* b) { return String(a.s + b); }
inline String operator+(const char* a, const String& b) { return String(std::string(a) + b.s); }
struct HardwareSerial {
  std::deque<uint8_t> rx; std::vector<uint8_t> tx; bool swapped = false; size_t rxSize = 0;
  void setRxBufferSize(size_t n) { rxSize = n; }
  void begin(unsigned long) {}
  unsigned long baud = 0; void updateBaudRate(unsigned long b) { baud = b; }
  void swap() { swapped = true; }
  int available() { return (int)rx.size(); }
  int read() { if (rx.empty()) return -1; uint8_t b = rx.front(); rx.pop_front(); return b; }
  size_t write(const uint8_t* b, size_t n) { tx.insert(tx.end(), b, b + n); return n; }
  void println(const char* c) { if (getenv("VERBOSE")) ::printf("  [dbg] %s\n", c); }
  void println(const String& c) { println(c.c_str()); }
  void printf(const char* f, ...) { if (!getenv("VERBOSE")) return; va_list a; va_start(a, f); ::printf("  [dbg] "); ::vprintf(f, a); va_end(a); }
};
extern HardwareSerial Serial, Serial1;
