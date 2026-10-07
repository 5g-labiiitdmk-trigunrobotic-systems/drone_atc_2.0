#pragma once
#include "ESP8266WiFi.h"
struct WiFiManager { void setConfigPortalTimeout(int) {} void setConnectTimeout(int) {} bool autoConnect(const char*) { return true; } };
