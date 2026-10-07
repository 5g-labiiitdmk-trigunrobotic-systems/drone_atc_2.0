# pixhawk_esp_datacheck - is data flowing both ways?

A standalone ESP8266 sketch (no libraries, no MAVLink headers to copy) that shows two big lights:

* **PIXHAWK -> ESP**: bytes and CRC-valid MAVLink frames arrive from the Pixhawk.
* **ESP -> PIXHAWK**: the Pixhawk *answers* what the ESP sends (a parameter read and a command).
  A reply proves the Pixhawk received the ESP's data. It alternates MAVLink 1 and 2 frames.

Wiring: Pixhawk TELEM **TX -> D7**, **RX -> D8**, **GND -> GND**; power the NodeMCU over USB.
Pixhawk: `SERIALn_PROTOCOL = 2`, `SERIALn_BAUD = 57`, reboot.

1. Flash `pixhawk_esp_datacheck.ino` (board NodeMCU 1.0).
2. Join WiFi **PixhawkCheck** (password `12345678`), open **http://192.168.4.1**.
3. Read the lights and the sentence under them.

If bytes arrive but nothing decodes the page scans other baud rates by itself.
Echo test (checks the ESP alone): unplug the Pixhawk and jumper D7 to D8 -> "ECHO OK".
The page also shows the raw bytes, which message ids are heard, and the Pixhawk's sysid/mode/armed state.
Tests: `bash drone2/test/run_tests.sh` (cross-checks the encoder, parser and CRC table against the real MAVLink library).
