# drone2 firmware (ESP8266 MAVLink bridge)

`drone2.ino` bridges the flight controller (MAVLink on the hardware UART,
57600 baud) to the Flask ATC server over WiFi.

Libraries (Arduino Library Manager): **WiFiManager** (tzapu), **ArduinoJson**
(v6). MAVLink is not bundled: generate the C headers (common dialect) from
https://github.com/mavlink/c_library_v2 and copy them into `drone2/mavlink/`
so that `mavlink/common/mavlink.h` exists next to the sketch.

Server discovery is automatic (the server broadcasts `DRONE-ATC:<port>` on UDP 2390).

## Arming flow
1. Pilot arms; the bridge sees `armed` without ATC approval and sends a
   disarm immediately (the FC cannot be blocked before it arms; it is
   disarmed within ~200 ms).
2. The bridge POSTs `/drone/arm_request`; the dashboard shows an ARM REQUEST card.
3. Authority presses APPROVE ARM -> server sets the flight approval ->
   the bridge picks it up from `/pilot/status/<id>` (1 s poll).
4. Pilot arms again and is allowed. Approval resets on disarm.

`../drone2_servo/` is the separate ESP32 servo-trigger sketch (previously
mixed into this file).
