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

## ESP32 wiring (flight controller TELEM -> ESP32)
Board: any ESP32 dev board (select "ESP32 Dev Module"). The sketch builds for
ESP32 or ESP8266 automatically. On ESP32 the FC uses UART2 and USB stays free
for the Serial Monitor.

| FC TELEM pin | ESP32 pin | Note |
|---|---|---|
| TX  | GPIO16 (RX2) | FC transmit -> ESP receive |
| RX  | GPIO17 (TX2) | ESP transmit -> FC receive |
| GND | GND | common ground is required |
| 5V  | not used | power the ESP32 from its own 5V BEC / USB, not the FC port |

TX/RX cross over. FC telemetry pins are 3.3 V logic, which the ESP32 accepts directly.

ArduPilot settings for the TELEM port used (replace `n` with its SERIAL number):
`SERIALn_PROTOCOL = 2` (MAVLink 2), `SERIALn_BAUD = 57` (57600).
Reboot the FC after changing them. Libraries: WiFiManager (tzapu, ESP32 build), ArduinoJson v6.

## ESP8266 (NodeMCU) wiring
`ESP8266_SWAP_UART` at the top of `drone2.ino` selects the wiring.

**Recommended (`= 1`, swapped UART)** - FC no longer shares pins with USB:

| FC TELEM pin | NodeMCU pin |
|---|---|
| TX | **D7** (GPIO13, RX) |
| RX | **D8** (GPIO15, TX) |
| GND | GND |

Debug text goes to `Serial1` (TX only, **D4**/GPIO2) - read it with a USB-TTL adapter.
D8 must stay low at power-up; an FC RX input is high-impedance and normally fine,
but if the board will not boot with it connected, add a 10 kOhm resistor from D8 to GND.

**Alternative (`= 0`)** - FC TX -> RX (GPIO3), FC RX -> TX (GPIO1). These pins are also
USB: unplug the FC wires when flashing, and expect USB-serial/FC contention if USB is connected.
