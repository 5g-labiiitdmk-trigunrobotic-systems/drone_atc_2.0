# drone2 - ESP8266 bridge firmware (v10)

`drone2.ino` connects an ArduPilot flight controller (FC) to the Flask ATC server.
Board: NodeMCU / any ESP8266. (`../drone2_servo/` is a separate ESP32 servo sketch.)

## Install
1. Arduino IDE -> board **NodeMCU 1.0 (ESP-12E)** (ESP8266 core).
2. Library Manager: **WiFiManager** (tzapu) and **ArduinoJson 6.x**.
3. MAVLink: copy the contents of <https://github.com/mavlink/c_library_v2> into
   `drone2/mavlink/` so that `drone2/mavlink/common/mavlink.h` exists.
4. Edit `DRONE_ID` at the top of `drone2.ino` (must equal the ID registered in the pilot portal, case-sensitive).
5. Flash. First boot: join the `DroneSetup` WiFi hotspot once and enter your WiFi.
   The server is found automatically (UDP broadcast); `SERVER_FALLBACK_URL` is optional.

## Wiring (default `USE_SWAPPED_UART 1`)
| FC TELEM pin | NodeMCU |
|---|---|
| TX | **D7** (GPIO13) |
| RX | **D8** (GPIO15) |
| GND | GND |

Power the NodeMCU separately (not from the FC). Debug text is on **D4** (`Serial1`, 115200,
transmit only - use a USB-TTL adapter). D8 must be low at power-up; if the board will not
boot with the FC connected add a 10 kOhm resistor D8 -> GND.
Set `USE_SWAPPED_UART 0` to use the RX/TX pins instead (shared with USB; unplug the FC to flash).

FC settings (ArduPilot, for the TELEM port used): `SERIALn_PROTOCOL = 2`, `SERIALn_BAUD = 57`; reboot the FC.

## What the firmware does
- Telemetry to `/update` every 1 s, and immediately when the arm state changes
  (includes `fw_approved`, `fw_arm_allowed`, `fw_lock_hits` for diagnosis; browse `/get_fleet`).
- **Arm authority.** The ESP can only react after the FC arms. An arm it did not allow
  (no approval, no authority arm) is force-disarmed (retried every 250 ms until the FC
  disarms) and reported through `/drone/arm_request`.
  - It never force-disarms something it did not see go from disarmed to armed (an ESP
    reset in flight must not drop the drone) and never while clearly airborne (> 2 m): it alerts instead.
  - Approval is checked every 0.3 s while disarmed, spent when the flight ends, ignored for 3 s
    afterwards, and dropped if the server is unreachable for 15 s.
- Commands from `/drone/poll_commands`: `rtl`, `land`, `hover`, `kill` (forced disarm), `move`,
  `arm` (pilot, needs approval), `auth_arm` (authority override), `disarm` (normal disarm).
- Failsafes: velocity stop 0.5 s after the last move; LAND after 5 consecutive low-battery readings.
- The ESP exposes only a read-only `/test` status page - no unauthenticated control endpoints.

## Making arming truly permission-gated (one-time FC setup)
A listener cannot stop an FC from arming. Make the ESP the **only** thing that can arm it:
`ARMING_RUDDER = 0`; no `RCx_OPTION` set to 41, 153 or 154; no other ground station able to arm.
Then: pilot portal REQUEST FLIGHT -> authority approves -> pilot portal ARM MOTORS ->
server `/pilot/arm/<id>` (403 unless approved) -> ESP sends the arm command.
The disarm lock stays as a backup. A hardware interlock (ESC power relay) is the only absolute guarantee.

## Tests
`bash drone2/test/run_tests.sh` compiles the real sketch on a PC (g++, git) against the real MAVLink
and ArduinoJson headers, with the Arduino/ESP8266 APIs stubbed, and runs 13 scenarios against a
simulated flight controller and server: discovery, telemetry format, unauthorized arm, disarm retry,
STATUSTEXT fast path, the full approval flow, armed-at-boot, airborne guard, every command,
sysid/compid learning, GCS heartbeat rejection, stale approval, and battery failsafe.
It does not test the real radio, UART timing or the flight controller itself - bench-test with props off.
