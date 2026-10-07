#!/usr/bin/env bash
# Host-side tests for drone2.ino: compiles the real sketch against the real MAVLink
# and ArduinoJson headers (Arduino/ESP8266 APIs are stubbed) and runs scenarios
# against a fake flight controller and a fake ATC server. Needs g++ and git.
set -e
cd "$(dirname "$0")"
EXT=build/ext; mkdir -p "$EXT"
[ -d "$EXT/c_library_v2" ] || git clone -q --depth 1 https://github.com/mavlink/c_library_v2.git "$EXT/c_library_v2"
[ -d "$EXT/ArduinoJson" ]  || git clone -q --depth 1 --branch v6.21.5 https://github.com/bblanchon/ArduinoJson.git "$EXT/ArduinoJson"
ln -sfn "$(pwd)/$EXT/c_library_v2" stubs/mavlink
g++ -std=c++17 -x c++ -DESP8266 -Wall -Wno-unused-function -Wno-address-of-packed-member \
    -Istubs -I"$EXT/ArduinoJson/src" -o build/t harness.cpp
status=0
for sc in discovery telemetry unauth_arm disarm_retry statustext approved_flow boot_armed airborne commands targets gcs_ignored stale_approval battery; do
  build/t "$sc" || status=1
done
g++ -std=c++17 -x c++ -DESP8266 -Wall -Wno-unused-function -Wno-address-of-packed-member \
    -Istubs -I"$EXT/ArduinoJson/src" -o build/tl linktest_harness.cpp
for sc in no_data noise rx_only two_way ack_only echo text reset page replies_no_heartbeat; do
  build/tl "$sc" || status=1
done
exit $status
