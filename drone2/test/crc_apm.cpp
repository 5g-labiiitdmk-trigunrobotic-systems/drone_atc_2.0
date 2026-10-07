#include <cstdio>
#include "ardupilotmega/mavlink.h"
struct CE { int id, ex; };
int main() { CE tab[] = {
#include "build/crc_ids.inc"
}; int bad = 0; for (auto& e : tab) { const mavlink_msg_entry_t* me = mavlink_get_msg_entry(e.id); if (!me || me->crc_extra != e.ex) { printf("mismatch id %d: mine %d lib %d\n", e.id, e.ex, me ? me->crc_extra : -1); bad++; } } printf(bad ? "CRC: %d MISMATCH\n" : "CRC table matches the ArduPilot dialect (all %zu)\n", bad ? bad : sizeof(tab)/sizeof(tab[0])); return bad; }
