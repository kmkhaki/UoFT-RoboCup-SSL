---
name: can-motor-control-working
description: ESP32-to-B-G431B-ESC1 CAN velocity control is fully working end-to-end (confirmed on real motor)
metadata: 
  node_type: memory
  type: project
  originSessionId: d7b1a9f7-0d40-4807-bb7d-3d677b1b9f86
  modified: 2026-08-14T17:15:05.594Z
---

As of 2026-08-14, ESP32 → B-G431B-ESC1 (ESC1) CAN velocity control works end-to-end on the real motor: typed values on the ESP32 (range -100..100 tested, full range is -128..127) correctly drive the motor via SimpleFOC velocity control on ESC1, confirmed by the user watching the motor respond through a full -100→100 sweep.

Root cause of the ~4-day CAN failure that preceded this: **SimpleCanLib itself was broken**, not the wiring/hardware (which had been exhaustively verified). The fix was abandoning SimpleCanLib entirely in favor of:
- ESP32 side: ESP-IDF's native `driver/twai.h` (no third-party Arduino CAN library).
- ESC1 (G431) side: `meFDCAN` (mackelec/meFDCAN), patched locally for G431 (single-FDCAN-peripheral) compatibility — see [[can-protocol-details]] for the exact patches and pin/protocol details.

See [[can-protocol-details]] for wiring, protocol byte layout, and library file locations.

A separate, later crisis (ESP32 reset-looping every ~10-20ms) was root-caused by the user to a physically damaged/shorting CAN wire — replacing the wire fixed it. Worth remembering as a first-check item if erratic resets recur on this project.

**Why this matters:** if CAN communication on this project ever breaks again, do NOT re-attempt SimpleCanLib or re-litigate wiring/bitrate from scratch — the working combination (native TWAI + patched meFDCAN) is proven, and physical wire damage is a real recurring failure mode on this bench setup worth checking early.
