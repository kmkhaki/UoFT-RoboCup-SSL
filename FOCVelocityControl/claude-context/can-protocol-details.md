---
name: can-protocol-details
description: "Wiring, protocol, and file locations for the working ESP32<->ESC1 CAN velocity link"
metadata: 
  node_type: memory
  type: reference
  originSessionId: d7b1a9f7-0d40-4807-bb7d-3d677b1b9f86
  modified: 2026-08-14T17:15:21.302Z
---

Technical details for the working CAN link described in [[can-motor-control-working]].

**Protocol:** single signed byte (int8_t, -128..127) as CAN ID `0x120`, 250 kbit/s, applied 1:1 as rad/s target on ESC1. ESP32 auto-repeats the last value every 150ms (heartbeat); ESC1 has a 500ms fail-safe timeout that zeros the target if nothing arrives in time. Deferred: widening to int16_t for the full -150..150 rad/s range (not yet requested/done).

**ESP32 side** (`/Users/khaleelkhaki/Documents/PlatformIO/Projects/esp32_can_test/src/main.cpp`): ESP-IDF native `driver/twai.h`, TX=GPIO5, RX=GPIO35. Non-blocking digit-filtered serial parsing (never use `Serial.readStringUntil` here — its default ~1s blocking timeout starves the heartbeat and causes real lag/glitches on the motor). PlatformIO env `esp32dev`, upload_port `/dev/cu.usbserial-110` (the original devkit died 2026-09-03 - CP2102 enumerated but no TX0/RX0 loopback even with the chip held in reset; replaced). Transceiver is an SN65HVD230 (3.3 V part) - GPIO5=D/TXD, GPIO35=R/RXD, VCC to 3V3 NOT 5V, since GPIO35 is input-only and rated 3.6 V max.

**ESC1 (G431) side, real motor sketch** (`/Users/khaleelkhaki/Documents/Arduino/CurrentControlTest/CurrentControlTest.ino`): interrupt-driven `meFDCAN` reception (`FDCAN1_IT0_IRQHandler` override), PA11=RX, PB9=TX, `CAN_SHDN_PIN=PC11` driven LOW to enable the onboard transceiver. This file is flashed via the user's own Arduino IDE, not via my PlatformIO/OpenOCD flash — `pio run -t upload` (stlink protocol) programs+verifies flash correctly but the sketch doesn't reliably start afterward (unresolved quirk); Arduino IDE's uploader works fine. I verify compilation only, via the scratch PlatformIO project at `/private/tmp/.../scratchpad/verify_current_control/` (mirrors the .ino, `lib/meFDCAN/` copied in, `lib_deps = askuric/Simple FOC`, board `disco_b_g431b_esc1`).

**meFDCAN library patches** (must be kept in sync in TWO places: `/Users/khaleelkhaki/Documents/PlatformIO/Projects/g431_can_test/lib/meFDCAN/meFDCAN.h` and `/Users/khaleelkhaki/Documents/Arduino/libraries/meFDCAN/meFDCAN.h`):
- Added `{170,250,40,14,2}` row to `brSettings[]` — G431 actually runs its FDCAN clock at 170MHz, not the 150MHz the stock library assumed.
- Removed all FDCAN2/FDCAN3-specific code (G431 only has one FDCAN peripheral; stock library targeted G473).
- Replaced `Serial1.println` debug calls with `Serial.println` (this board has no usable second UART).
- Must call `me_FDCAN_addFilter()` BEFORE `meFDCAN_init()` — the library's default global filter otherwise rejects everything.

**g431_can_test project** (`/Users/khaleelkhaki/Documents/PlatformIO/Projects/g431_can_test/`) is a standalone dry-run test firmware (same protocol, plus RTT debug output via a self-written `MiniRTT.h`/`.cpp`) — useful for testing CAN changes in isolation before touching the real motor sketch, since the real sketch has no RTT wired in (RTT was deliberately left out of CurrentControlTest.ino over control-loop-timing concerns).


**2026-09-04 - CAN now integrated into the NEW working FOC firmware.** The ESC1 no
longer runs `CurrentControlTest.ino`; it runs
`/Users/khaleelkhaki/Documents/PlatformIO/Projects/g431_foc/src/closedloop.cpp`
(env `closedloop`), which has the position-map commutation from
[[g431-closed-loop-position-map]] plus the same interrupt-driven meFDCAN
reception. `lib/meFDCAN/` is copied into that project. Received int8 is applied
1:1 as the velocity target; 500 ms fail-safe zeroes the target if the heartbeat
stops. Serial commands still work: `N0` disables CAN control for bench testing,
`N1` re-enables, `S` shows `rx`/`canvel`/`age_ms`.

Flashing the ESC1 from PlatformIO/OpenOCD worked reliably throughout the
2026-09-03/04 sessions, contradicting the older note above about sketches not
starting after a `pio run -t upload` - that quirk did not reappear.

Verified end to end 2026-09-04: commanded 20/45/-30/0 rad/s over the bus, all
tracked; heartbeat cut mid-run stopped the motor within the 500 ms window with
the shaft angle frozen (not coasting).
