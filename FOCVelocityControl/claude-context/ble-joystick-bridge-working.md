---
name: ble-joystick-bridge-working
description: Joystick-on-Feather -> BLE -> ESP32 -> CAN -> ESC1 motor control chain confirmed working end-to-end
metadata: 
  node_type: memory
  type: project
  originSessionId: d7b1a9f7-0d40-4807-bb7d-3d677b1b9f86
  modified: 2026-08-17T14:39:08.773Z
---

As of 2026-08-17, the full remote-control chain works end-to-end and drives the real motor: a 2-axis joystick wired to an Adafruit Feather 32u4 Bluefruit LE reads X/Y + button, maps to -128..127, streams it over BLE to the ESP32 (which forwards it over CAN to ESC1, same protocol as [[can-protocol-details]]). Confirmed live by the user moving the joystick and seeing the motor respond.

**Role split (important, non-obvious):** the Feather's onboard nRF51822 BLE module is **peripheral-only** — Adafruit's own firmware/SoftDevice for nRF51 hardware has no Central-mode support at all (only their newer nRF52 boards do). So the Feather advertises and the ESP32 scans+connects, the reverse of a naive BLE server/client split. Conveniently the Nordic UART Service UUIDs (`6E400001/02/03-B5A3-F393-E0A9-E50E24DCCA9E`) are exactly what the Feather's module natively advertises in UART "DATA mode" — no custom GATT needed on that side.

**Dead Bluefruit module lesson:** the first Feather board hit a hard hang on every real AT command after `ble.begin()` (factory reset, echo, isConnected — all hung), with zero blue LED activity ever. Root-caused by directly probing the IRQ pin (7) with `digitalRead()` in a tight loop, bypassing the library entirely — it read permanently HIGH (should idle LOW, only pulse HIGH when a response is ready), meaning the module wasn't actually alive. Swapping to a second physical board fixed it immediately (IRQ read LOW at idle, AT commands completed normally). If this happens again on this project: probe the IRQ pin directly before suspecting code/timing — a stuck-HIGH IRQ line with no LED activity means dead hardware, not a bug to fix in software.

**Current files:**
- `/Users/khaleelkhaki/Documents/PlatformIO/Projects/feather32u4_joystick_ble/` — Feather firmware. Joystick pins: VRx→A0, VRy→A1, SW→12 (GND/3V power). Reserved BLE-module pins on this board: 4 (RST), 7 (IRQ), 8 (CS), plus hardware SPI (SCK/MOSI/MISO). `ble.factoryReset()` is skipped entirely (hangs on this hardware and isn't needed since no custom GATT service is used) — code goes `ble.begin()` → `delay(500)` settle → straight into use.
- `/Users/khaleelkhaki/Documents/PlatformIO/Projects/esp32_can_test/src/main.cpp` — now v4: ESP32 as BLE central, receives velocity notifications from the Feather and forwards to CAN (replaced the earlier v3 local-wired-joystick version entirely — the joystick now lives only on the Feather). Two independent fail-safes: BLE_TIMEOUT_MS (500ms, zeros target if no BLE update arrives) plus ESC1's own existing 500ms CAN-side fail-safe.
- `/Users/khaleelkhaki/Documents/PlatformIO/Projects/esp32_ble_central_test/` — standalone BLE-receive-and-print test version (predates merging into esp32_can_test), kept as a reference/fallback for BLE-only debugging without CAN in the loop.

**Why this matters:** if BLE stops working again, check for a dead module via the IRQ-probe technique before re-debugging code. If CAN stops working after BLE was just being tested, check ESC1 is actually powered on first — that was the cause the one time it looked like a regression (`twai_transmit` returning `ESP_ERR_TIMEOUT` on every send, which just means nothing is ACKing on the bus).
