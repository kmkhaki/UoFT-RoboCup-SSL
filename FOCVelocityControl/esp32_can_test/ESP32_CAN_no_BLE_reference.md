# ESP32 CAN Velocity Sender — pre-Bluetooth reference

This documents the ESP32-side CAN velocity sender **before** Bluetooth was
added (see [[ble-joystick-bridge-working]] for the current BLE-driven
version). `esp32_can_test/src/main.cpp` has since been overwritten with that
BLE version, so this file preserves the two earlier stages for reference.

Both versions share the same protocol: a single signed byte (`int8_t`,
`-128..127`) as CAN ID `0x120`, 250 kbit/s, via ESP-IDF's native
`driver/twai.h` (TX = GPIO5, RX = GPIO35), auto-repeated every 150ms so
ESC1's 500ms CAN-side fail-safe timeout stays satisfied.

---

## v2 — serial-typed velocity commander

Type a number + Enter on the ESP32's serial monitor to set the target
velocity; auto-repeats until a new value is typed. This was the version
validated end-to-end on the real motor via a full `-100 → 100` sweep test
(21 steps, all confirmed via direct `twai_transmit` result checks and the
motor visibly responding).

```cpp
// esp32_can_test/src/main.cpp
// VELOCITY COMMAND (v2): type a number + Enter to set the target velocity;
// it's then auto-repeated on the bus every SEND_INTERVAL_MS until you type a
// new one. This is what feeds ESC1's 500ms fail-safe timeout - typing once
// keeps the motor running, but if this board actually goes silent (crash,
// disconnect, reset) the repeats stop and ESC1 will zero its target on its
// own. Built on the ESP-IDF native TWAI driver.
//
// Range note: -128..127 is the full range of a signed 8-bit integer - simplest
// possible encoding (1 byte, no endianness/precision concerns). The eventual
// target is -150..150 rad/s; when ready, widen to a signed 16-bit value (2
// bytes) instead - same approach, just twice the payload size.

#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_35

#define CAN_ID_VELOCITY 0x120

#define VELOCITY_MIN -128
#define VELOCITY_MAX 127

// Repeat well under ESC1's 500ms fail-safe timeout, so a couple of
// consecutive dropped frames won't trip a stop on their own.
#define SEND_INTERVAL_MS 150

int8_t CurrentVelocity = 0;
uint32_t LastSendMillis = 0;

static void SendVelocity(int8_t Velocity, bool Verbose) {
  twai_message_t TxMsg = {};
  TxMsg.identifier = CAN_ID_VELOCITY;
  TxMsg.data_length_code = 1;
  TxMsg.data[0] = (uint8_t)Velocity;

  esp_err_t Result = twai_transmit(&TxMsg, pdMS_TO_TICKS(100));
  LastSendMillis = millis();

  if (Verbose || Result != ESP_OK) {
    Serial.printf("Sent velocity=%d -> twai_transmit result: %s\n", Velocity,
                  Result == ESP_OK ? "OK" : esp_err_to_name(Result));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 VELOCITY COMMAND (v2): int8_t, -128..127, auto-repeating");

  twai_general_config_t GConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t TConfig = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t FConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&GConfig, &TConfig, &FConfig) != ESP_OK) {
    Serial.println("FATAL: twai_driver_install failed");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("FATAL: twai_start failed");
    while (1) delay(1000);
  }

  Serial.printf("Ready. Type a number (%d..%d) + Enter to set the target velocity (repeats every %dms).\n",
                VELOCITY_MIN, VELOCITY_MAX, SEND_INTERVAL_MS);
}

void loop() {
  // Drain RX (not expecting anything back in this direction, but keep the
  // queue clear and surface anything unexpected for visibility).
  twai_message_t RxMsg;
  while (twai_receive(&RxMsg, 0) == ESP_OK) {
    Serial.printf("Received unexpected ID 0x%lx\n", (unsigned long)RxMsg.identifier);
  }

  // Non-blocking line accumulation - Serial.readStringUntil() WAITS (up to a
  // 1s default timeout) for the terminator to actually arrive if it isn't
  // already buffered, which stalls loop() and, with it, the heartbeat below.
  // Read only what's already available each iteration and never wait.
  //
  // Only digits and a leading '-' are accepted into the buffer - anything
  // else (stray bytes from a monitor reconnecting, control characters, etc.)
  // is silently dropped rather than corrupting the accumulated number. A
  // length cap is a second line of defense against the same failure mode.
  static String InputBuffer;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (InputBuffer.length() > 0) {
        long Value = InputBuffer.toInt();
        if (Value < VELOCITY_MIN) Value = VELOCITY_MIN;
        if (Value > VELOCITY_MAX) Value = VELOCITY_MAX;
        CurrentVelocity = (int8_t)Value;

        Serial.printf("New target velocity: %d\n", CurrentVelocity);
        SendVelocity(CurrentVelocity, true);  // send immediately, don't wait for the next tick
      }
      InputBuffer = "";  // always clear on a line ending, valid or not
    } else if (isDigit(c) || (c == '-' && InputBuffer.length() == 0)) {
      if (InputBuffer.length() < 8) {  // a valid value is at most 4 chars ("-128")
        InputBuffer += c;
      }
    }
    // else: stray/garbage byte, silently ignored
  }

  if (millis() - LastSendMillis >= SEND_INTERVAL_MS) {
    SendVelocity(CurrentVelocity, false);  // silent heartbeat repeat
  }
}
```

---

## v3 — local joystick-driven velocity commander

Directly wired a 2-axis joystick to the ESP32 (the joystick later moved to
the Feather 32u4 over BLE instead — see [[ble-joystick-bridge-working]]).
SW released → X axis drives velocity; SW held → Y axis drives velocity.
Written and compiled successfully but never got a full hardware validation
run of its own before the project moved on to the Bluetooth architecture.

**Wiring:** GND→GND, +5V→3.3V, VRx→GPIO34, VRy→GPIO32, SW→GPIO33 (see
[[can-protocol-details]] for why these specific pins).

```cpp
// esp32_can_test/src/main.cpp
// VELOCITY COMMAND (v3, joystick-driven): a 2-axis joystick sets the target
// velocity, auto-repeated on the bus every SEND_INTERVAL_MS so ESC1's 500ms
// fail-safe timeout stays satisfied even while the joystick sits still.
// Built on the ESP-IDF native TWAI driver.
//
// Control mapping:
//   SW released -> VRx (X axis) drives velocity
//   SW held     -> VRy (Y axis) drives velocity
//
// Joystick wiring (see esp32_joystick_test for the standalone pin-test):
//   GND -> GND   +5V -> 3.3V   VRx -> GPIO34   VRy -> GPIO32   SW -> GPIO33
//
// Range note: -128..127 is the full range of a signed 8-bit integer - simplest
// possible encoding (1 byte, no endianness/precision concerns). The eventual
// target is -150..150 rad/s; when ready, widen to a signed 16-bit value (2
// bytes) instead - same approach, just twice the payload size.

#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_35

#define VRX_PIN 34
#define VRY_PIN 32
#define SW_PIN  33

#define CAN_ID_VELOCITY 0x120

#define VELOCITY_MIN -128
#define VELOCITY_MAX 127

// Raw ADC counts (0..4095) around the calibrated center that are treated as
// "still zero" - filters idle jitter/noise so the motor doesn't creep when
// the stick is resting.
#define DEADBAND 100

// How often the joystick is sampled. Fast enough to feel responsive, slow
// enough not to flood the CAN bus with a new frame every iteration.
#define JOYSTICK_READ_INTERVAL_MS 20

// Repeat well under ESC1's 500ms fail-safe timeout, so a couple of
// consecutive dropped frames won't trip a stop on their own.
#define SEND_INTERVAL_MS 150

// Throttle console printing separately from CAN sending - sending should
// stay prompt on every real change, but printing every 20ms would flood the
// terminal while the stick is being moved continuously.
#define PRINT_INTERVAL_MS 200

int8_t CurrentVelocity = 0;
uint32_t LastSendMillis = 0;

int CenterX = 2048;
int CenterY = 2048;

static void SendVelocity(int8_t Velocity, bool Verbose) {
  twai_message_t TxMsg = {};
  TxMsg.identifier = CAN_ID_VELOCITY;
  TxMsg.data_length_code = 1;
  TxMsg.data[0] = (uint8_t)Velocity;

  esp_err_t Result = twai_transmit(&TxMsg, pdMS_TO_TICKS(100));
  LastSendMillis = millis();

  if (Verbose || Result != ESP_OK) {
    Serial.printf("Sent velocity=%d -> twai_transmit result: %s\n", Velocity,
                  Result == ESP_OK ? "OK" : esp_err_to_name(Result));
  }
}

// Maps a raw ADC reading to -128..127 around a calibrated center, with a
// deadband so noise/imprecision near center doesn't produce nonzero output.
// Center is used (not a hardcoded 2048) because modules vary, and each side
// of center is mapped independently so an off-center resting point still
// reaches the full -128..127 range at the physical extremes.
static int16_t MapJoystickToVelocity(int Raw, int Center) {
  int LowEdge = Center - DEADBAND;
  int HighEdge = Center + DEADBAND;
  if (LowEdge < 1) LowEdge = 1;
  if (HighEdge > 4094) HighEdge = 4094;

  if (Raw >= LowEdge && Raw <= HighEdge) return 0;

  long Velocity;
  if (Raw > HighEdge) {
    Velocity = map(Raw, HighEdge, 4095, 0, VELOCITY_MAX);
  } else {
    Velocity = map(Raw, 0, LowEdge, VELOCITY_MIN, 0);
  }
  if (Velocity < VELOCITY_MIN) Velocity = VELOCITY_MIN;
  if (Velocity > VELOCITY_MAX) Velocity = VELOCITY_MAX;
  return (int16_t)Velocity;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 VELOCITY COMMAND (v3): joystick-driven, -128..127, auto-repeating");

  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

  // Calibrate center point - assumes the stick is untouched right after
  // reset/power-up. Averaging filters ADC noise out of the baseline.
  Serial.println("Calibrating joystick center - leave it untouched...");
  delay(300);
  long SumX = 0, SumY = 0;
  const int CalibSamples = 50;
  for (int i = 0; i < CalibSamples; i++) {
    SumX += analogRead(VRX_PIN);
    SumY += analogRead(VRY_PIN);
    delay(2);
  }
  CenterX = SumX / CalibSamples;
  CenterY = SumY / CalibSamples;
  Serial.printf("Calibrated center: X=%d Y=%d\n", CenterX, CenterY);

  twai_general_config_t GConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t TConfig = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t FConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&GConfig, &TConfig, &FConfig) != ESP_OK) {
    Serial.println("FATAL: twai_driver_install failed");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("FATAL: twai_start failed");
    while (1) delay(1000);
  }

  Serial.println("Ready. SW released = X axis controls velocity, SW held = Y axis controls velocity.");
}

void loop() {
  // Drain RX (not expecting anything back in this direction, but keep the
  // queue clear and surface anything unexpected for visibility).
  twai_message_t RxMsg;
  while (twai_receive(&RxMsg, 0) == ESP_OK) {
    Serial.printf("Received unexpected ID 0x%lx\n", (unsigned long)RxMsg.identifier);
  }

  static uint32_t LastJoystickRead = 0;
  static uint32_t LastPrint = 0;
  if (millis() - LastJoystickRead >= JOYSTICK_READ_INTERVAL_MS) {
    LastJoystickRead = millis();

    int Vrx = analogRead(VRX_PIN);
    int Vry = analogRead(VRY_PIN);
    bool Pressed = (digitalRead(SW_PIN) == LOW);

    int16_t NewVelocity = Pressed ? MapJoystickToVelocity(Vry, CenterY)
                                   : MapJoystickToVelocity(Vrx, CenterX);

    if ((int8_t)NewVelocity != CurrentVelocity) {
      CurrentVelocity = (int8_t)NewVelocity;
      SendVelocity(CurrentVelocity, false);  // send immediately on a real change
    }

    if (millis() - LastPrint >= PRINT_INTERVAL_MS) {
      LastPrint = millis();
      Serial.printf("velocity=%4d  axis=%s  VRx=%4d  VRy=%4d  SW=%s\n", CurrentVelocity,
                    Pressed ? "Y" : "X", Vrx, Vry, Pressed ? "PRESSED" : "released");
    }
  }

  if (millis() - LastSendMillis >= SEND_INTERVAL_MS) {
    SendVelocity(CurrentVelocity, false);  // silent heartbeat repeat
  }
}
```

---

## What replaced this

`esp32_can_test/src/main.cpp` is now v4: the ESP32 acts as a BLE **central**,
receiving joystick + button state from a Feather 32u4 (BLE peripheral) and
forwarding velocity over CAN with the same protocol shown above. See
[[ble-joystick-bridge-working]] and [[can-protocol-details]] for the current
architecture and file locations.
