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