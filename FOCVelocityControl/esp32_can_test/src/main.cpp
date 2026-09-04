// esp32_can_test/src/main.cpp
// VELOCITY COMMAND (v2): type a number + Enter to set the target velocity;
// it's then auto-repeated on the bus every SEND_INTERVAL_MS until you type a
// new one. This is what feeds ESC1's 500ms fail-safe timeout - typing once
// keeps the motor running, but if this board actually goes silent (crash,
// disconnect, reset) the repeats stop and ESC1 will zero its target on its
// own. Built on the ESP-IDF native TWAI driver.
//
// Temporarily restored (from ESP32_CAN_no_BLE_reference.md) in place of the
// BLE-driven v4, for clean/repeatable typed-value testing of the ESC1 motor
// stall behavior - no joystick jitter in the way. Backup of v4 is at
// /private/tmp/.../scratchpad/main_v4_ble.cpp.bak - restore it to go back.
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

// Bus-off recovery. The TWAI controller removes itself from the bus once its
// transmit error counter passes 255, which happens within seconds if nothing
// acknowledges - a loose CAN wire is enough. Without this it stays off
// permanently, returning ESP_ERR_INVALID_STATE to every transmit forever, and
// only a power cycle brings it back. Observed exactly that on 2026-09-04.
//
// Recovery also makes diagnosis honest: a latched bus-off reports the same
// error whether or not the underlying wiring has since been fixed, whereas
// after recovery a genuine wiring fault shows as ESP_ERR_TIMEOUT (nobody
// ACKing) and a healthy bus simply works.
static void RecoverIfBusOff() {
  twai_status_info_t Status;
  if (twai_get_status_info(&Status) != ESP_OK) return;
  if (Status.state == TWAI_STATE_BUS_OFF) {
    Serial.println("CAN bus-off detected - initiating recovery");
    twai_initiate_recovery();
  } else if (Status.state == TWAI_STATE_STOPPED) {
    // recovery finished; the controller must be restarted explicitly
    if (twai_start() == ESP_OK) Serial.println("CAN restarted after recovery");
  }
}

static void SendVelocity(int8_t Velocity, bool Verbose) {
  twai_message_t TxMsg = {};
  TxMsg.identifier = CAN_ID_VELOCITY;
  TxMsg.data_length_code = 1;
  TxMsg.data[0] = (uint8_t)Velocity;

  esp_err_t Result = twai_transmit(&TxMsg, pdMS_TO_TICKS(100));
  LastSendMillis = millis();

  // INVALID_STATE means the controller is not in a state that accepts frames -
  // bus-off, or stopped mid-recovery. Both are handled here.
  if (Result == ESP_ERR_INVALID_STATE) RecoverIfBusOff();

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

// Diagnostic: watch the transceiver's R (receive) output directly.
//
// TWAI owns GPIO35 normally, so the driver is stopped and uninstalled first to
// release the pin. With a POWERED, ACTIVE transceiver the idle bus is recessive
// and R sits HIGH; any traffic on CANH/CANL makes it toggle. This separates the
// cases that all leave bus resistance looking normal:
//   toggles          -> module powered, active, and hearing the bus
//   steady HIGH      -> module powered but hears nothing (bus not reaching it)
//   steady LOW/float -> module unpowered, or RS floating (standby, driver off)
static void ProbeRxPin() {
  Serial.println("Releasing TWAI and probing GPIO35 (R) for 5 s - have the ESC1 transmit now");
  twai_stop();
  twai_driver_uninstall();
  pinMode(CAN_RX_PIN, INPUT);
  int First = digitalRead(CAN_RX_PIN);
  int Lo = 0, Hi = 0, Edges = 0, Prev = First;
  uint32_t End = millis() + 5000;
  while (millis() < End) {
    int V = digitalRead(CAN_RX_PIN);
    if (V) Hi++; else Lo++;
    if (V != Prev) { Edges++; Prev = V; }
  }
  Serial.printf("GPIO35: high=%d low=%d edges=%d\n", Hi, Lo, Edges);
  if (Edges > 10)      Serial.println("-> TOGGLING: transceiver is powered, active, and hearing the bus");
  else if (Hi > 0 && Lo == 0) Serial.println("-> steady HIGH: powered and idle, but no bus traffic reaching it");
  else if (Lo > 0 && Hi == 0) Serial.println("-> steady LOW: transceiver unpowered, in standby (RS floating), or bus stuck dominant");
  else                 Serial.println("-> indeterminate");
  Serial.println("Reboot the ESP32 to restore CAN.");
}

// Local transceiver self-test, independent of anything else on the bus.
//
// A CAN transceiver's receiver monitors the bus continuously, including frames
// it is itself driving. So with the chip powered and out of standby, driving D
// LOW forces the bus dominant and R MUST follow LOW; releasing D HIGH lets the
// bus go recessive and R MUST return HIGH. That makes D->R a complete loopback
// test of power, RS/standby state, and the chip itself - no second node, no
// multimeter, and it does not care whether the far end is connected.
//
//   R follows D        -> transceiver is healthy; fault is beyond it (harness/ESC1)
//   R stuck LOW        -> bus held dominant, or the chip cannot release it
//   R stuck HIGH       -> chip is not driving the bus at all (standby / D not wired)
static void TransceiverSelfTest() {
  Serial.println("Transceiver self-test: driving D (GPIO5), watching R (GPIO35)");
  twai_stop();
  twai_driver_uninstall();
  pinMode(CAN_RX_PIN, INPUT);
  pinMode(CAN_TX_PIN, OUTPUT);
  int MatchLow = 0, MatchHigh = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(CAN_TX_PIN, HIGH);            // recessive
    delayMicroseconds(500);
    int RHigh = digitalRead(CAN_RX_PIN);
    digitalWrite(CAN_TX_PIN, LOW);             // dominant
    delayMicroseconds(500);
    int RLow = digitalRead(CAN_RX_PIN);
    if (RHigh == 1) MatchHigh++;
    if (RLow  == 0) MatchLow++;
    Serial.printf("  D=1 -> R=%d   D=0 -> R=%d\n", RHigh, RLow);
  }
  digitalWrite(CAN_TX_PIN, HIGH);              // leave the bus recessive
  Serial.printf("recessive matches=%d/8  dominant matches=%d/8\n", MatchHigh, MatchLow);
  if (MatchHigh >= 7 && MatchLow >= 7)
    Serial.println("-> TRANSCEIVER OK: R follows D. Fault is beyond it (harness or ESC1 end).");
  else if (MatchLow >= 7 && MatchHigh == 0)
    Serial.println("-> R STUCK LOW: bus held dominant, or chip cannot release it.");
  else if (MatchHigh >= 7 && MatchLow == 0)
    Serial.println("-> R STUCK HIGH: chip not driving the bus (standby / D not wired).");
  else
    Serial.println("-> inconsistent: suspect wiring of D or R.");
  Serial.println("Reboot the ESP32 to restore CAN.");
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
    } else if (c == 'p' || c == 'P') {
      ProbeRxPin();
    } else if (c == 't' || c == 'T') {
      TransceiverSelfTest();
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
