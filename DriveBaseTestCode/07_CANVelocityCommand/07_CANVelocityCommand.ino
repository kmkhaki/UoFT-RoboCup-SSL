// g431_can_test/src/main.cpp
// VELOCITY COMMAND (v2, interrupt-driven): receives a single signed byte
// (-128..127) over CAN, sent by the ESP32's auto-repeating heartbeat.
// Same interrupt-based reception pattern as CurrentControlTest.ino - this is
// the dry run before touching the real motor sketch, so any problem shows up
// here first with nothing spinning.

#include <Arduino.h>
#include <meFDCAN.h>
#include "MiniRTT.h"

#define CAN_SHDN_PIN PC11

#define CAN_ID_VELOCITY 0x120

volatile int8_t LatestVelocity = 0;
volatile uint32_t MsgCount = 0;
volatile uint32_t LastMsgMillis = 0;

// ISR - keep this fast: no Serial/RTT printing here (meFDCAN's own example
// warns about this - a slow ISR would defeat the point of not polling).
void CAN1_RX_ISR() {
  uint32_t Id;
  uint8_t Len;
  uint8_t RxData[8];
  int Qty = meFDCAN_receive(&Id, &Len, RxData, 1);
  if (Qty > 0 && Id == CAN_ID_VELOCITY) {
    LatestVelocity = (int8_t)RxData[0];
    MsgCount++;
    LastMsgMillis = millis();
  }
}

// FDCAN1's interrupt vector is weakly defined by the STM32 HAL - this
// overrides it. Must be extern "C" to match the HAL's vector table entry.
extern "C" void FDCAN1_IT0_IRQHandler(void) {
  CAN1_RX_ISR();
  FDCAN1->IR |= FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE;  // clear the interrupt flag
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  MiniRTT_Printf("---- ESC1 VELOCITY COMMAND (v2, interrupt-driven) ----\n");
  Serial.println("ESC1 VELOCITY COMMAND (v2, interrupt-driven)");

  pinMode(CAN_SHDN_PIN, OUTPUT);
  digitalWrite(CAN_SHDN_PIN, LOW);  // transceiver active

  // Exact-match filter on our one velocity-command ID (a range with equal
  // bounds) - must be added BEFORE meFDCAN_init(), which bakes in whatever's
  // queued at that point. Same tighter filter as the motor sketch now uses.
  me_FDCAN_addFilter(CAN_ID_VELOCITY, CAN_ID_VELOCITY, FDCAN_FILTER_RANGE, 1);

  bool Ok = meFDCAN_init(250, 1, PA11, PB9);
  if (!Ok) {
    Serial.println("FATAL: meFDCAN_init failed");
    MiniRTT_Printf("FATAL: meFDCAN_init failed\n");
    while (1) delay(1000);
  }
  if (!me_FDCAN_setRxInterrupt(1, 0)) {
    Serial.println("FATAL: CAN interrupt setup failed");
    MiniRTT_Printf("FATAL: CAN interrupt setup failed\n");
    while (1) delay(1000);
  }

  Serial.println("Ready - listening for velocity commands (interrupt-driven).");
  MiniRTT_Printf("Ready - listening for velocity commands (interrupt-driven).\n");
}

void loop() {
  // No polling here - CAN1_RX_ISR() (above) updates the volatiles directly.
  // loop() just reports on changes, at a throttled rate - printing is only
  // ever done here, never in the ISR.
  static uint32_t LastPrintedCount = 0;
  uint32_t Count = MsgCount;  // snapshot - avoid reading a moving target twice
  if (Count != LastPrintedCount) {
    Serial.printf("Received velocity: %d (total %lu)\n", LatestVelocity, (unsigned long)Count);
    MiniRTT_Printf("Received velocity: %d (total %lu)\n", LatestVelocity, (unsigned long)Count);
    LastPrintedCount = Count;
  }

  static uint32_t LastReport = 0;
  if (millis() - LastReport > 2000) {
    bool Stale = (millis() - LastMsgMillis) > 5000;
    Serial.printf("Still alive. Latest velocity=%d  total msgs=%lu%s\n", LatestVelocity, (unsigned long)Count,
                  Stale ? "  [STALE - no recent command]" : "");
    MiniRTT_Printf("Still alive. Latest velocity=%d  total msgs=%lu%s\n", LatestVelocity, (unsigned long)Count,
                   Stale ? "  [STALE]" : "");
    LastReport = millis();
  }
}
