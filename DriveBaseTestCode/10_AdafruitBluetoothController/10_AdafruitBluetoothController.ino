// feather32u4_joystick_ble/src/main.cpp
// Reads the joystick plus 3 buttons and streams them over BLE UART as
// "velocity,btn1,btn2,btn3\n" (e.g. "-45,0,1,0"), so the ESP32 (acting as
// BLE CENTRAL) can pick it up and forward it over CAN.
//
// ROLE NOTE: the onboard nRF51822 module on this board is PERIPHERAL-ONLY -
// Adafruit's own firmware/SoftDevice for nRF51 hardware doesn't support
// Central mode at all (confirmed via Adafruit's own docs: central requires
// the newer nRF52-based boards). So this board must be the BLE peripheral
// (advertises, waits for a connection), and the ESP32 must be the BLE
// central (scans, connects). That's the reverse of the original ESP32
// sketch, which set the ESP32 up as a server/peripheral - see
// esp32_ble_central_test for the corrected ESP32-side client/central code.
//
// Conveniently, this module's built-in BLE UART "DATA mode" natively
// advertises the exact Nordic UART Service UUIDs already used in the
// original ESP32 sketch (6E400001/02/03-B5A3-F393-E0A9-E50E24DCCA9E) - no
// custom GATT setup needed here, just switch to DATA mode and print/read.
//
// Wiring:
//   Joystick: GND->GND  +5V->3V  VRx->A0  VRy->A1  SW->12
//   Buttons (each: pin -> switch -> GND, internal pull-up, pressed = LOW):
//     Button 1 -> 5   Button 2 -> 6   Button 3 -> 9
//   BLE module is built into this board - CS/IRQ/RST (8/7/4) and the
//   hardware SPI pins are already wired on the PCB, nothing to connect.
//   (5/6/9/12 and the BLE-reserved pins are the only ones spoken for so far -
//   0/1/2/3/10/11/13/A2-A5 remain free for anything else added later.)

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"

#define BLUEFRUIT_SPI_CS   8
#define BLUEFRUIT_SPI_IRQ  7
#define BLUEFRUIT_SPI_RST  4

#define VRX_PIN A0
#define VRY_PIN A1
#define SW_PIN  12

#define BTN1_PIN 5
#define BTN2_PIN 6
#define BTN3_PIN 9

#define VELOCITY_MIN -128
#define VELOCITY_MAX 127

// 10-bit ADC here (0..1023), vs. the ESP32 joystick code's 12-bit (0..4095) -
// deadband scaled down proportionally (~100/4095 of full scale).
#define DEADBAND 25

// Matches the CAN heartbeat cadence used elsewhere in this project, so once
// this gets wired into the CAN sender the timing stays consistent.
#define SEND_INTERVAL_MS 150

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

int CenterX = 512;
int CenterY = 512;
int8_t LastSentVelocity = 0;
bool LastBtn1 = false, LastBtn2 = false, LastBtn3 = false;
uint32_t LastSendMillis = 0;

void Error(const __FlashStringHelper *Msg) {
  Serial.println(Msg);
  while (1) delay(1000);
}

// Same shape as the ESP32 joystick mapping (see esp32_can_test) - independent
// linear map on each side of a calibrated center, with a deadband so idle
// jitter doesn't produce nonzero output.
static int16_t MapJoystickToVelocity(int Raw, int Center) {
  int LowEdge = Center - DEADBAND;
  int HighEdge = Center + DEADBAND;
  if (LowEdge < 1) LowEdge = 1;
  if (HighEdge > 1022) HighEdge = 1022;

  if (Raw >= LowEdge && Raw <= HighEdge) return 0;

  long Velocity;
  if (Raw > HighEdge) {
    Velocity = map(Raw, HighEdge, 1023, 0, VELOCITY_MAX);
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
  Serial.println(F("Feather 32u4 joystick -> BLE UART (peripheral)"));

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);

  Serial.print(F("Initialising Bluefruit LE module: "));
  if (!ble.begin(true)) {  // true = verbose AT command echo to Serial while connecting
    Error(F("Couldn't find Bluefruit LE module - check wiring/battery"));
  }
  Serial.println(F("OK"));

  // Factory reset is skipped - it hung indefinitely at AT+FACTORYRESET on
  // this module (a known rough edge with that command on some units/firmware
  // versions) and isn't actually required here: we're not configuring any
  // custom GATT service, just using the module's default/native UART mode.
  //
  // The very next AT command (ATE=0, i.e. ble.echo(false)) then hung too -
  // same symptom, different command. In the official example that command
  // only ever runs AFTER factoryReset(), which reboots the module and (as a
  // side effect) gives it time to fully settle before anything else is sent.
  // Without that reboot, we're hitting it too fast right after begin(). Give
  // it an explicit settling delay instead, and skip echo(false) too - it's
  // purely cosmetic (just quiets AT command echo on Serial), not required.
  delay(500);

  Serial.println(F("Calibrating joystick center - leave it untouched..."));
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
  Serial.print(F("Calibrated center: X="));
  Serial.print(CenterX);
  Serial.print(F(" Y="));
  Serial.println(CenterY);

  Serial.println(F("Advertising - waiting for the ESP32 (central) to connect..."));
  while (!ble.isConnected()) {
    delay(250);
  }
  Serial.println(F("Connected! Switching to DATA mode."));
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

void loop() {
  int Vrx = analogRead(VRX_PIN);
  int Vry = analogRead(VRY_PIN);
  bool Pressed = (digitalRead(SW_PIN) == LOW);

  int16_t Velocity = Pressed ? MapJoystickToVelocity(Vry, CenterY)
                              : MapJoystickToVelocity(Vrx, CenterX);

  bool Btn1 = (digitalRead(BTN1_PIN) == LOW);
  bool Btn2 = (digitalRead(BTN2_PIN) == LOW);
  bool Btn3 = (digitalRead(BTN3_PIN) == LOW);

  bool VelocityChanged = ((int8_t)Velocity != LastSentVelocity);
  bool ButtonsChanged = (Btn1 != LastBtn1) || (Btn2 != LastBtn2) || (Btn3 != LastBtn3);
  bool HeartbeatDue = (millis() - LastSendMillis >= SEND_INTERVAL_MS);

  if (VelocityChanged || ButtonsChanged || HeartbeatDue) {
    LastSentVelocity = (int8_t)Velocity;
    LastBtn1 = Btn1;
    LastBtn2 = Btn2;
    LastBtn3 = Btn3;
    LastSendMillis = millis();

    ble.print(LastSentVelocity);
    ble.print(',');
    ble.print(Btn1 ? '1' : '0');
    ble.print(',');
    ble.print(Btn2 ? '1' : '0');
    ble.print(',');
    ble.print(Btn3 ? '1' : '0');
    ble.print('\n');

    Serial.print(F("Sent: velocity="));
    Serial.print(LastSentVelocity);
    Serial.print(F(" btn1="));
    Serial.print(Btn1);
    Serial.print(F(" btn2="));
    Serial.print(Btn2);
    Serial.print(F(" btn3="));
    Serial.println(Btn3);
  }
}
