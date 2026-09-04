// Smallest program that brings up the 6-PWM driver. Keep it: it is the fastest
// way to tell a board/wiring problem from a sketch problem, and it is what
// isolated the weak-symbol linking bug (see lib_archive = no in platformio.ini).
// Expect "driver.init() = 1" plus STM32-DRV timer lines. A 0 with NO STM32-DRV
// output means the weak generic stub got linked instead of the STM32 one.
#include <Arduino.h>
#include <SimpleFOC.h>

Encoder encoder = Encoder(PB6, PB7, 1000);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

BLDCMotor motor = BLDCMotor(12);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,
                                       A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

void setup() {
  Serial.begin(115200);
  delay(2000);
  SimpleFOCDebug::enable(&Serial);
  SimpleFOCDebug::println("debug alive");
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  driver.voltage_power_supply = 24.5f;
  driver.voltage_limit = 6;
  int R = driver.init();
  Serial.print(F("driver.init() = ")); Serial.println(R);
  Serial.println(F("MINTEST DONE"));
}
void loop() {}
