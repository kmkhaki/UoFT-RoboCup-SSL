#include <SimpleFOC.h>

#define   POLE_PAIRS        12
#define   PHASE_RESISTANCE  0.168
#define   KV_RATING         330
#define   INDUCTANCE_Q      NOT_SET   // optional - can be left as NOT_SET for basic use

BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, INDUCTANCE_Q);
// TODO instance driver
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // set your pins

// TODO instance sensor
Encoder encoder = Encoder(PB6, PB7, 1000);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
static uint32_t last = micros();

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  //motor.voltage_limit = 2;

  driver.voltage_power_supply = 12; // set voltage [V]
  //driver.voltage_limit = 6; // set voltage [V]
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 1.5;

  // Estimated current control
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::estimated_current;

  motor.updateCurrentLimit(3.0); // A
  motor.target = 0.0;            // A - zero torque command to start

  if(!motor.init()) { Serial.println("Motor init failed"); return; }
  if(!motor.initFOC()) { Serial.println("FOC init failed"); return; }

  command.add('M', doMotor, "Motor");
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();
  command.run();

  if (millis() - last >= 100) {
    last = millis();

    Serial.println(encoder.getVelocity(), 3);
  }
}
