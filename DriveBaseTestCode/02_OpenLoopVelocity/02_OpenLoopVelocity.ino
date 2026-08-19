#include <SimpleFOC.h>

// BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver6PWM(pwmAh, pwmAl, pwmBh, pwmBl, pwmCh, pwmCl, (en optional))
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // set your pins

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {

  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12; //set your power supply voltage;
  // limit the maximal dc voltage the driver can set
  driver.voltage_limit = 6; //set your voltage limit; Usually half of power supply voltage is a good
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 1; //set your voltage limit in volts;   // [V]
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // set the target velocity [rad/s]
  motor.target = 6.28; // one rotation per second

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {

  // loop FOC algorithm, should be called as
  // frequently as possible for best
  // performance (e.g. 1kHz+)
  motor.loopFOC();

  // open loop velocity movement
  motor.move();

  // user communication
  command.run();
}
