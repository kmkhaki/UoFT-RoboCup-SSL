#include <SimpleFOC.h>

// encoder instance
Encoder encoder = Encoder(PB6, PB7, 1000);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(12);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // set your pins

// commander interface
Commander command = Commander(Serial);
void onPID(char* cmd){ command.pid(&motor.PID_velocity, cmd); }
void onLPF(char* cmd){ command.lpf(&motor.LPF_velocity, cmd); }
void onMotion(char* cmd){ command.motion(&motor, cmd); }

static uint32_t last = micros();

void setup() {
  // monitoring port
  Serial.begin(115200);
  // enable the debugging output
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  motor.voltage_limit = 2;
  motor.voltage_sensor_align = 1.5;
  //motor.sensor_direction = CW;

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  // set torque control type to voltage
  motor.torque_controller = TorqueControlType::voltage;

  // contoller configuration based on the control type
  //at P=0.24, the motor starts to vibrate, so seeting it to between 0.5 and 0.7, I chose 0.144 (60%)
  motor.PID_velocity.P = 0.144;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;

  // calculate the filter time constant
  // based on the max velocity you need
  // and the rule of thumb for the cutoff frequency
  float max_velocity = 100.0;                       // rad/s
  float motor_frequency_hz = max_velocity / (2 * PI); // ~16 Hz
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;//1.0 / (2.0 * PI * motor_frequency_hz * 5); // ~80 Hz

  // use monitoring with serial for motor init
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 10; // Rad / sec

  // define the motor id
  command.add('V', onPID, "pid");
  command.add('L', onLPF, "lpf");
  command.add('M', onMotion, "motion");
  command.decimal_places = 5; // for better visibility of the changes in the parameters

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Target velocity : target 2 Rad/sec."));
  Serial.println(F("Set/Read the PID gains with: V"));
  Serial.println(F("Set/Read the LPF time constant with: L"));
  Serial.println(F("Set the motion target with: M"));
  // print current values
  Serial.println(F("Current PID gains:"));
  Serial.print(F("P: ")); Serial.println(motor.PID_velocity.P);
  Serial.print(F("I: ")); Serial.println(motor.PID_velocity.I);
  Serial.print(F("D: ")); Serial.println(motor.PID_velocity.D);
  Serial.print(F("Current LPF Tf: ")); Serial.println(motor.LPF_velocity.Tf);

  motor.PID_velocity.output_ramp = 200;

  _delay(1000);
}


void loop() {
  // iterative setting of the FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if target not set in parameter uses motor.target variable
  motor.move();
  // user communication
  command.run();

  /*if (millis() - last >= 100) {
    last = millis();

    Serial.print(F("P: ")); Serial.print(motor.PID_velocity.P);
    Serial.print(F(" I: ")); Serial.print(motor.PID_velocity.I);

    Serial.print("\tTarget: ");
    Serial.print(motor.target);

    Serial.print("\tAngle: ");
    Serial.print(encoder.getAngle(), 3);

    Serial.print("\tVelocity: ");
    Serial.print(encoder.getVelocity(), 3);

    Serial.print("\tUq: ");
    Serial.println(motor.voltage.q, 3);
  }*/
}
