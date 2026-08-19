#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(12);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // set your pins

// encoder instance
Encoder encoder = Encoder(PB6, PB7, 1000);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void onPID(char* cmd){ command.pid(&motor.PID_velocity, cmd); }
void onLPF(char* cmd){ command.lpf(&motor.LPF_velocity, cmd); }
void onMotion(char* cmd){ command.motion(&motor, cmd); }

float max_velocity = 100.0;                       // rad/s
float motor_frequency_hz = max_velocity / (2 * PI); // ~16 Hz
float filter_cutoff_hz = motor_frequency_hz * 5;    // ~80 Hz

static uint32_t last = micros();
//static uint32_t count = 0;*/

void setup() {
  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  //added, add to documentation why
  motor.PID_velocity.P = 0.144;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01;
  motor.P_angle.P = 20;
  motor.PID_velocity.output_ramp = 200;

  // Tf = 1 / (2 * PI * f_cutoff)
  //motor.LPF_velocity.Tf = 1.0 / (2.0 * PI * filter_cutoff_hz);
  motor.velocity_limit = max_velocity;
  motor.voltage_limit = 2;
  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_limit = 6;
  driver.voltage_power_supply = 12;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 1.5;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  motor.useMonitoring(Serial);
  // initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the initial motor target
  motor.target = 0; // Volts

  // add target command M
  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));

  Serial.print("P_angle.P = ");
  Serial.println(motor.P_angle.P);

  Serial.print("PID_velocity.P = ");
  Serial.println(motor.PID_velocity.P);

  Serial.print("PID_velocity.I = ");
  Serial.println(motor.PID_velocity.I);

  Serial.print("Velocity limit = ");
  Serial.println(motor.velocity_limit);

  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();

  /*if (millis() - last >= 50) {
    last = millis();

    Serial.print("Target: ");
    Serial.print(motor.target);

    Serial.print("\tAngle: ");
    Serial.print(encoder.getAngle(), 3);

    Serial.print("\tVelocity: ");
    Serial.print(encoder.getVelocity(), 3);

    Serial.print("\tUq: ");
    Serial.println(motor.voltage.q, 3);
  }*/
  //Serial.println(encoder.getAngle());
}
