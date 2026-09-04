// B-G431B-ESC1 - working closed-loop FOC
//
// Everything here rests on measurements taken on this hardware, not defaults:
//
//   pole pairs 12      24 electrical revs = 2.000 mechanical revs
//   encoder    4000 CPR / 1000 PPR, hand turn = 4002 counts
//   rail       ~24.5 V measured at A_VBUS (NOT the 12 V the old sketch declared)
//   Ke         ~0.0227 V.s/rad (~44 rad/s per volt of Uq, ~421 rpm/V)
//
// The thing that actually made closed loop work: the encoder's angle does not
// map linearly onto the rotor's electrical angle. The residual error swings
// about +-4 rad ELECTRICAL over a revolution, roughly two cycles per turn, and
// it is repeatable and torque-independent (identical at 1.5 V and 2.5 V drive).
// Torque reverses beyond +-PI/2, so that error flips the sign of the torque
// several times per revolution. No constant offset can fix it, which is why
// alignSensor() and a full sweep of zero_electric_angle both failed - measured
// directly: best travel over 24 offsets was 0.15 rev, versus 42 rev compensated.
//
// So commutation is done here rather than through BLDCMotor: the electrical
// angle is taken from the encoder and then corrected by a measured per-position
// map. FOCMotor::electricalAngle() is not virtual and correcting the sensor
// angle instead risks a non-monotonic angle (the correction's slope can exceed
// the raw angle's), so the loop is explicit.

#include <Arduino.h>
#include <SimpleFOC.h>
#include <meFDCAN.h>
#include "map_data.h"

// --- CAN velocity command -----------------------------------------------
// Protocol (unchanged from the working ESP32 link): one signed byte, -128..127,
// CAN ID 0x120 at 250 kbit/s, applied 1:1 as a rad/s target. The ESP32 repeats
// the last value every 150 ms as a heartbeat.
#define CAN_SHDN_PIN     PC11      // drive LOW to enable the onboard transceiver
// The board has a switchable 120 ohm bus terminator on PC14. A CAN bus wants
// one at EACH end (60 ohm total); with only the transceiver module's resistor
// fitted the bus is singly terminated and reflections show up as stuff errors.
// Polarity is not documented in the variant header, so it is made settable and
// determined by measuring the error rate both ways.
#define CAN_TERM_PIN     PC14
#define CAN_ID_VELOCITY  0x120
// If the heartbeat stops (ESP32 crash, unplugged bus) the motor must not keep
// spinning. 500 ms is comfortably longer than the 150 ms repeat, so a couple of
// dropped frames will not trip it on their own.
#define CAN_TIMEOUT_MS   500

volatile int8_t   CanVelocity   = 0;
volatile uint32_t CanMsgCount   = 0;
volatile uint32_t CanLastMillis = 0;

// Keep the ISR short - it runs alongside a FOC loop that must not be stalled.
void CAN1_RX_ISR() {
  uint32_t Id;
  uint8_t  Len;
  uint8_t  RxData[8];
  int Qty = meFDCAN_receive(&Id, &Len, RxData, 1);
  if (Qty > 0 && Id == CAN_ID_VELOCITY) {
    CanVelocity   = (int8_t)RxData[0];
    CanMsgCount++;
    CanLastMillis = millis();
  }
}

// FDCAN1's vector is weakly defined by the STM32 HAL; this overrides it and
// must be extern "C" to match the vector table entry.
extern "C" void FDCAN1_IT0_IRQHandler(void) {
  CAN1_RX_ISR();
  FDCAN1->IR |= FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE;
}

#define POLE_PAIRS  12
#define ENCODER_PPR 1000
#define VBUS_DIVIDER 10.39f
// 64 bins is 5.6 deg mechanical per bin, which at 12 pole pairs is 67 deg
// ELECTRICAL per bin - far too coarse when torque reverses at 90 deg, and the
// measured symptom was the motor intermittently stalling mid-revolution even in
// open-loop torque mode. 256 bins is ~17 deg electrical per bin.
// Cost: 256 * 16 B = 4 kB of the 32 kB RAM, and ~37 samples per bin from the
// 9600-sample calibration sweep.
#define MAP_BINS    256

Encoder encoder = Encoder(PB6, PB7, ENCODER_PPR);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

BLDCMotor motor = BLDCMotor(POLE_PAIRS);   // used only for setPhaseVoltage()
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,
                                       A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// --- calibration map --------------------------------------------------------
float MapRaw[MAP_BINS];
bool  MapValid = false;

float ReadBusVoltage() {
  return analogRead(A_VBUS) * (3.3f / 4095.0f) * VBUS_DIVIDER;
}

// getMechanicalAngle() spans (-2PI, 2PI) because Encoder::update() uses
// (counter % cpr) and C++ keeps the sign of the dividend. Everything that bins
// or indexes on it has to fold it into [0, 2PI) first.
static inline float Wrap2Pi(float A) { return A < 0 ? A + _2PI : A; }

float MapLookup(float Mech) {
  if (!MapValid) return 0;
  float F = Wrap2Pi(Mech) / _2PI * MAP_BINS;
  int Bin0 = ((int)F) % MAP_BINS;          // not B0/B1: binary.h defines those
  int Bin1 = (Bin0 + 1) % MAP_BINS;
  float Frac = F - (int)F;
  float Lo = MapRaw[Bin0], Hi = MapRaw[Bin1];
  float D = Hi - Lo;
  while (D >  _PI) D -= _2PI;
  while (D < -_PI) D += _2PI;
  return Lo + Frac * D;
}

// Speed-proportional lead angle, in electrical rad per (shaft rad/s).
// The map is measured quasi-statically at ~1.3 rad/s, but at speed the phase
// current lags the applied voltage by roughly atan(we*L/R), which grows with
// electrical frequency (we = 12 * shaft speed here). That lag is absent from
// the map, so torque falls off and ripples as speed rises. Advancing the
// commutation angle in proportion to speed compensates it.
// Sign is empirical: el decreases as the shaft turns positive, so leading in
// the direction of motion means a NEGATIVE gain. Tunable at runtime via 'G'.
float LeadGain = 0.0f;
extern float ShaftVelocity;

// The rotor's electrical angle, corrected. The map stores
// err = (-pp*mech) - rotor_electrical, so rotor_electrical = -pp*mech - err.
float RotorElectricalAngle() {
  float Mech = encoder.getMechanicalAngle();
  return _normalizeAngle(-(float)POLE_PAIRS * Mech - MapLookup(Mech)
                         + LeadGain * ShaftVelocity);
}

float MapSin[MAP_BINS], MapCos[MAP_BINS];
long  MapN[MAP_BINS];
float CalVoltage = 1.5f;

void MapSweep(int Dir, int ElecRevs, int StepDelayUs) {
  const int StepsPerRev = 200;
  float El = _3PI_2;
  uint32_t Until = millis() + 600;
  while ((int32_t)(millis() - Until) < 0) {
    motor.setPhaseVoltage(CalVoltage, 0, El); encoder.update(); delayMicroseconds(200);
  }
  for (int i = 0; i < ElecRevs * StepsPerRev; i++) {
    El += Dir * _2PI / StepsPerRev;
    motor.setPhaseVoltage(CalVoltage, 0, El);
    encoder.update();
    delayMicroseconds(StepDelayUs);
    // During a slow open-loop sweep the rotor d-axis sits at ElCmd + PI/2, so
    // the true electrical angle is known and can be differenced against what
    // the encoder predicts. Both directions are accumulated into the same bins
    // so the load angle, which is direction-dependent, cancels.
    float Mech = encoder.getMechanicalAngle();
    float Err = _normalizeAngle(-(float)POLE_PAIRS * Mech - (El + _PI_2));
    int B = (int)(Wrap2Pi(Mech) / _2PI * MAP_BINS);
    if (B < 0) B = 0;
    if (B >= MAP_BINS) B = MAP_BINS - 1;
    MapSin[B] += sinf(Err);
    MapCos[B] += cosf(Err);
    MapN[B]++;
  }
}

// DO NOT USE AT BOOT. Kept only for reference/experiments.
//
// A stored map CANNOT be reused across power cycles as things stand. The
// encoder is INCREMENTAL: its zero is wherever the rotor sat at power-on, and
// the map is indexed by getMechanicalAngle(), which counts from that zero. A
// map captured in one boot is therefore offset by an arbitrary amount in the
// next, making the commutation angle wrong at every position. The failure is
// not subtle - the field lands on the rotor's d-axis, giving full current and
// zero torque, i.e. an instant stall at the supply's current limit. Observed
// exactly that on 2026-09-04: 3 A and a dead stall the moment it was commanded.
//
// To make a stored map valid you first need an absolute position reference.
// The encoder's index/Z channel is on PB8 and is currently unused - finding
// the index once at startup would pin the map to a known rotor angle and make
// persistence sound. Until then, measure the map every boot.
void LoadStoredMap() {
  for (int i = 0; i < MAP_BINS; i++) MapRaw[i] = MapDefault[i];
  MapValid = true;
  Serial.println(F("loaded stored position map (no sweep, shaft stays still)"));
}

void CalibrateMap() {
  Serial.println(F("calibrating position map (~15 s, shaft will turn) ..."));
  for (int i = 0; i < MAP_BINS; i++) { MapSin[i] = MapCos[i] = 0; MapN[i] = 0; }
  driver.enable();
  encoder.update();
  float A0 = encoder.getAngle();
  MapSweep(+1, 24, 1500);
  encoder.update();
  float A1 = encoder.getAngle();
  MapSweep(-1, 24, 1500);
  motor.setPhaseVoltage(0, 0, 0);
  driver.disable();

  // The sweep drives 24 electrical revolutions, which at 12 pole pairs is 2
  // mechanical revolutions (~12.57 rad). If the shaft did not actually turn,
  // every bin still gets samples - they are just all from a stationary rotor -
  // and the old code reported "256/256 OK" on a completely invalid map.
  //
  // That is exactly what happens when the board is powered over ST-Link USB
  // with the 24 V motor supply OFF: calibration silently "succeeds" and the
  // motor then stalls the instant real power returns. Checking that the rotor
  // moved is the difference between a caught error and a mystery stall.
  const float Expected = 24.0f * _2PI / POLE_PAIRS;   // ~12.57 rad
  float Moved = fabsf(A1 - A0);
  if (Moved < Expected * 0.5f) {
    MapValid = false;
    Serial.print(F("CALIBRATION FAILED: shaft moved only "));
    Serial.print(Moved, 2);
    Serial.print(F(" rad, expected ~")); Serial.print(Expected, 2);
    Serial.println(F(" rad"));
    Serial.println(F("Is the motor supply ON? Map is INVALID - do not run the motor."));
    Serial.print(F("VBUS reads ")); Serial.print(ReadBusVoltage(), 2); Serial.println(F(" V"));
    return;
  }
  Serial.print(F("shaft moved ")); Serial.print(Moved, 2);
  Serial.print(F(" rad (expected ~")); Serial.print(Expected, 2); Serial.println(F(")"));

  int Empty = 0;
  for (int i = 0; i < MAP_BINS; i++) {
    if (!MapN[i]) { MapRaw[i] = 0; Empty++; continue; }
    MapRaw[i] = atan2f(MapSin[i], MapCos[i]);
  }
  MapValid = (Empty == 0);
  Serial.print(F("map bins filled: ")); Serial.print(MAP_BINS - Empty);
  Serial.print('/'); Serial.print(MAP_BINS);
  Serial.println(MapValid ? F("  OK") : F("  INCOMPLETE - recalibrate"));
}

void PrintMap(char* cmd) {
  Serial.println(F("bin\tmech_rad\terr_elec_rad"));
  for (int i = 0; i < MAP_BINS; i++) {
    Serial.print(i); Serial.print('\t');
    Serial.print(_2PI * i / MAP_BINS, 4); Serial.print('\t');
    Serial.println(MapRaw[i], 4);
  }
}

// --- control ----------------------------------------------------------------
// Measured on this motor/encoder pair: a POSITIVE Uq drives the shaft NEGATIVE
// (Uq +1.5 V -> -62 rad/s). Folding that in here means a positive target is a
// positive shaft velocity in both modes; without it the velocity loop feeds
// back positively and runs away, which is exactly what the first test did.
#define TORQUE_SIGN (-1.0f)

enum Mode { MODE_TORQUE, MODE_VELOCITY };
Mode  ControlMode = MODE_TORQUE;
bool  DriverEnabled = false;
// When true, Target is driven by the CAN heartbeat rather than the serial T
// command. Serial still overrides for bench work - useful when the bus is down.
bool  CanControl = true;

// --- stall protection ---------------------------------------------------
// There is NO current sensing in this firmware (SimpleFOC 2.4.0 reports
// "Low-side cs not supported!" for this board), so the supply's own current
// limit has been the only protection. A stall is the dangerous case: with no
// back-EMF the current is just Uq/R, which at Vlim=3 V and ~0.168 ohm is on the
// order of 18 A demanded. On the bench the supply folds back at 3 A; on a
// battery nothing would.
//
// A stall is therefore detected behaviourally: we are pushing real voltage at a
// meaningful commanded speed, and the shaft is not turning.
#define STALL_VEL_THRESH   2.0f    // rad/s - below this counts as not turning
#define STALL_TARGET_MIN   5.0f    // only meaningful commands can stall
#define STALL_UQ_THRESH    0.4f    // V - must actually be pushing
#define STALL_TIME_MS      300     // sustained, so normal starts do not trip it

// Runaway protection. Discovered the hard way on 2026-09-04: with commutation
// inverted (a 90 deg map error), commanding +30 rad/s drove the shaft to -163
// rad/s. The velocity loop saw a growing error and pushed HARDER in the
// direction making it worse - positive feedback. A stall is bounded by the
// supply's current limit; a runaway is not, so this matters more.
#define OVERSPEED_LIMIT    140.0f  // rad/s - above anything commandable (int8 caps at 127)
#define RUNAWAY_TIME_MS    250     // sustained wrong-direction overshoot

// Wrong-direction-under-load. The gap between the two rules above: with
// commutation inverted the shaft sat at -3 rad/s against a +30 command, pulling
// an estimated 17 A - too fast for the stall rule (<2 rad/s), far too slow for
// the runaway rule (>80 rad/s), so nothing tripped. Direction is the reliable
// signal here, not magnitude.
// The timeout is deliberately long: during a LEGITIMATE reversal the shaft is
// also moving against the new target while it brakes through zero, and that
// must not trip. Settling is 25-50 ms, so 500 ms leaves wide margin.
#define REVERSE_TIME_MS    500

// Over-current on the ESTIMATE - the primary protection.
//
// The direction-based rules proved unreliable: with commutation broken the
// velocity oscillates in sign, so a wrong-direction fault took 5 s to trip once
// and was missed entirely the next run. Measured current separates the cases
// far more cleanly - normal running sits at 0.7-3.5 A with a 6.7 A worst-case
// acceleration transient, while both stall and reverse faults sat at ~17-18 A.
//
// This does not need PhaseR to be accurate: an error in R scales normal and
// fault readings together, so the ~5x separation survives. 12 A leaves margin
// above the worst normal transient and well below any observed fault.
#define OVERCURRENT_LIMIT   12.0f   // A, estimated
#define OVERCURRENT_TIME_MS 200

// Plant constants for the current ESTIMATE (reporting only - not a trip).
// Ke is measured (44.1 rad/s per volt -> 1/44.1). PhaseR is from the project
// notes and has NOT been verified here, so the estimate is indicative only.
#define MOTOR_KE     0.0227f       // V.s/rad, measured
#define MOTOR_PHASE_R 0.168f       // ohm, from notes - unverified

bool     StallFault = false;      // latched fault, any cause
uint32_t StallSinceMs = 0;
uint32_t RunawaySinceMs = 0;
uint32_t ReverseSinceMs = 0;
uint32_t OverCurrentSinceMs = 0;
const char* FaultReason = "none";
float    LastUq = 0;

float EstimateCurrent(float Uq, float Vel) {
  float Drive = fabsf(Uq) - MOTOR_KE * fabsf(Vel);
  if (Drive < 0) Drive = 0;
  return Drive / MOTOR_PHASE_R;
}
float Target = 0;
float VoltageLimit = 3.0f;

// Gains sized from the measured plant: ~44 rad/s of shaft speed per volt of Uq.
// Holding 30 rad/s therefore needs ~0.7 V, so P=0.03 gives most of that from
// proportional alone and I only has to trim the remainder plus friction.
// output_ramp bounds how fast Uq can move, which keeps reversals from slamming
// the supply into current limit.
// Tuned against the RAW ENCODER ANGLE, not the filtered velocity.
//
// The first tuning pass optimised sd of VelLPF's output - the very signal the
// filter smooths - so it was blind to the oscillation it was creating and
// settled on P=0.05/I=5.0/Tf=0.05. Recording raw angle at 1 kHz showed that
// loop running a ~12 Hz limit cycle: velocity mode measured sd 35 at 40 rad/s
// where open-loop torque mode at the same speed measured sd 9.5. The loop was
// making the motor ~4x jerkier than no loop at all.
//
// A ripple whose frequency barely moves with shaft speed (8.8 Hz at 10 rad/s,
// 12.6 Hz at 40) is loop dynamics; commutation ripple would track electrical
// frequency (20 Hz -> 77 Hz over that range). That is what identified it.
//
// Retuned against raw-angle ripple, at target 20 rad/s:
//   P=0.05 I=5.0  Tf=0.05  sd 10.71   (the old defaults)
//   P=0.08 I=0.5  Tf=0.01  sd  1.98   <- these
// Across speeds: sd 5.48 @10, 1.98 @20, 5.72 @40, 13.04 @60, and settling
// improved to 14-16 ms with steady-state error still under 0.05 rad/s.
PIDController  VelPID  = PIDController(0.08f, 0.5f, 0.0f, 200.0f, 3.0f);
LowPassFilter  VelLPF  = LowPassFilter(0.01f);
float ShaftVelocity = 0;   // referenced by RotorElectricalAngle() above

// Encoder::getVelocity() is derived from pulse timing and holds its last value
// for up to 100 ms after the final pulse before forcing zero, which makes it a
// poor error signal for a loop running at tens of kHz. Differentiating the
// continuous angle over a fixed 2 ms window is better behaved: getAngle() is
// full_rotations*2PI + angle_prev, so it does not wrap.
float SampleVelocity() {
  static uint32_t LastUs = 0;
  static float LastAngle = 0;
  static float Vel = 0;
  uint32_t Now = micros();
  if (LastUs == 0) { LastUs = Now; LastAngle = encoder.getAngle(); return 0; }
  uint32_t Dt = Now - LastUs;
  if (Dt >= 2000) {
    float A = encoder.getAngle();
    Vel = (A - LastAngle) / (Dt * 1e-6f);
    LastAngle = A;
    LastUs = Now;
  }
  return Vel;
}

void doTarget(char* cmd)  { Target = atof(cmd); VelPID.reset(); }
void doLimit(char* cmd)   { VoltageLimit = atof(cmd); VelPID.limit = VoltageLimit; }
void doP(char* cmd)       { VelPID.P = atof(cmd); }
void doTf(char* cmd)      { VelLPF.Tf = atof(cmd); }
void doLead(char* cmd)    { LeadGain = atof(cmd); }
void doI(char* cmd)       { VelPID.I = atof(cmd); }
void doCalib(char* cmd)   { Target = 0; CalibrateMap(); }

void doMode(char* cmd) {
  Target = 0;
  VelPID.reset();
  ControlMode = (cmd[0] == '1') ? MODE_VELOCITY : MODE_TORQUE;
  Serial.println(ControlMode == MODE_VELOCITY
                 ? F("mode = velocity (target rad/s)")
                 : F("mode = torque (target Uq volts)"));
}

bool Streaming = false;
void doStream(char* cmd) { Streaming = !Streaming; }

void StreamTelemetry(float Uq) {
  static uint32_t LastMs = 0;
  if (!Streaming || millis() - LastMs < 20) return;
  LastMs = millis();
  Serial.print(F("t=")); Serial.print(millis());
  Serial.print(F(" tgt=")); Serial.print(Target, 1);
  Serial.print(F(" vel=")); Serial.print(ShaftVelocity, 2);
  Serial.print(F(" Uq=")); Serial.print(Uq, 3);
  Serial.print(F(" Vbus=")); Serial.println(ReadBusVoltage(), 2);
}

// Velocity statistics accumulated ON DEVICE, with no serial traffic during the
// run. StreamTelemetry() emits ~60 chars every 20 ms, which is ~5 ms of UART
// time at 115200; a blocking write stalls the FOC loop and freezes commutation,
// so streaming while measuring perturbs the very thing being measured.
void doQuality(char* cmd) {
  float Uq = cmd && cmd[0] ? atof(cmd) : 1.5f;
  bool WasStreaming = Streaming;
  Streaming = false;
  Mode WasMode = ControlMode;
  ControlMode = MODE_TORQUE;
  Target = Uq;

  uint32_t T0 = millis();
  while (millis() - T0 < 1500) { encoder.update(); ShaftVelocity = VelLPF(SampleVelocity());
    driver.enable(); DriverEnabled = true;
    motor.setPhaseVoltage(TORQUE_SIGN * Uq, 0, RotorElectricalAngle()); }

  double Sum = 0, Sum2 = 0; float Lo = 1e9, Hi = -1e9; long N = 0;
  T0 = millis();
  while (millis() - T0 < 3000) {
    encoder.update();
    ShaftVelocity = VelLPF(SampleVelocity());
    motor.setPhaseVoltage(TORQUE_SIGN * Uq, 0, RotorElectricalAngle());
    float V = ShaftVelocity;
    Sum += V; Sum2 += (double)V * V; N++;
    if (V < Lo) Lo = V;
    if (V > Hi) Hi = V;
  }
  Target = 0;
  motor.setPhaseVoltage(0, 0, RotorElectricalAngle());
  driver.disable(); DriverEnabled = false;

  float Mean = Sum / N;
  float Sd = sqrtf((float)(Sum2 / N - (double)Mean * Mean));
  Serial.print(F("Uq=")); Serial.print(Uq, 2);
  Serial.print(F(" mean=")); Serial.print(Mean, 2);
  Serial.print(F(" sd=")); Serial.print(Sd, 2);
  Serial.print(F(" min=")); Serial.print(Lo, 2);
  Serial.print(F(" max=")); Serial.print(Hi, 2);
  Serial.print(F(" n=")); Serial.println(N);
  ControlMode = WasMode;
  Streaming = WasStreaming;
}

// Same on-device measurement, but closing the velocity loop, so the controller
// is judged without serial traffic perturbing it.
void doVelTest(char* cmd) {
  float Tgt = cmd && cmd[0] ? atof(cmd) : 30.0f;
  bool WasStreaming = Streaming;
  Streaming = false;
  ControlMode = MODE_VELOCITY;
  VelPID.reset();
  Target = Tgt;

  uint32_t T0 = millis();
  double Sum = 0, Sum2 = 0; float Lo = 1e9, Hi = -1e9; long N = 0;
  float SettleMs = -1;
  while (millis() - T0 < 4500) {
    encoder.update();
    ShaftVelocity = VelLPF(SampleVelocity());
    float Out = VelPID(Target - ShaftVelocity);
    float Uq = TORQUE_SIGN * _constrain(Out, -VoltageLimit, VoltageLimit);
    if (!DriverEnabled) { driver.enable(); DriverEnabled = true; }
    motor.setPhaseVoltage(Uq, 0, RotorElectricalAngle());
    uint32_t T = millis() - T0;
    if (SettleMs < 0 && fabsf(ShaftVelocity - Tgt) < fabsf(Tgt) * 0.1f) SettleMs = T;
    if (T > 1500) {   // statistics after the transient
      float V = ShaftVelocity;
      Sum += V; Sum2 += (double)V * V; N++;
      if (V < Lo) Lo = V;
      if (V > Hi) Hi = V;
    }
  }
  Target = 0;
  motor.setPhaseVoltage(0, 0, RotorElectricalAngle());
  driver.disable(); DriverEnabled = false;
  VelPID.reset();

  float Mean = Sum / N;
  float Sd = sqrtf((float)(Sum2 / N - (double)Mean * Mean));
  Serial.print(F("tgt=")); Serial.print(Tgt, 1);
  Serial.print(F(" mean=")); Serial.print(Mean, 2);
  Serial.print(F(" err=")); Serial.print(Mean - Tgt, 2);
  Serial.print(F(" sd=")); Serial.print(Sd, 2);
  Serial.print(F(" min=")); Serial.print(Lo, 2);
  Serial.print(F(" max=")); Serial.print(Hi, 2);
  Serial.print(F(" settle_ms=")); Serial.println(SettleMs, 0);
  Streaming = WasStreaming;
}

// TEST ONLY: shifts the whole map by 90 electrical degrees, which puts the
// stator vector on the rotor's d-axis - full current, zero torque, a guaranteed
// stall. Used to prove the stall detector actually trips on the real thing
// rather than only on injected flags. Run 'A' afterwards to restore a good map.
// TEST ONLY. 'Z' shifts the map by -90 deg elec, putting the stator vector on
// the rotor's d-axis: full current, zero torque, a genuine stall. 'Z1' shifts
// by +90 instead, which gives REVERSE torque and produces a runaway rather than
// a stall - the two need different protections and so are tested separately.
// Run 'A' afterwards to restore a good map.
void doInjectStall(char* cmd) {
  bool Reverse = (cmd && cmd[0] == '1');
  float Shift = Reverse ? _PI_2 : -_PI_2;
  for (int i = 0; i < MAP_BINS; i++) MapRaw[i] = _normalizeAngle(MapRaw[i] + Shift);
  Serial.print(F("TEST: map shifted "));
  Serial.println(Reverse ? F("+90 deg (expect RUNAWAY)") : F("-90 deg (expect STALL)"));
}

void doClearFault(char* cmd) {
  StallFault = false;
  StallSinceMs = 0;
  RunawaySinceMs = 0;
  ReverseSinceMs = 0;
  OverCurrentSinceMs = 0;
  FaultReason = "none";
  VelPID.reset();
  Serial.println(F("stall fault cleared"));
}

void doCan(char* cmd) {
  CanControl = (cmd[0] != '0');
  Target = 0;
  Serial.println(CanControl ? F("CAN control ON") : F("CAN control OFF (serial T only)"));
}

// Pure observer: measures velocity statistics for 3 s WITHOUT touching Target,
// mode, or the driver, so whatever is currently driving the motor (CAN or
// serial) keeps driving it. Needed to compare CAN-driven against serial-driven
// on equal terms - doVelTest() commands its own target and so cannot.
// Telemetry is suppressed during the window because streaming stalls the loop.
void doObserve(char* cmd) {
  bool WasStreaming = Streaming;
  Streaming = false;
  double Sum = 0, Sum2 = 0; float Lo = 1e9, Hi = -1e9; long N = 0;
  uint32_t T0 = millis();
  while (millis() - T0 < 3000) {
    // run the normal control path exactly as loop() would
    if (CanControl) {
      ControlMode = MODE_VELOCITY;
      Target = (millis() - CanLastMillis > CAN_TIMEOUT_MS) ? 0.0f : (float)CanVelocity;
    }
    encoder.update();
    ShaftVelocity = VelLPF(SampleVelocity());
    float Uq;
    if (ControlMode == MODE_TORQUE) Uq = TORQUE_SIGN * _constrain(Target, -VoltageLimit, VoltageLimit);
    else Uq = TORQUE_SIGN * _constrain(VelPID(Target - ShaftVelocity), -VoltageLimit, VoltageLimit);
    if (!DriverEnabled && Target != 0) { driver.enable(); DriverEnabled = true; }
    if (DriverEnabled) motor.setPhaseVoltage(Uq, 0, RotorElectricalAngle());
    float V = ShaftVelocity;
    Sum += V; Sum2 += (double)V * V; N++;
    if (V < Lo) Lo = V;
    if (V > Hi) Hi = V;
  }
  float Mean = Sum / N;
  float Sd = sqrtf((float)(Sum2 / N - (double)Mean * Mean));
  Serial.print(F("obs mean=")); Serial.print(Mean, 2);
  Serial.print(F(" sd=")); Serial.print(Sd, 2);
  Serial.print(F(" min=")); Serial.print(Lo, 2);
  Serial.print(F(" max=")); Serial.print(Hi, 2);
  Serial.print(F(" src=")); Serial.println(CanControl ? "CAN" : "serial");
  Streaming = WasStreaming;
}

// Records raw shaft angle at 1 kHz while whatever is currently driving keeps
// driving. Dumping the angle rather than the velocity is the point: velocity is
// a derived quantity whose noise depends on the estimator window, so it cannot
// distinguish real motion ripple from measurement quantisation. The angle is
// the raw measurement - if it advances uniformly the motion is smooth and the
// jerk is in my estimator; if it advances in steps the motion really is jerky.
#define JIT_N 800
float JitBuf[JIT_N];
void doJitter(char* cmd) {
  bool WasStreaming = Streaming; Streaming = false;
  uint32_t Next = micros();
  for (int i = 0; i < JIT_N; i++) {
    while ((int32_t)(micros() - Next) < 0) {
      // keep the control loop running while we sample
      encoder.update();
      ShaftVelocity = VelLPF(SampleVelocity());
      float Uq;
      if (ControlMode == MODE_TORQUE) Uq = TORQUE_SIGN * _constrain(Target, -VoltageLimit, VoltageLimit);
      else Uq = TORQUE_SIGN * _constrain(VelPID(Target - ShaftVelocity), -VoltageLimit, VoltageLimit);
      if (!DriverEnabled && Target != 0) { driver.enable(); DriverEnabled = true; }
      if (DriverEnabled) motor.setPhaseVoltage(Uq, 0, RotorElectricalAngle());
    }
    JitBuf[i] = encoder.getAngle();
    Next += 1000;   // 1 kHz
  }
  Serial.println(F("--- angle trace (rad, 1 kHz) ---"));
  for (int i = 0; i < JIT_N; i++) Serial.println(JitBuf[i], 5);
  Serial.println(F("--- end ---"));
  Streaming = WasStreaming;
}

// Reads the FDCAN peripheral's own error state. This distinguishes the two
// wiring faults that look identical from the ESP32 side (both give TIMEOUT):
//   ACT=0/idle, LEC=0, REC=0  -> ESC1 sees NOTHING on the bus (open circuit,
//                                missing GND, transceiver unpowered/disabled)
//   LEC nonzero, REC climbing -> ESC1 sees traffic but cannot decode it
//                                (CANH/CANL swapped, wrong bitrate, no termination)
void doCanTerm(char* cmd) {
  int Level = (cmd && cmd[0] == '1') ? HIGH : LOW;
  pinMode(CAN_TERM_PIN, OUTPUT);
  digitalWrite(CAN_TERM_PIN, Level);
  Serial.print(F("CAN_TERM (PC14) = ")); Serial.println(Level);
}

// Counts receive errors and frames over a fixed window, so the effect of a
// wiring or termination change can be measured instead of guessed at.
void doCanQuality(char* cmd) {
  uint32_t Rec0 = (FDCAN1->ECR >> 8) & 0x7F;
  uint32_t Rx0 = CanMsgCount;
  uint32_t Errors = 0, Last = Rec0;
  uint32_t T0 = millis();
  while (millis() - T0 < 4000) {
    uint32_t R = (FDCAN1->ECR >> 8) & 0x7F;
    if (R > Last) Errors += (R - Last);      // REC rises on each receive error
    Last = R;
    delay(2);
  }
  uint32_t Rx1 = CanMsgCount;
  Serial.print(F("4s window: frames=")); Serial.print((unsigned long)(Rx1 - Rx0));
  Serial.print(F(" rec_rises=")); Serial.print((unsigned long)Errors);
  Serial.print(F(" REC=")); Serial.print((FDCAN1->ECR >> 8) & 0x7F);
  Serial.print(F(" LEC=")); Serial.print(FDCAN1->PSR & 0x7);
  Serial.print(F(" EP=")); Serial.println((FDCAN1->PSR >> 5) & 1);
}

void doCanDiag(char* cmd) {
  uint32_t Psr = FDCAN1->PSR;      // reading clears LEC to 7 ("no change")
  uint32_t Ecr = FDCAN1->ECR;
  Serial.print(F("FDCAN PSR=0x")); Serial.print(Psr, HEX);
  Serial.print(F(" LEC="));  Serial.print(Psr & 0x7);
  Serial.print(F(" ACT="));  Serial.print((Psr >> 3) & 0x3);
  Serial.print(F(" EP="));   Serial.print((Psr >> 5) & 1);
  Serial.print(F(" BO="));   Serial.print((Psr >> 7) & 1);
  Serial.print(F("  TEC=")); Serial.print(Ecr & 0xFF);
  Serial.print(F(" REC="));  Serial.print((Ecr >> 8) & 0x7F);
  Serial.print(F("  SHDN_pin=")); Serial.print(digitalRead(CAN_SHDN_PIN));
  Serial.print(F(" rx=")); Serial.println((unsigned long)CanMsgCount);
}

// Drives the bus from the ESC1 end. Until now only the ESP32 has transmitted,
// so "ESC1 sees nothing" could mean an open bus OR a dead ESP32-side
// transceiver. If the ESC1 transmits and its own TEC climbs, its transceiver is
// driving but nobody is acknowledging; if TEC stays 0 the frame is not even
// leaving. Either way it narrows which end is at fault.
void doCanTx(char* cmd) {
  uint8_t Data[1] = { 0x5A };
  uint32_t Ecr0 = FDCAN1->ECR;
  Serial.print(F("TEC before=")); Serial.println(Ecr0 & 0xFF);
  for (int i = 0; i < 20; i++) {
    meFDCAN_transmit(0x321, Data, 1, 1);
    delay(25);
  }
  delay(200);
  uint32_t Ecr1 = FDCAN1->ECR;
  uint32_t Psr  = FDCAN1->PSR;
  Serial.print(F("TEC after=")); Serial.print(Ecr1 & 0xFF);
  Serial.print(F(" REC=")); Serial.print((Ecr1 >> 8) & 0x7F);
  Serial.print(F(" LEC=")); Serial.print(Psr & 0x7);
  Serial.print(F(" BO=")); Serial.println((Psr >> 7) & 1);
  Serial.println(F("TEC climbing = transceiver drives but no ACK (open bus / dead far end)"));
  Serial.println(F("TEC stuck at 0 = frames are not reaching the bus at all"));
}

void doStatus(char* cmd) {
  Serial.print(F("FAULT=")); Serial.print(StallFault ? FaultReason : "none");
  Serial.print(F(" Iest=")); Serial.print(EstimateCurrent(LastUq, ShaftVelocity), 2);
  Serial.print(F("A  "));
  Serial.print(F("can=")); Serial.print(CanControl ? "on" : "off");
  Serial.print(F(" rx=")); Serial.print((unsigned long)CanMsgCount);
  Serial.print(F(" canvel=")); Serial.print((int)CanVelocity);
  Serial.print(F(" age_ms="));
  Serial.print((unsigned long)(millis() - CanLastMillis));
  Serial.print(F("  "));
  Serial.print(F("mode=")); Serial.print(ControlMode == MODE_VELOCITY ? "vel" : "torque");
  Serial.print(F(" target=")); Serial.print(Target, 2);
  Serial.print(F(" vel=")); Serial.print(ShaftVelocity, 2);
  Serial.print(F(" angle=")); Serial.print(encoder.getAngle(), 3);
  Serial.print(F(" Vlim=")); Serial.print(VoltageLimit, 2);
  Serial.print(F(" map=")); Serial.print(MapValid ? "ok" : "INVALID");
  Serial.print(F(" Vbus=")); Serial.println(ReadBusVoltage(), 2);
}

Commander command = Commander(Serial);

void setup() {
  Serial.begin(115200);
  delay(2000);
  SimpleFOCDebug::enable(&Serial);
  analogReadResolution(12);

  encoder.init();
  encoder.enableInterrupts(doA, doB);

  driver.voltage_power_supply = 24.5f;
  driver.voltage_limit = 6;
  // Always check this. SimpleFOC's weak fallback _configure6PWM() returns
  // failure silently, and the symptom - no current, motor never moves - is
  // indistinguishable from dead hardware. See lib_archive = no in
  // platformio.ini for why that fallback can get linked in at all.
  if (!driver.init()) {
    Serial.println(F("FATAL: driver.init() failed - no PWM configured"));
    Serial.println(F("check lib_archive = no is set in platformio.ini"));
    while (1) delay(1000);
  }
  motor.linkDriver(&driver);
  motor.voltage_limit = 6;
  driver.disable();

  command.add('T', doTarget, "target: Uq volts (torque) or rad/s (velocity)");
  command.add('L', doLimit,  "voltage limit");
  command.add('C', doMode,   "C0 torque, C1 velocity");
  command.add('A', doCalib,  "re-run the position map calibration");
  command.add('D', PrintMap, "dump the position map");
  command.add('P', doP,      "velocity PID P");
  command.add('I', doI,      "velocity PID I");
  command.add('S', doStatus, "status");
  command.add('N', doCan,    "N1 CAN control on, N0 off (serial T only)");
  command.add('O', doObserve,"observe velocity stats without changing anything");
  command.add('X', doClearFault, "clear a latched stall fault");
  command.add('Z', doInjectStall, "TEST: break commutation to prove stall trip");
  command.add('J', doJitter, "record raw angle at 1 kHz and dump");
  command.add('E', doCanDiag, "FDCAN error/status registers");
  command.add('U', doCanTerm, "U1/U0 set the board's 120 ohm CAN terminator");
  command.add('Y', doCanQuality, "measure CAN frame/error rate over 4 s");
  command.add('B', doCanTx,  "transmit test frames from the ESC1 end");
  command.add('R', doStream, "toggle telemetry streaming");
  command.add('F', doTf,     "velocity low-pass Tf (s)");
  command.add('G', doLead,   "lead angle gain, elec rad per shaft rad/s");
  command.add('Q', doQuality,"on-device velocity quality test, arg = Uq");
  command.add('W', doVelTest,"on-device closed velocity-loop test, arg = rad/s");

  pinMode(CAN_SHDN_PIN, OUTPUT);
  digitalWrite(CAN_SHDN_PIN, LOW);   // transceiver active
  // PC14 must be DRIVEN, not left floating. Measured 2026-09-04 over a 4 s
  // window with the ESP32 heartbeat running:
  //   driven LOW : 26 frames, 0 errors, REC=0, LEC=0        <- clean
  //   driven HIGH:  0 frames, REC=127, LEC=2 (form errors), error-passive
  // Left floating it behaved like the HIGH case intermittently, which is what
  // produced the stuff errors and the repeated drops into bus-off.
  pinMode(CAN_TERM_PIN, OUTPUT);
  digitalWrite(CAN_TERM_PIN, LOW);
  // Filter must be added BEFORE meFDCAN_init() - init bakes in whatever is
  // queued at that point, and the library's default filter rejects everything.
  me_FDCAN_addFilter(CAN_ID_VELOCITY, CAN_ID_VELOCITY, FDCAN_FILTER_RANGE, 1);
  if (!meFDCAN_init(250, 1, PA11, PB9)) {
    Serial.println(F("FATAL: meFDCAN_init failed"));
    while (1) delay(1000);
  }
  if (!me_FDCAN_setRxInterrupt(1, 0)) {
    Serial.println(F("FATAL: CAN interrupt setup failed"));
    while (1) delay(1000);
  }

  Serial.println(F("\n=== B-G431B-ESC1 closed-loop FOC ==="));
  Serial.print(F("VBUS: ")); Serial.print(ReadBusVoltage(), 2); Serial.println(F(" V"));
  CalibrateMap();
  Serial.println(F("ready. C0/C1 to pick mode, T to set target, S for status."));
}

uint32_t LastVelUs = 0;

void loop() {
  // CAN commands the velocity when enabled. The fail-safe matters more than the
  // command: if the heartbeat stops, zero the target rather than keep spinning.
  if (CanControl) {
    ControlMode = MODE_VELOCITY;
    if (millis() - CanLastMillis > CAN_TIMEOUT_MS) {
      Target = 0;
    } else {
      Target = (float)CanVelocity;
    }
  }

  encoder.update();

  // velocity, filtered - encoder.getVelocity() holds its last value for up to
  // 100 ms after the final pulse before forcing zero, so it is filtered rather
  // than used raw
  ShaftVelocity = VelLPF(SampleVelocity());

  float Uq = 0;
  if (ControlMode == MODE_TORQUE) {
    Uq = TORQUE_SIGN * _constrain(Target, -VoltageLimit, VoltageLimit);
  } else {
    float Out = VelPID(Target - ShaftVelocity);
    Uq = TORQUE_SIGN * _constrain(Out, -VoltageLimit, VoltageLimit);
  }

  // Stall detection. Trips only when a meaningful speed is commanded, real
  // voltage is being applied, and the shaft still is not turning - so normal
  // acceleration from rest (which settles in 25-50 ms) cannot trip the 300 ms
  // timer. Latches: it stays tripped until the commanded target returns to 0
  // or 'X' is sent, so a stuck rotor cannot be repeatedly re-driven.
  // Over-current first: it is the fastest and most general of the checks.
  if (!StallFault && DriverEnabled) {
    float Iest = EstimateCurrent(Uq, ShaftVelocity);
    if (Iest > OVERCURRENT_LIMIT) {
      if (OverCurrentSinceMs == 0) OverCurrentSinceMs = millis();
      else if (millis() - OverCurrentSinceMs > OVERCURRENT_TIME_MS) {
        StallFault = true;
        FaultReason = "OVERCURRENT";
        Serial.print(F("OVERCURRENT - driver off. Iest="));
        Serial.print(Iest, 1);
        Serial.print(F("A target=")); Serial.print(Target, 1);
        Serial.print(F(" vel=")); Serial.println(ShaftVelocity, 2);
      }
    } else {
      OverCurrentSinceMs = 0;
    }
  } else {
    OverCurrentSinceMs = 0;
  }

  // Overspeed: nothing legitimate exceeds this, so trip immediately.
  if (!StallFault && fabsf(ShaftVelocity) > OVERSPEED_LIMIT) {
    StallFault = true;
    FaultReason = "OVERSPEED";
    Serial.print(F("OVERSPEED - driver off. vel="));
    Serial.println(ShaftVelocity, 1);
  }

  // Runaway: moving hard in the OPPOSITE direction to the command, and well
  // past it in magnitude. The margin keeps normal reversals clear - during a
  // legitimate reversal the speed passes through zero, it does not overshoot
  // backwards beyond twice the target.
  if (!StallFault && fabsf(Target) >= STALL_TARGET_MIN &&
      ShaftVelocity * Target < 0 &&
      fabsf(ShaftVelocity) > 2.0f * fabsf(Target) + 20.0f) {
    if (RunawaySinceMs == 0) RunawaySinceMs = millis();
    else if (millis() - RunawaySinceMs > RUNAWAY_TIME_MS) {
      StallFault = true;
      FaultReason = "RUNAWAY";
      Serial.print(F("RUNAWAY - driver off. target="));
      Serial.print(Target, 1);
      Serial.print(F(" vel=")); Serial.println(ShaftVelocity, 1);
    }
  } else {
    RunawaySinceMs = 0;
  }

  // Driving hard, but the shaft is going the wrong way and staying there.
  //
  // Uses a LEAKY ACCUMULATOR rather than a plain timer. Broken commutation
  // makes the velocity oscillate in sign, and a timer that resets on every
  // zero-crossing never reaches its threshold - that is why the runaway case
  // took 1376 ms to be caught by OVERSPEED instead of ~500 ms by this rule.
  // Bad time accumulates, good time drains at half rate, so brief sign flips
  // no longer wipe the evidence while a genuine reversal still drains clear.
  {
    static uint32_t LastTickMs = 0;
    uint32_t NowMs = millis();
    uint32_t Dt = LastTickMs ? (NowMs - LastTickMs) : 0;
    LastTickMs = NowMs;
    bool Bad = fabsf(Target) >= STALL_TARGET_MIN &&
               fabsf(Uq) > STALL_UQ_THRESH && ShaftVelocity * Target < 0;
    if (Bad) ReverseSinceMs += Dt;
    else     ReverseSinceMs = (ReverseSinceMs > Dt / 2) ? ReverseSinceMs - Dt / 2 : 0;
  }
  if (!StallFault && fabsf(Target) >= STALL_TARGET_MIN) {
    if (ReverseSinceMs > REVERSE_TIME_MS) {
      StallFault = true;
      FaultReason = "REVERSE";
      Serial.print(F("REVERSE FAULT - driver off. target="));
      Serial.print(Target, 1);
      Serial.print(F(" vel=")); Serial.print(ShaftVelocity, 2);
      Serial.print(F(" Iest=")); Serial.print(EstimateCurrent(Uq, ShaftVelocity), 1);
      Serial.println(F("A"));
    }
  }

  if (!StallFault && fabsf(Target) >= STALL_TARGET_MIN &&
      fabsf(Uq) > STALL_UQ_THRESH && fabsf(ShaftVelocity) < STALL_VEL_THRESH) {
    if (StallSinceMs == 0) StallSinceMs = millis();
    else if (millis() - StallSinceMs > STALL_TIME_MS) {
      StallFault = true;
      FaultReason = "STALL";
      Serial.print(F("STALL DETECTED - driver off. target="));
      Serial.print(Target, 1);
      Serial.print(F(" vel=")); Serial.print(ShaftVelocity, 2);
      Serial.print(F(" Uq=")); Serial.print(Uq, 2);
      Serial.print(F(" Iest=")); Serial.print(EstimateCurrent(Uq, ShaftVelocity), 1);
      Serial.println(F("A  (send X, or command 0, to clear)"));
    }
  } else {
    StallSinceMs = 0;
  }

  // A commanded stop re-arms, so the ESP32 sending 0 clears the latch without
  // needing serial access - matching how the CAN fail-safe already behaves.
  if (StallFault && fabsf(Target) < 0.5f) {
    StallFault = false;
    FaultReason = "none";
    RunawaySinceMs = 0;
    ReverseSinceMs = 0;
    OverCurrentSinceMs = 0;
    VelPID.reset();
  }

  if (StallFault) {
    Uq = 0;
    VelPID.reset();
  }
  LastUq = Uq;

  // enable/disable ONLY on transition. BLDCDriver6PWM::enable() ends with
  // setPwm(0,0,0), so calling it every iteration zeroed the PWM registers
  // immediately before setPhaseVoltage() rewrote them - chopping the real
  // output and cutting 1.5 V from ~62 rad/s down to ~24 with heavy ripple and
  // intermittent stalling.
  // Never drive on an invalid map: commutation would be arbitrary, which means
  // full current into a locked rotor.
  bool WantEnabled = MapValid && !StallFault &&
                     !(Target == 0 && fabsf(ShaftVelocity) < 0.5f);
  if (WantEnabled != DriverEnabled) {
    if (WantEnabled) {
      driver.enable();
    } else {
      driver.disable();
      VelPID.reset();
    }
    DriverEnabled = WantEnabled;
  }
  if (DriverEnabled) motor.setPhaseVoltage(Uq, 0, RotorElectricalAngle());
  StreamTelemetry(Uq);

  command.run();
}
