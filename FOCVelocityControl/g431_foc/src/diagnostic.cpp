// B-G431B-ESC1 - open-loop calibration diagnostic
//
// This is deliberately NOT a control sketch. It runs the motor open loop
// only, and measures the three things every closed-loop failure so far has
// depended on but that have never been measured cleanly:
//
//   1. the real pole pair count,
//   2. whether the encoder scale (PPR/CPR) is right,
//   3. whether encoder counts survive PWM switching.
//
// Why the existing evidence can't answer these: SimpleFOC's own "PP check"
// (FOCMotor.cpp:899) reports 2*PI/moved, where `moved` is the shaft angle
// travelled during ONE electrical revolution. A single electrical rev is
// ~0.5 rad of shaft at 12pp - small enough that cogging and where the rotor
// happens to settle dominate the number. That is exactly why it has been
// reporting 15.15 one boot and 6.67 another on identical hardware: the
// measurement is real but far too short to be trusted.
//
// Sweeping many electrical revolutions and dividing at the end averages the
// settling error away, so the ratio converges on the true value.

#include <Arduino.h>
#include <SimpleFOC.h>
// Reached directly because the library's own failure paths in _configure6PWM()
// return SIMPLEFOC_DRIVER_INIT_FAILED without printing anything, so a failed
// driver init is otherwise completely silent.
#include "drivers/hardware_specific/stm32/stm32_searchtimers.h"

#define ENCODER_PPR 1000  // stated 1000 PPR incremental -> cpr = 4000

// B-G431B-ESC1 VBUS sense divider. Approximate - the boot printout exists so
// this can be calibrated against the bench supply's own display.
#define VBUS_DIVIDER 10.39f

Encoder encoder = Encoder(PB6, PB7, ENCODER_PPR);

// Counted in the ISRs themselves. This separates "the interrupt never fires"
// from "it fires but the quadrature decode rejects the edge" - Encoder::handleA
// only moves pulse_counter when the pin state actually differs from its cached
// A_active, so a firing ISR can still leave the count at zero.
volatile uint32_t IsrCountA = 0;
volatile uint32_t IsrCountB = 0;
void doA() { IsrCountA++; encoder.handleA(); }
void doB() { IsrCountB++; encoder.handleB(); }

// pole_pairs is a placeholder here on purpose: setPhaseVoltage() does not use
// it, and none of the math below reads it. Nothing in this sketch assumes 6
// or 12 - the sweep measures the ratio from scratch.
BLDCMotor motor = BLDCMotor(12);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,
                                       A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// The board's own low-side shunts, unused until now. This is the only
// instrument that can distinguish the two remaining explanations: if the field
// really is 90 degrees ahead of the rotor then Iq carries the current and Id is
// near zero, and a stall means there is genuinely not enough current to make
// torque. If instead Id carries the current, the vector is sitting on the rotor
// and the commutation angle is wrong no matter what the geometry says.
// Shunt 3 mOhm, PGA gain -64/7, per the board's reference design.
LowsideCurrentSense currentSense =
    LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
bool CurrentSenseOk = false;

// Tunable over serial so a whole test matrix runs without a reflash.
float SweepVoltage = 1.5f;
int   SweepRevs    = 24;
// Sweep speed. The whole point of making this tunable: the earlier "no lost
// counts" result was taken at 2000us/step = ~1.3 rad/s, while the closed-loop
// failure happens at ~23 rad/s. Lost quadrature counts are a rate-dependent
// failure, so a clean result at crawl speed proves nothing about 23 rad/s.
int   StepDelayUs  = 2000;

float ReadBusVoltage() {
  return analogRead(A_VBUS) * (3.3f / 4095.0f) * VBUS_DIVIDER;
}

// Park the field at a fixed electrical angle and let the rotor stop ringing.
// Holding (rather than cutting drive) matters: with drive removed the rotor
// relaxes into the nearest cogging detent, which is not where the field put it.
void HoldAndSettle(float ElecAngle, int Millis) {
  uint32_t Until = millis() + Millis;
  while ((int32_t)(millis() - Until) < 0) {
    motor.setPhaseVoltage(SweepVoltage, 0, ElecAngle);
    encoder.update();
    delayMicroseconds(200);
  }
}

void PrintRow(const char* Dir, int Rev, float Shaft, float Delta, float Cum) {
  // Cumulative estimate, not per-rev: pole pairs = electrical revs travelled
  // per mechanical rev, so after N electrical revs the shaft must have moved
  // N*2PI/pp. Per-rev deltas are noisy from cogging; the cumulative figure is
  // the one that converges.
  float PpEst = (fabsf(Cum) > 1e-4f) ? (Rev * _2PI / fabsf(Cum)) : 0.0f;
  Serial.print(Dir);        Serial.print('\t');
  Serial.print(Rev);        Serial.print('\t');
  Serial.print(Shaft, 4);   Serial.print('\t');
  Serial.print(Delta, 4);   Serial.print('\t');
  Serial.print(Cum, 4);     Serial.print('\t');
  Serial.print(PpEst, 3);   Serial.print('\t');
  Serial.println(ReadBusVoltage(), 2);
}

void RunSweep() {
  const int StepsPerRev = 200;

  Serial.println(F("\n# === open-loop electrical sweep ==="));
  Serial.print(F("# sweep_voltage="));  Serial.print(SweepVoltage, 2);
  Serial.print(F("V  elec_revs="));     Serial.print(SweepRevs);
  Serial.print(F("  ppr="));            Serial.print(ENCODER_PPR);
  Serial.print(F("  cpr="));            Serial.print(encoder.cpr);
  Serial.print(F("  step_delay_us="));  Serial.println(StepDelayUs);

  driver.enable();

  float El = _3PI_2;
  HoldAndSettle(El, 600);
  encoder.update();
  const float Start = encoder.getAngle();

  Serial.println(F("dir\trev\tshaft\tdelta\tcum\tpp_est\tVbus"));

  float Prev = Start;
  uint32_t FwdStartMs = millis();
  for (int Rev = 1; Rev <= SweepRevs; Rev++) {
    for (int s = 0; s < StepsPerRev; s++) {
      El += _2PI / StepsPerRev;
      motor.setPhaseVoltage(SweepVoltage, 0, El);
      encoder.update();
      delayMicroseconds(StepDelayUs);
    }
    HoldAndSettle(El, 150);
    encoder.update();
    float A = encoder.getAngle();
    PrintRow("fwd", Rev, A, A - Prev, A - Start);
    Prev = A;
  }
  const float FwdEnd = Prev;
  const uint32_t FwdMs = millis() - FwdStartMs;

  for (int Rev = 1; Rev <= SweepRevs; Rev++) {
    for (int s = 0; s < StepsPerRev; s++) {
      El -= _2PI / StepsPerRev;
      motor.setPhaseVoltage(SweepVoltage, 0, El);
      encoder.update();
      delayMicroseconds(StepDelayUs);
    }
    HoldAndSettle(El, 150);
    encoder.update();
    float A = encoder.getAngle();
    PrintRow("rev", Rev, A, A - Prev, A - FwdEnd);
    Prev = A;
  }
  const float End = Prev;

  motor.setPhaseVoltage(0, 0, El);
  driver.disable();

  const float FwdMoved = fabsf(FwdEnd - Start);
  const float RevMoved = fabsf(End - FwdEnd);
  const float ReturnErr = End - Start;

  Serial.println(F("\n# --- summary ---"));
  Serial.print(F("# forward: "));  Serial.print(SweepRevs);
  Serial.print(F(" elec rev -> ")); Serial.print(FwdMoved, 4);
  Serial.print(F(" rad shaft  => pole_pairs = "));
  Serial.println(SweepRevs * _2PI / FwdMoved, 3);
  Serial.print(F("# reverse: "));  Serial.print(SweepRevs);
  Serial.print(F(" elec rev -> ")); Serial.print(RevMoved, 4);
  Serial.print(F(" rad shaft  => pole_pairs = "));
  Serial.println(SweepRevs * _2PI / RevMoved, 3);
  Serial.print(F("# mech revs travelled each way: "));
  Serial.println(FwdMoved / _2PI, 3);
  Serial.print(F("# forward sweep took ")); Serial.print(FwdMs);
  Serial.print(F(" ms -> mean shaft speed "));
  Serial.print(FwdMoved / (FwdMs / 1000.0f), 2);
  Serial.println(F(" rad/s"));
  // Forward and back is the same physical path, so any shaft angle left over
  // is encoder counts that were gained or lost rather than real movement.
  Serial.print(F("# return error: ")); Serial.print(ReturnErr, 4);
  Serial.print(F(" rad = ")); Serial.print(ReturnErr * encoder.cpr / _2PI, 1);
  Serial.println(F(" counts"));
  Serial.println(F("# (return error near 0 = no lost counts; a large or"));
  Serial.println(F("#  growing value = counts dropped under PWM switching)"));
}

// Rotor held still by a fixed field, driver switching hard. Any encoder angle
// change here is not motion - it is switching noise being counted as edges.
void RunNoiseTest() {
  Serial.println(F("\n# === static noise test (10s, rotor held by field) ==="));
  driver.enable();
  const float El = _3PI_2;
  HoldAndSettle(El, 800);
  encoder.update();
  const float Base = encoder.getAngle();
  Serial.println(F("t_ms\tdrift_rad\tdrift_counts\tVbus"));
  for (int i = 0; i < 20; i++) {
    HoldAndSettle(El, 500);
    encoder.update();
    float D = encoder.getAngle() - Base;
    Serial.print((i + 1) * 500);              Serial.print('\t');
    Serial.print(D, 5);                       Serial.print('\t');
    Serial.print(D * encoder.cpr / _2PI, 1);  Serial.print('\t');
    Serial.println(ReadBusVoltage(), 2);
  }
  motor.setPhaseVoltage(0, 0, El);
  driver.disable();
  Serial.println(F("# nonzero drift with a stationary rotor = electrical noise on PB6/PB7"));
}

// Driver off, encoder only - the hand-turn scale check.
void RunEncoderMonitor() {
  driver.disable();
  Serial.println(F("\n# === encoder monitor - turn the shaft by hand, any key stops ==="));
  Serial.println(F("# one full revolution must read 6.2832 rad / 4000 counts"));
  while (!Serial.available()) {
    encoder.update();
    float A = encoder.getAngle();
    Serial.print(A, 4);
    Serial.print(F(" rad\t"));
    Serial.print(A * encoder.cpr / _2PI, 1);
    Serial.println(F(" counts"));
    delay(200);
  }
  while (Serial.available()) Serial.read();
}

// Driver off. Shows the encoder signal chain at three levels at once: the raw
// pin voltages, whether the edges reach the ISRs, and whether the decoder turns
// them into counts. Whichever level stays frozen is where the chain is broken.
void RunGpioMonitor() {
  driver.disable();
  Serial.println(F("\n# === encoder signal chain - turn the shaft by hand, any key stops ==="));
  Serial.println(F("# A/B = raw pin level, isrA/isrB = interrupts seen, angle = decoded"));
  Serial.println(F("A\tB\tisrA\tisrB\tangle\tcounts"));
  while (!Serial.available()) {
    encoder.update();
    float Ang = encoder.getAngle();
    Serial.print(digitalRead(PB6));                 Serial.print('\t');
    Serial.print(digitalRead(PB7));                 Serial.print('\t');
    Serial.print(IsrCountA);                        Serial.print('\t');
    Serial.print(IsrCountB);                        Serial.print('\t');
    Serial.print(Ang, 4);                           Serial.print('\t');
    Serial.println(Ang * encoder.cpr / _2PI, 1);
    delay(250);
  }
  while (Serial.available()) Serial.read();
}

// Re-init the encoder pins with the MCU's internal pullups. An open-collector
// or push-pull-to-ground encoder with no pullup anywhere sits at a constant
// level and produces exactly the dead-flat zero reading we are looking at.
void EnablePullups() {
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  IsrCountA = IsrCountB = 0;
  Serial.println(F("encoder re-inited with INTERNAL pullups"));
}

// Rotate the field slowly with drive on, and report whether the shaft moved -
// the open-loop sanity check, with VBUS sag as an independent witness that
// current is actually flowing.
void RunDriveTest() {
  Serial.println(F("\n# === drive test: field rotates 5 elec revs, WATCH THE SHAFT ==="));
  Serial.print(F("# voltage=")); Serial.println(SweepVoltage, 2);
  driver.enable();
  float El = _3PI_2;
  HoldAndSettle(El, 500);
  encoder.update();
  float Start = encoder.getAngle();
  uint32_t IsrStart = IsrCountA + IsrCountB;
  float VbusMin = 99.0f;
  for (int s = 0; s < 5 * 200; s++) {
    El += _2PI / 200.0f;
    motor.setPhaseVoltage(SweepVoltage, 0, El);
    encoder.update();
    float V = ReadBusVoltage();
    if (V < VbusMin) VbusMin = V;
    delayMicroseconds(3000);
  }
  HoldAndSettle(El, 300);
  encoder.update();
  float Moved = encoder.getAngle() - Start;
  motor.setPhaseVoltage(0, 0, El);
  driver.disable();
  if (CurrentSenseOk) {
    DQCurrent_s C = currentSense.getFOCCurrents(El);
    Serial.print(F("# open-loop currents at end: Id=")); Serial.print(C.d, 3);
    Serial.print(F(" Iq=")); Serial.print(C.q, 3);
    Serial.print(F(" |I|=")); Serial.println(sqrtf(C.d*C.d + C.q*C.q), 3);
  }
  Serial.print(F("# encoder moved: ")); Serial.print(Moved, 4);
  Serial.print(F(" rad, isr edges: ")); Serial.println(IsrCountA + IsrCountB - IsrStart);
  Serial.print(F("# VBUS min under load: ")); Serial.print(VbusMin, 2);
  Serial.print(F(" V (idle ")); Serial.print(ReadBusVoltage(), 2); Serial.println(F(" V)"));
  Serial.println(F("# did the shaft physically turn? that vs. the numbers above is the answer"));
}

// Continuous out-and-back at a chosen speed, with no per-revolution dwell, to
// test encoder integrity at the speed where closed loop actually fails (~23
// rad/s) rather than at the crawl speed the first sweep used.
//
// Reading the residual: the field ends exactly where it started, so the rotor
// must too. One electrical revolution of rotor slip = 2PI/12 rad = 333.3
// counts. A residual near a multiple of 333 means the rotor slipped poles
// (a torque problem). A small residual that is NOT such a multiple means the
// encoder gained or lost edges (a counting problem).
void RunSpeedIntegrity() {
  const int StepsPerRev = 200;
  Serial.println(F("\n# === high-speed encoder integrity ==="));
  Serial.print(F("# elec_revs=")); Serial.print(SweepRevs);
  Serial.print(F("  step_delay_us=")); Serial.print(StepDelayUs);
  Serial.print(F("  voltage=")); Serial.println(SweepVoltage, 2);

  driver.enable();
  float El = _3PI_2;
  HoldAndSettle(El, 600);
  encoder.update();
  const float Start = encoder.getAngle();

  uint32_t T0 = micros();
  for (int i = 0; i < SweepRevs * StepsPerRev; i++) {
    El += _2PI / StepsPerRev;
    motor.setPhaseVoltage(SweepVoltage, 0, El);
    encoder.update();
    delayMicroseconds(StepDelayUs);
  }
  uint32_t FwdUs = micros() - T0;
  HoldAndSettle(El, 400);
  encoder.update();
  const float Mid = encoder.getAngle();

  for (int i = 0; i < SweepRevs * StepsPerRev; i++) {
    El -= _2PI / StepsPerRev;
    motor.setPhaseVoltage(SweepVoltage, 0, El);
    encoder.update();
    delayMicroseconds(StepDelayUs);
  }
  HoldAndSettle(El, 400);
  encoder.update();
  const float End = encoder.getAngle();

  motor.setPhaseVoltage(0, 0, El);
  driver.disable();

  const float Moved = fabsf(Mid - Start);
  const float Speed = Moved / (FwdUs / 1e6f);
  const float ResidRad = End - Start;
  const float ResidCounts = ResidRad * encoder.cpr / _2PI;
  const float ElecRevCounts = encoder.cpr / 12.0f;  // 333.3 counts per elec rev

  Serial.print(F("# forward moved ")); Serial.print(Moved, 4);
  Serial.print(F(" rad in ")); Serial.print(FwdUs / 1000.0f, 1);
  Serial.print(F(" ms -> mean speed ")); Serial.print(Speed, 2);
  Serial.println(F(" rad/s"));
  Serial.print(F("# expected if tracking: ")); Serial.print(SweepRevs * _2PI / 12.0f, 4);
  Serial.print(F(" rad  (tracked ")); Serial.print(100.0f * Moved / (SweepRevs * _2PI / 12.0f), 1);
  Serial.println(F("%)"));
  Serial.print(F("# residual: ")); Serial.print(ResidRad, 4);
  Serial.print(F(" rad = ")); Serial.print(ResidCounts, 1);
  Serial.print(F(" counts = ")); Serial.print(ResidCounts / ElecRevCounts, 2);
  Serial.println(F(" electrical revs"));
  Serial.println(F("# near a whole number of elec revs -> pole slip (torque)"));
  Serial.println(F("# small and NOT a multiple            -> lost encoder counts"));
}

// --- offset calibration from the whole sweep, not from one settling event ----
//
// During an open-loop sweep the rotor's d-axis sits where the stator vector put
// it. setPhaseVoltage(Uq, 0, ElCmd) places that vector at ElCmd + PI/2, so the
// rotor's true electrical angle is ElCmd + PI/2 at every step. That gives one
// estimate of the offset per step instead of the single one alignSensor() takes,
// and averaging thousands of them circularly makes cogging cancel instead of
// biasing the answer.
//
// Sweeping both directions and averaging also cancels the load-angle lag: the
// rotor trails the field going forward and leads it coming back by the same
// amount.
#define CAL_PP 12

float CalZeroSin = 0, CalZeroCos = 0;
long  CalZeroN = 0;

void AccumulateZero(float ElCmd, float Mech) {
  float Cand = _normalizeAngle(-(float)CAL_PP * Mech - ElCmd - _PI_2);
  CalZeroSin += sinf(Cand);
  CalZeroCos += cosf(Cand);
  CalZeroN++;
}

float SweepForZero(int Dir, int ElecRevs) {
  const int StepsPerRev = 200;
  CalZeroSin = CalZeroCos = 0; CalZeroN = 0;
  float El = _3PI_2;
  HoldAndSettle(El, 600);
  for (int i = 0; i < ElecRevs * StepsPerRev; i++) {
    El += Dir * _2PI / StepsPerRev;
    motor.setPhaseVoltage(SweepVoltage, 0, El);
    encoder.update();
    delayMicroseconds(StepDelayUs);
    AccumulateZero(El, encoder.getMechanicalAngle());
  }
  return _normalizeAngle(atan2f(CalZeroSin / CalZeroN, CalZeroCos / CalZeroN));
}

// Minimal closed-loop torque commutation, deliberately not using BLDCMotor:
// read the encoder, compute the rotor's electrical angle, put the stator vector
// 90 degrees ahead of it. If this spins and BLDCMotor does not, the fault is in
// the motor-layer configuration rather than in the commutation itself.
void RunMyClosedLoop(float Uq, float Zero, int Ms) {
  Serial.print(F("# closed loop: Uq=")); Serial.print(Uq, 2);
  Serial.print(F(" zero=")); Serial.print(Zero, 4);
  Serial.print(F(" for ")); Serial.print(Ms); Serial.println(F(" ms"));
  driver.enable();
  encoder.update();
  float StartAngle = encoder.getAngle();
  uint32_t T0 = millis();
  uint32_t Last = millis();
  while (millis() - T0 < (uint32_t)Ms) {
    encoder.update();
    float El = _normalizeAngle(-(float)CAL_PP * encoder.getMechanicalAngle() - Zero);
    motor.setPhaseVoltage(Uq, 0, El);
    if (millis() - Last >= 100) {
      Last = millis();
      Serial.print(F("  t=")); Serial.print(millis() - T0);
      Serial.print(F(" angle=")); Serial.print(encoder.getAngle(), 3);
      Serial.print(F(" el=")); Serial.print(El, 3);
      if (CurrentSenseOk) {
        DQCurrent_s C = currentSense.getFOCCurrents(El);
        // atan2(q,d) is the angle of the current vector relative to the rotor.
        // 90 deg = all torque, 0 deg = all magnetising force and no torque.
        Serial.print(F(" Id=")); Serial.print(C.d, 3);
        Serial.print(F(" Iq=")); Serial.print(C.q, 3);
        Serial.print(F(" |I|=")); Serial.print(sqrtf(C.d*C.d + C.q*C.q), 3);
        Serial.print(F(" ang=")); Serial.print(atan2f(C.q, C.d) * 180.0f / _PI, 1);
      }
      Serial.print(F(" Vbus=")); Serial.println(ReadBusVoltage(), 2);
    }
  }
  motor.setPhaseVoltage(0, 0, 0);
  driver.disable();
  encoder.update();
  float Travel = encoder.getAngle() - StartAngle;
  Serial.print(F("# travelled ")); Serial.print(Travel, 3);
  Serial.print(F(" rad = ")); Serial.print(Travel / _2PI, 2);
  Serial.print(F(" rev, mean speed ")); Serial.print(Travel / (Ms / 1000.0f), 2);
  Serial.println(F(" rad/s"));
}

// Repeat of the offset sweep, but with my own commutation loop and on a rail
// that is known-good. The earlier version of this ran through BLDCMotor and,
// crucially, may have run on a collapsed 4.7 V rail - so its "no offset works"
// conclusion cannot be trusted. Each offset gets a full second of drive from a
// standstill, and travel is what counts: a real commutation angle produces
// continuous rotation, not a lurch into the next detent.
// --- position-error map ------------------------------------------------------
// During a slow open-loop sweep the rotor's true electrical angle is known:
// setPhaseVoltage(Uq,0,ElCmd) puts the stator vector at ElCmd+PI/2 and the rotor
// d-axis sits there. So at every step we can compare the electrical angle the
// ENCODER predicts against the electrical angle the rotor is ACTUALLY at:
//
//   predicted = -pp*mech            (constant offset folded out later)
//   actual    = ElCmd + PI/2
//   error     = predicted - actual
//
// Binned by mechanical position and averaged circularly. Sweeping both
// directions and averaging cancels the load angle, which is direction-dependent
// while a genuine sensor error is not.
//
// The number that matters is the SPREAD of this error across the revolution.
// A constant offset is harmless - alignSensor() exists to absorb it. A spread
// approaching +-PI/2 is fatal: torque reverses sign as the shaft turns, which is
// exactly "accelerates briefly, then stalls, at every offset".
#define MAP_BINS 64
float MapSin[MAP_BINS], MapCos[MAP_BINS];
long  MapN[MAP_BINS];
bool  MapValid = false;
float MapErr[MAP_BINS];   // mean removed, for reporting the spread
float MapRaw[MAP_BINS];   // absolute per-bin error, used for compensation

void MapAccumulate(float ElCmd, float Mech) {
  float Err = _normalizeAngle(-(float)CAL_PP * Mech - (ElCmd + _PI_2));
  // Encoder::update() computes angle_prev as (counter % cpr), and C++ keeps the
  // sign of the dividend, so getMechanicalAngle() spans (-2PI, 2PI) rather than
  // [0, 2PI). Binning it raw sent every negative-count sample into bin 0.
  float M = Mech;
  if (M < 0) M += _2PI;
  int B = (int)(M / _2PI * MAP_BINS);
  if (B < 0) B = 0;
  if (B >= MAP_BINS) B = MAP_BINS - 1;
  MapSin[B] += sinf(Err);
  MapCos[B] += cosf(Err);
  MapN[B]++;
}

void MapSweep(int Dir, int ElecRevs) {
  const int StepsPerRev = 200;
  float El = _3PI_2;
  HoldAndSettle(El, 600);
  for (int i = 0; i < ElecRevs * StepsPerRev; i++) {
    El += Dir * _2PI / StepsPerRev;
    motor.setPhaseVoltage(SweepVoltage, 0, El);
    encoder.update();
    delayMicroseconds(StepDelayUs);
    MapAccumulate(El, encoder.getMechanicalAngle());
  }
}

void RunPositionMap() {
  Serial.println(F("\n# === encoder position-error map ==="));
  Serial.print(F("# Uq=")); Serial.print(SweepVoltage, 2);
  Serial.print(F("  Vbus=")); Serial.println(ReadBusVoltage(), 2);
  for (int i = 0; i < MAP_BINS; i++) { MapSin[i] = MapCos[i] = 0; MapN[i] = 0; }

  driver.enable();
  MapSweep(+1, 24);
  MapSweep(-1, 24);
  motor.setPhaseVoltage(0, 0, 0);
  driver.disable();

  // circular mean per bin, then remove the overall mean so only the
  // position-dependent part remains
  float GS = 0, GC = 0;
  for (int i = 0; i < MAP_BINS; i++) {
    if (!MapN[i]) { MapErr[i] = 0; continue; }
    MapErr[i] = atan2f(MapSin[i], MapCos[i]);
    MapRaw[i] = MapErr[i];
    GS += MapSin[i]; GC += MapCos[i];
  }
  float Mean = atan2f(GS, GC);
  Serial.print(F("# mean offset (absorbed by alignment) = "));
  Serial.println(Mean, 4);
  Serial.println(F("bin\tmech_rad\terr_elec_rad\tsamples"));
  float Lo = 99, Hi = -99;
  for (int i = 0; i < MAP_BINS; i++) {
    float E = MapErr[i] - Mean;
    while (E >  _PI) E -= _2PI;
    while (E < -_PI) E += _2PI;
    MapErr[i] = E;
    if (MapN[i]) { if (E < Lo) Lo = E; if (E > Hi) Hi = E; }
    Serial.print(i);                        Serial.print('\t');
    Serial.print(_2PI * i / MAP_BINS, 4);   Serial.print('\t');
    Serial.print(E, 4);                     Serial.print('\t');
    Serial.println(MapN[i]);
  }
  MapValid = true;
  Serial.print(F("# spread across revolution: ")); Serial.print(Lo, 4);
  Serial.print(F(" .. ")); Serial.print(Hi, 4);
  Serial.print(F(" rad electrical  (peak-to-peak "));
  Serial.print(Hi - Lo, 4); Serial.println(F(")"));
  Serial.println(F("# < 0.5 rad p-p: sensor is fine, look elsewhere"));
  Serial.println(F("# > 1.5 rad p-p: torque reverses mid-revolution, this is the fault"));
}

// Interpolated lookup of the measured error, wrapping correctly at the seam
// between the last and first bin.
float MapLookup(float Mech) {
  float M = Mech;
  if (M < 0) M += _2PI;
  float F = M / _2PI * MAP_BINS;
  // NB: not B0/B1 - Arduino's binary.h defines those as numeric constants
  int Bin0 = ((int)F) % MAP_BINS;
  int Bin1 = (Bin0 + 1) % MAP_BINS;
  float Frac = F - (int)F;
  float Lo = MapRaw[Bin0], Hi = MapRaw[Bin1];
  float D = Hi - Lo;
  while (D >  _PI) D -= _2PI;
  while (D < -_PI) D += _2PI;
  return Lo + Frac * D;
}

// Closed loop using the measured map to correct the encoder's electrical angle.
// The map stores err = (-pp*mech) - rotor_electrical, so the rotor's true
// electrical angle is (-pp*mech - err). If this spins where the uncompensated
// loop stalls, the map is real and the sensor mapping was the fault.
void RunCompensatedLoop(char* cmd) {
  if (!MapValid) {
    Serial.println(F("# run M first to measure the map"));
    return;
  }
  const float Uq = SweepVoltage;
  Serial.print(F("\n# === compensated closed loop, Uq=")); Serial.print(Uq, 2);
  Serial.println(F(" ==="));
  driver.enable();
  encoder.update();
  float A0 = encoder.getAngle();
  uint32_t T0 = millis(), Last = 0;
  while (millis() - T0 < 4000) {
    encoder.update();
    float Mech = encoder.getMechanicalAngle();
    float El = _normalizeAngle(-(float)CAL_PP * Mech - MapLookup(Mech));
    motor.setPhaseVoltage(Uq, 0, El);
    if (millis() - Last >= 250) {
      Last = millis();
      Serial.print(F("  t=")); Serial.print(millis() - T0);
      Serial.print(F(" angle=")); Serial.print(encoder.getAngle(), 3);
      Serial.print(F(" el=")); Serial.print(El, 3);
      Serial.print(F(" Vbus=")); Serial.println(ReadBusVoltage(), 2);
    }
  }
  motor.setPhaseVoltage(0, 0, 0);
  driver.disable();
  encoder.update();
  float Travel = encoder.getAngle() - A0;
  Serial.print(F("# travelled ")); Serial.print(Travel, 3);
  Serial.print(F(" rad = ")); Serial.print(Travel / _2PI, 2);
  Serial.print(F(" rev, mean speed ")); Serial.print(Travel / 4.0f, 2);
  Serial.println(F(" rad/s"));
}

void RunOffsetSweep() {
  const int Steps = 24;
  Serial.println(F("\n# === offset sweep with independent commutation ==="));
  Serial.print(F("# Uq=")); Serial.print(SweepVoltage, 2);
  Serial.print(F("  Vbus=")); Serial.println(ReadBusVoltage(), 2);
  Serial.println(F("zero\ttravel_rad\trevs\tmean_rad_s\tVbus_min"));
  for (int i = 0; i < Steps; i++) {
    float Z = _2PI * i / Steps;
    driver.enable();
    encoder.update();
    float A0 = encoder.getAngle();
    float VbusMin = 99;
    uint32_t T0 = millis();
    while (millis() - T0 < 1000) {
      encoder.update();
      float El = _normalizeAngle(-(float)CAL_PP * encoder.getMechanicalAngle() - Z);
      motor.setPhaseVoltage(SweepVoltage, 0, El);
      float V = ReadBusVoltage();
      if (V < VbusMin) VbusMin = V;
    }
    motor.setPhaseVoltage(0, 0, 0);
    driver.disable();
    encoder.update();
    float Travel = encoder.getAngle() - A0;
    Serial.print(Z, 3);            Serial.print('\t');
    Serial.print(Travel, 3);       Serial.print('\t');
    Serial.print(Travel / _2PI, 2);Serial.print('\t');
    Serial.print(Travel, 2);       Serial.print('\t');
    Serial.println(VbusMin, 2);
    delay(300);
  }
  Serial.println(F("# a correct offset should give many rad of continuous travel"));
}

void RunCalibrateAndTest() {
  Serial.println(F("\n# === offset calibration from full sweep ==="));
  driver.enable();
  float ZFwd = SweepForZero(+1, 12);
  Serial.print(F("# zero from forward sweep: ")); Serial.println(ZFwd, 4);
  float ZRev = SweepForZero(-1, 12);
  Serial.print(F("# zero from reverse sweep: ")); Serial.println(ZRev, 4);
  // circular mean of the two, so the load-angle lag cancels
  float Zero = _normalizeAngle(atan2f(sinf(ZFwd) + sinf(ZRev), cosf(ZFwd) + cosf(ZRev)));
  Serial.print(F("# calibrated zero_electric_angle = ")); Serial.println(Zero, 4);
  Serial.print(F("# (half-spread between sweeps = "));
  float Diff = ZFwd - ZRev;
  while (Diff >  _PI) Diff -= _2PI;
  while (Diff < -_PI) Diff += _2PI;
  Serial.print(fabsf(Diff) / 2, 4); Serial.println(F(" rad = load angle)"));
  motor.setPhaseVoltage(0, 0, 0);
  driver.disable();
  delay(500);
  RunMyClosedLoop(SweepVoltage, Zero, 3000);
}

void PrintHelp() {
  Serial.println(F("\ncommands:"));
  Serial.println(F("  S      run the open-loop electrical sweep"));
  Serial.println(F("  N      static noise test (is PWM corrupting encoder counts?)"));
  Serial.println(F("  E      encoder monitor for the hand-turn scale check"));
  Serial.println(F("  G      encoder signal chain: raw pins + ISR counts + decoded angle"));
  Serial.println(F("  D      drive test: rotate the field 5 elec revs, watch the shaft"));
  Serial.println(F("  U      re-init encoder with internal pullups"));
  Serial.println(F("  B      read bus voltage"));
  Serial.println(F("  V<x>   set sweep voltage, e.g. V2.5"));
  Serial.println(F("  R<n>   set sweep electrical revolutions, e.g. R48"));
  Serial.println(F("  F<us>  set per-step delay (speed), e.g. F120 for ~23 rad/s"));
  Serial.println(F("  H      high-speed encoder integrity test (continuous, no dwell)"));
  Serial.println(F("  C      calibrate zero_electric_angle from the sweep, then closed-loop test"));
  Serial.println(F("  O      sweep commutation offset with independent loop, report travel"));
  Serial.println(F("  M      map encoder position error vs shaft angle"));
  Serial.println(F("  K      closed loop using the measured map (run M first)"));
  Serial.println(F("  ?      this help"));
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  analogReadResolution(12);

  // Without this SimpleFOC's own init diagnostics go nowhere, which is how a
  // failed _configure6PWM() can look identical to a working driver that simply
  // produces no torque.
  SimpleFOCDebug::enable(&Serial);
  SimpleFOCDebug::println("SimpleFOCDebug is alive");

  // Which timer/channel the library picks for each of the six PWM pins, and
  // the score it assigns. score<0 = no workable combination (init fails),
  // score<10 = true hardware 6-PWM, >=10 = software fallback.
  {
    int Pins[6] = { A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,
                    A_PHASE_VL, A_PHASE_WH, A_PHASE_WL };
    PinMap* Timers[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
    int Score = stm32_findBestTimerCombination(6, Pins, Timers);
    Serial.print(F("timer search score: ")); Serial.println(Score);
    const char* Names[6] = {"UH","UL","VH","VL","WH","WL"};
    for (int i = 0; i < 6; i++) {
      Serial.print(F("  ")); Serial.print(Names[i]);
      Serial.print(F(" pin=")); Serial.print(Pins[i]);
      if (!Timers[i]) { Serial.println(F("  -> NO TIMER")); continue; }
      Serial.print(F("  tim=0x"));
      Serial.print((uint32_t)Timers[i]->peripheral, HEX);
      Serial.print(F("  ch=")); Serial.print(STM_PIN_CHANNEL(Timers[i]->function));
      Serial.print(F("  inverted=")); Serial.println(STM_PIN_INVERTED(Timers[i]->function));
    }
  }

  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  // Declared supply voltage sets the modulation scaling, so if it disagrees
  // with reality every commanded voltage is wrong by that ratio. The measured
  // value is printed right below it as a cross-check.
  driver.voltage_power_supply = 24;
  driver.voltage_limit = 6;
  int DriverOk = driver.init();
  Serial.print(F("driver.init() returned: ")); Serial.print(DriverOk);
  Serial.print(F("   driver.initialized: ")); Serial.println(driver.initialized);
  motor.linkDriver(&driver);
  motor.voltage_limit = 6;

  // Low-side current sensing is DISABLED on purpose. SimpleFOC 2.4.0 reports
  // "Low-side cs not supported!" for this board, and worse, the attempt is not
  // free: _configureADCLowSide() re-inits ADC1 and starts a DMA on it, which is
  // the same ADC that analogRead(A_VBUS) uses. Leaving it in made VBUS read
  // 4.72 V on a 22 V rail - a broken instrument, not a broken supply.

  // No motor.init() / initFOC() on purpose - alignment is one of the things
  // under suspicion, so nothing here depends on it having succeeded.
  driver.disable();

  Serial.println(F("\n=== B-G431B-ESC1 open-loop calibration diagnostic ==="));
  Serial.print(F("declared voltage_power_supply: "));
  Serial.println(driver.voltage_power_supply, 2);
  Serial.print(F("measured VBUS: "));
  Serial.print(ReadBusVoltage(), 2);
  Serial.println(F(" V  <-- compare against the bench supply display"));
  Serial.print(F("encoder ppr/cpr: "));
  Serial.print(ENCODER_PPR); Serial.print('/'); Serial.println(encoder.cpr);
  PrintHelp();
}

void loop() {
  if (!Serial.available()) return;
  char C = Serial.read();
  switch (C) {
    case 'S': case 's': RunSweep(); break;
    case 'N': case 'n': RunNoiseTest(); break;
    case 'E': case 'e': RunEncoderMonitor(); break;
    case 'G': case 'g': RunGpioMonitor(); break;
    case 'D': case 'd': RunDriveTest(); break;
    case 'U': case 'u': EnablePullups(); break;
    case 'B': case 'b':
      Serial.print(F("VBUS: ")); Serial.print(ReadBusVoltage(), 2);
      Serial.print(F(" V  (raw adc ")); Serial.print(analogRead(A_VBUS));
      Serial.println(F(")"));
      break;
    case 'V': case 'v':
      SweepVoltage = Serial.parseFloat();
      Serial.print(F("sweep_voltage = ")); Serial.println(SweepVoltage, 2);
      break;
    case 'H': case 'h': RunSpeedIntegrity(); break;
    case 'C': case 'c': RunCalibrateAndTest(); break;
    case 'O': case 'o': RunOffsetSweep(); break;
    case 'M': case 'm': RunPositionMap(); break;
    case 'K': case 'k': RunCompensatedLoop(nullptr); break;
    case 'F': case 'f':
      StepDelayUs = Serial.parseInt();
      Serial.print(F("step_delay_us = ")); Serial.println(StepDelayUs);
      break;
    case 'R': case 'r':
      SweepRevs = Serial.parseInt();
      Serial.print(F("sweep_revs = ")); Serial.println(SweepRevs);
      break;
    case '?': PrintHelp(); break;
    default: break;
  }
}
