---
name: g431-closed-loop-position-map
description: B-G431B-ESC1 closed loop needs a measured encoder position-error map; SimpleFOC's linear electricalAngle cannot work
metadata:
  type: project
---

On the B-G431B-ESC1 + 12-pole-pair motor + 1000 PPR encoder rig, the encoder
angle does NOT map linearly onto the rotor's electrical angle. The residual
error swings ~+-4 rad ELECTRICAL over a revolution (~2 cycles per turn),
repeatable and torque-independent (identical at 1.5 V and 2.5 V drive).

**Why:** torque reverses beyond +-PI/2, so that error flips torque sign several
times per revolution. No constant offset can fix it - measured directly on
2026-09-03: best travel over 24 different `zero_electric_angle` offsets was
0.15 rev, versus 42 rev with the map applied. This is why `initFOC()` /
`alignSensor()` and every manual offset sweep failed, and why
`zero_electric_angle` came out different every boot.

**How to apply:** commutation is done manually in `src/closedloop.cpp` of
`~/Documents/PlatformIO/Projects/g431_foc` - a 256-bin map is measured at boot
(~15 s open-loop sweep both directions) and used as
`el = normalize(-pp*mech - MapLookup(mech))`. SimpleFOC's `initFOC/loopFOC/move`
are bypassed because `FOCMotor::electricalAngle()` assumes a linear map and is
not virtual. Confirmed working: velocity commands 10-80 rad/s both directions,
steady-state error <0.1 rad/s, settling 20-50 ms.

Two traps that cost time: `BLDCDriver6PWM::enable()` ends with `setPwm(0,0,0)`
so it must be called ONLY on transition, never every loop iteration; and host
serial telemetry during measurement stalls the FOC loop enough to corrupt the
numbers, so measure statistics on-device.
Related: [[simplefoc-weak-symbol-linking]]


**The map CANNOT be persisted across power cycles (learned the hard way
2026-09-04).** The encoder is incremental, so its zero is wherever the rotor sat
at power-on, and the map is indexed by `getMechanicalAngle()` counting from that
zero. A map captured in one boot is offset by an arbitrary amount in the next,
so commutation is wrong at every position - the field lands on the rotor d-axis
and you get full current with zero torque. Baking a captured map into the
firmware caused an instant stall at the supply's 3 A limit, and the "jerkier"
run reported just before it was the same misaligned map, not a CAN problem.

Measure the map at every boot. To make persistence sound, first establish an
absolute reference: the encoder's index/Z channel is on PB8 and is unused -
homing to it once at startup would pin the map to a known rotor angle. Also note
map quality varies between boots (the rotor stick-slips through the sweep); a
good one measured sd 2.22 @20 rad/s vs a typical ~4.9.


**Fault protection added 2026-09-04** (there is NO current sensing, so the bench
supply's limit was previously the only protection). Primary trip is
OVERCURRENT on an estimate `I = (|Uq| - Ke*|vel|)/R` with Ke=0.0227 (measured)
and PhaseR=0.168 (from notes, unverified). Threshold 12 A for 200 ms.
Normal running estimates 0.7-3.5 A (6.7 A worst acceleration transient); both
stall and reverse-commutation faults sit at ~17-18 A, so the ~5x separation
survives even if R is wrong, since an R error scales both together.
Backups: OVERSPEED >140 rad/s, RUNAWAY, REVERSE, and a behavioural STALL rule.
Faults latch, disable the driver, and clear on 'X' or on a commanded 0 (so the
ESP32 sending 0 re-arms without serial access).

Verified: 3/3 stall injections caught in 217-218 ms, 3/3 reverse injections in
315-705 ms, zero false positives across 10-80 rad/s and hard reversals to 80/-80.
Direction-based rules alone were unreliable (5 s once, missed once) because
broken commutation makes velocity oscillate in sign - current is the better
discriminator. 'Z'/'Z1' inject a -90/+90 deg map shift to test this on demand;
run 'A' afterwards to restore a good map.


**Velocity-loop retune 2026-09-04 - tune against RAW ANGLE, not filtered
velocity.** The first tuning pass optimised sd of the VelLPF output, i.e. the
signal the filter smooths, so it was blind to the oscillation it created and
chose P=0.05/I=5.0/Tf=0.05. Recording raw encoder angle at 1 kHz ('J' command)
exposed a ~12 Hz loop limit cycle: velocity mode measured sd 35 at 40 rad/s
while OPEN-LOOP torque mode at the same speed measured sd 9.5 - the loop was
making the motor ~4x jerkier than no loop at all.

Diagnostic that identified it: ripple frequency barely moved with shaft speed
(8.8 Hz at 10 rad/s, 12.6 Hz at 40). Commutation ripple would have tracked
electrical frequency (20 -> 77 Hz over that range), so a near-constant frequency
means loop dynamics. Worth reusing whenever "is it the plant or the controller".

New defaults P=0.08 I=0.5 Tf=0.01. Raw-angle sd at target 20 went 10.71 -> 1.98
(5.4x), at 40 went 35.38 -> 5.72 (6.2x), and settling improved to 14-16 ms with
steady-state error under 0.05 rad/s. Fault protection unaffected (stall still
trips in 218 ms) and no false trips over CAN at 10/25/60/-30.
