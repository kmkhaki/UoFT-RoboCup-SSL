# Closed-Loop FOC Velocity Control over CAN

ESP32 sends a velocity command over CAN; a B-G431B-ESC1 runs closed-loop
field-oriented control of a BLDC motor to hold that velocity.

This replaces the earlier Arduino-IDE sketches in `DriveBaseTestCode/` with two
PlatformIO projects. It exists because closed-loop control on this hardware did
not work with stock SimpleFOC, for a reason that took a long time to find and is
documented below — read **"Why this doesn't use SimpleFOC's motor layer"** before
changing the commutation code.

---

## Architecture

```
   ESP32 devkit                SN65HVD230              B-G431B-ESC1
  ┌──────────────┐            ┌──────────┐            ┌──────────────┐
  │ TWAI driver  │ GPIO5  →D  │          │ CANH ────→ │ FDCAN1       │
  │ int8 velocity│ GPIO35 ←R  │  3.3 V   │ CANL ────→ │ PA11/PB9     │
  │ 150 ms repeat│            │          │  GND ────→ │ onboard xcvr │
  └──────────────┘            └──────────┘            └──────────────┘
                                                       position map →
                                                       FOC commutation →
                                                       velocity PID
```

| | |
|---|---|
| Motor | BLDC, **12 pole pairs** (measured, see below) |
| Encoder | incremental quadrature, 1000 PPR / **4000 CPR**, A=PB6 B=PB7 |
| Driver | `BLDCDriver6PWM` on TIM1 — PA8/PC13, PA9/PA12, PA10/PB15 |
| Bus rail | ~24.5 V bench supply, current-limited |
| CAN | 250 kbit/s, ID `0x120`, one signed byte |

---

## Quick start

```bash
# ESC1 (motor controller)
cd g431_foc
pio run -e closedloop -t upload     # the real firmware
pio run -e diagnostic -t upload     # open-loop measurement tools
pio run -e mintest    -t upload     # minimal driver bring-up check

# ESP32 (command source)
cd esp32_can_test
pio run -t upload
```

**Power the motor supply before resetting the ESC1.** It calibrates at every
boot and needs the shaft to actually turn (see Startup below).

---

## Why this doesn't use SimpleFOC's motor layer

SimpleFOC's `initFOC()` / `loopFOC()` / `move()` are **not used**. Only its
`Encoder`, `BLDCDriver6PWM`, `setPhaseVoltage()`, `PIDController` and
`LowPassFilter` are.

`FOCMotor::electricalAngle()` assumes the encoder maps *linearly* onto rotor
electrical angle:

```
el = sensor_direction * pole_pairs * mechanical_angle - zero_electric_angle
```

On this motor that is false. The residual error swings about **±4 rad
electrical** over one revolution (~2 cycles per turn), and it is repeatable and
torque-independent — identical measured at 1.5 V and 2.5 V drive. Torque
reverses sign beyond ±π/2, so that error flips the torque direction several
times per revolution.

**No constant offset can correct a position-dependent error.** Measured directly:

| | shaft travel in the same window |
|---|---|
| best of 24 different `zero_electric_angle` offsets | 0.15 rev |
| with the measured position map applied | **42.4 rev** (repeatable to 0.3%) |

That is why `alignSensor()`, the PP check, and every manual offset sweep failed,
and why `zero_electric_angle` came out different on every boot.

`electricalAngle()` is not `virtual`, and correcting the *sensor* angle instead
risks a non-monotonic angle (the correction's slope can exceed the raw angle's),
so commutation is done explicitly in `closedloop.cpp`.

---

## The position map

At boot, an open-loop sweep drives 24 electrical revolutions in each direction.
During a slow sweep the rotor's d-axis sits where the stator vector puts it —
`setPhaseVoltage(Uq, 0, ElCmd)` places that vector at `ElCmd + π/2` — so the
true electrical angle is known at every step and can be differenced against what
the encoder predicts. Errors are binned by mechanical position (256 bins) and
averaged circularly. Sweeping **both** directions cancels the load angle, which
is direction-dependent while a genuine sensor error is not.

Commutation then uses:

```c
el = normalize(-POLE_PAIRS * mech - MapLookup(mech));
```

### The map cannot be persisted across power cycles

The encoder is **incremental**: its zero is wherever the rotor sat at power-on,
and the map is indexed from that zero. A map captured in one boot is offset by an
arbitrary amount in the next, making commutation wrong everywhere — the field
lands on the rotor d-axis, giving full current and zero torque. Baking a captured
map into the firmware caused an instant stall at the supply's 3 A limit.

To make persistence sound you need an absolute reference first. **The encoder's
index/Z channel is on PB8 and is currently unused** — homing to it once at
startup would pin the map to a known rotor angle. Until then, measure every boot.

Map quality varies between boots because the rotor stick-slips through the sweep.
A good map measured `sd 2.22` at 20 rad/s; a mediocre one ~4.9.

---

## Startup and the calibration check

Calibration verifies the shaft actually moved ~12.57 rad (24 electrical revs at
12 pole pairs). If it didn't, the map is marked **INVALID** and the driver
refuses to run.

This matters because the ESC1 is powered over ST-Link USB, so it boots and sweeps
happily with the **motor supply off** or the **encoder unpowered** — every bin
still gets samples, they are just all from a stationary rotor. The old code
reported `map bins filled: 256/256 OK` on a completely meaningless map, and the
motor then stalled the moment real power returned. Both failure modes were hit
during bring-up.

---

## Serial commands (ESC1, 115200)

| cmd | meaning |
|---|---|
| `T<v>` | target — Uq volts in torque mode, rad/s in velocity mode |
| `C0` / `C1` | torque / velocity mode |
| `N1` / `N0` | CAN control on / off (off = serial `T` only, for bench work) |
| `L<v>` | voltage limit |
| `P` `I` `F` | velocity PID P, I, and low-pass Tf |
| `G<x>` | commutation lead angle, elec rad per shaft rad/s |
| `A` | re-run the position map calibration |
| `D` | dump the position map |
| `S` | status — fault, Iest, CAN rx/age, mode, target, velocity, VBUS |
| `X` | clear a latched fault |
| `E` | FDCAN error/status registers |
| `U1` / `U0` | board's 120 Ω CAN terminator |
| `Y` | measure CAN frame/error rate over 4 s |
| `Q<uq>` | on-device torque-mode velocity statistics |
| `W<v>` | on-device closed velocity-loop test |
| `O` | observe velocity stats without changing anything |
| `J` | dump a 1 kHz raw angle trace |
| `Z` / `Z1` | **test only** — inject −90°/+90° map shift to prove fault trips |

ESP32 (115200): type a number `-128..127` + Enter. `T` runs the transceiver
self-test, `P` probes the R pin.

---

## CAN protocol

Unchanged from the earlier working link.

- **ID `0x120`**, 250 kbit/s, **1 data byte**, signed `int8_t`, applied 1:1 as rad/s
- ESP32 repeats the last value every **150 ms** as a heartbeat
- ESC1 **fail-safe**: no frame for **500 ms** → target forced to 0

Range is capped at ±127 rad/s by the single byte. The stated target of ±150 rad/s
needs widening to `int16`.

The ESP32 now **recovers from bus-off automatically**. Without that it latched
permanently on any transient bus fault (a loose wire is enough), returning
`ESP_ERR_INVALID_STATE` to every transmit until power-cycled.

---

## Fault protection

There is **no current sensing** — SimpleFOC 2.4.0 reports `Low-side cs not
supported!` for this board, and attempting it also hijacks ADC1 and breaks the
VBUS reading. So the bench supply's limit was the only protection.

Primary trip is **over-current on an estimate**:

```
I = (|Uq| - Ke*|vel|) / R      Ke = 0.0227 (measured), R = 0.168 Ω (unverified)
```

Threshold **12 A for 200 ms**. Normal running estimates 0.7–3.5 A (6.7 A worst
acceleration transient); stall and reverse-commutation faults both sit at
~17–18 A. The ~5× separation survives an error in `R`, since that scales normal
and fault readings together.

Backups: `OVERSPEED` (>140 rad/s), `RUNAWAY`, `REVERSE`, and a behavioural
`STALL` rule. Faults latch, disable the driver, and clear on `X` **or on a
commanded 0** — so the ESP32 sending 0 re-arms without serial access.

Verified: 3/3 stall injections caught in 217–218 ms, 3/3 reverse injections in
315–705 ms, **zero false positives** across 10–80 rad/s and hard reversals to
±80.

> Direction-based rules alone were unreliable — 5 s once, missed entirely
> another time — because broken commutation makes velocity oscillate in sign.
> Current is the better discriminator.

---

## Measured characteristics

| quantity | value | how |
|---|---|---|
| Pole pairs | **12.003** fwd / 12.008 rev | 24 elec revs = 1.999 mech revs |
| Encoder CPR | **4000** | hand turn = 4002 counts / 6.2864 rad |
| Encoder integrity | 0 counts lost at 20 rad/s | out-and-back, 100% tracking |
| Ke | 0.0227 V·s/rad (~421 rpm/V) | 44.1 rad/s per volt of Uq |
| Friction intercept | ~6.5 rad/s | linear fit of speed vs Uq |

Velocity loop, P=0.08 I=0.5 Tf=0.01, steady-state error ≤0.05 rad/s:

| target | raw-angle sd | settling |
|---|---|---|
| 10 | 5.48 | 35 ms |
| 20 | 1.98 | 21 ms |
| 40 | 5.72 | 28 ms |
| 60 | 13.04 | 40 ms |

---

## Gotchas that cost real time

**1. `lib_archive = no` is mandatory.** SimpleFOC ships a *weak* fallback
`_configure6PWM()` in `drivers/hardware_specific/generic_mcu.cpp` that
unconditionally returns `SIMPLEFOC_DRIVER_INIT_FAILED`, alongside the real strong
one in `stm32/stm32_mcu.cpp`. Linked as a static archive, the linker pulls
`generic_mcu.o` first, its weak symbol satisfies the reference, and
`stm32_mcu.o` is never pulled in. `driver.init()` then returns 0 **with no
diagnostic output at all** and the motor silently never moves —
indistinguishable from dead hardware. Confirm with:

```bash
arm-none-eabi-nm generic_mcu.cpp.o | grep configure6PWM   # W = weak
arm-none-eabi-nm stm32_mcu.cpp.o   | grep configure6PWM   # T = strong
```

**Always check `driver.init()`'s return value.**

**2. Tune against the raw encoder angle, not filtered velocity.** Optimising the
`sd` of `VelLPF`'s output — the signal the filter smooths — is blind to the
oscillation it creates. That produced P=0.05/I=5.0/Tf=0.05, which ran a ~12 Hz
limit cycle: velocity mode measured `sd 35` at 40 rad/s where **open-loop torque
mode measured 9.5**. The loop was making the motor ~4× jerkier than no loop.

Diagnostic that identified it: ripple frequency barely moved with shaft speed
(8.8 Hz at 10 rad/s, 12.6 Hz at 40). Commutation ripple would have tracked
electrical frequency (20 → 77 Hz). **A ripple that doesn't scale with speed is
loop dynamics, not the plant.**

**3. `BLDCDriver6PWM::enable()` ends with `setPwm(0,0,0)`.** Calling it every
loop iteration zeroes the PWM registers immediately before `setPhaseVoltage()`
rewrites them, chopping the output — 1.5 V gave 24 rad/s instead of 62, with
heavy ripple and intermittent stalling. Enable only on transition.

**4. Host serial telemetry perturbs the FOC loop.** ~60 chars every 20 ms is
~5 ms of blocking UART at 115200 and stalls commutation. Measure statistics
**on-device** (`Q`, `W`, `O`), not by streaming.

**5. `getMechanicalAngle()` can be negative.** `Encoder::update()` uses
`counter % cpr` and C++ keeps the dividend's sign, so it spans (−2π, 2π), not
[0, 2π). Anything that bins or indexes on it must fold it first.

**6. CAN `PC14` must be driven, not left floating.**

---

## Known issues / next steps

- **`R = 0.168 Ω` is unverified** — from project notes, not measured here. The
  fault threshold survives an error in it, but the displayed `Iest` amps are only
  as good as `R`. Command a stall and read the supply's actual current to back it out.
- **`Iest` is phase current, not supply current.** At ~6% duty the DC-side draw is
  much lower — `Iest=17.9 A` at stall and a supply clamping at 3 A are consistent.
- **±127 rad/s protocol cap** — widen to `int16` for the ±150 rad/s target.
- **`Vlim = 3.0 V`** caps speed at ~126 rad/s. Raising it gives more authority to
  reject ripple but raises stall current.
- **Low speed is the worst case relatively** (sd 5.48 on a mean of 10). A cogging
  feedforward table, learned like the position map, would target it.
- **Use PB8 (encoder index) to make the map persistent** and remove the 15 s
  startup sweep — important if this ever powers up under load.
- **Bring-up order matters.** Verify in this sequence, since a failure at any
  stage makes later ones meaningless: transceiver self-test (`T` on ESP32) →
  CAN quality (`Y`, three trials) → calibration shaft-moved check → motor.

---

## `claude-context/`

Working notes accumulated by Claude Code across these sessions — the same
findings above in memory-file form, including the reasoning that led to them.
