---
name: simplefoc-weak-symbol-linking
description: SimpleFOC driver.init() fails silently unless lib_archive = no is set in platformio.ini
metadata:
  type: project
---

SimpleFOC ships a weak fallback `_configure6PWM()` in
`drivers/hardware_specific/generic_mcu.cpp` that unconditionally returns
`SIMPLEFOC_DRIVER_INIT_FAILED`, alongside the real strong one in
`drivers/hardware_specific/stm32/stm32_mcu.cpp`. Linked as a static archive the
linker pulls `generic_mcu.o` first, its weak symbol satisfies the reference, and
`stm32_mcu.o` is never pulled in.

**Why:** `driver.init()` then returns 0 with NO diagnostic output at all, and the
motor draws no current and never moves - indistinguishable from dead hardware.
Cost several hours on 2026-09-03; the "fixes" that appeared to work (referencing
`stm32_findBestTimerCombination` or `stm32_getNumTimersUsed` from user code) only
worked by accidentally forcing that object into the link.

**How to apply:** set `lib_archive = no` in platformio.ini. Always check
`driver.init()`'s return value and fail loudly. `arm-none-eabi-nm` on the two
.o files shows `T` (strong) vs `W` (weak) for `_Z14_configure6PWMlfiiiiii`.
Related: [[g431-closed-loop-position-map]]
