---
name: wait-for-hardware-confirmation
description: Wait for the user to confirm a physical change before running the test, never use a timed countdown
metadata:
  type: feedback
---

When a diagnostic needs the user to physically change the rig (fit a jumper,
press BOOT/EN, disconnect wires, turn the shaft), ASK and then WAIT for their
reply. Do not run a test on a timed window like "you have 25 s, go now".

**Why:** stated directly on 2026-09-03 - "please wait in the future for me to do
it". The user does not reliably see tool output as it streams, so a countdown
starts before they have read it. A loopback test ran and reported a false
negative purely because the jumper was not fitted yet, which nearly led to
blaming the USB bridge.

**How to apply:** end the turn with the instruction and let them reply. The one
exception is a test they have already confirmed is set up, or one where they
explicitly asked for a window. Same applies to hand-turn/encoder tests.
Related: [[g431-closed-loop-position-map]]
