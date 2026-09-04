# UoFT RoboCup SSL

This is the repository for the UoFT RoboCup SSL team. More documentation and folders incoming!

## Contents

| folder | what |
|--------|------|
| [`DriveBaseTestCode/`](DriveBaseTestCode) | Arduino-IDE bring-up sketches, numbered 01–10 (blink → open loop → encoder → PID → CAN → Bluetooth) |
| [`FOCVelocityControl/`](FOCVelocityControl) | **Working closed-loop FOC velocity control over CAN.** PlatformIO. ESP32 commands a velocity over CAN; a B-G431B-ESC1 runs FOC to hold it. |

`FOCVelocityControl/` supersedes sketches 06–08. Its
[README](FOCVelocityControl/README.md) documents why stock SimpleFOC closed-loop
control does not work on this hardware and what was done instead — worth reading
before touching the commutation code.

