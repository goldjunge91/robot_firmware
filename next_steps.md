# Firmware Next Steps

Use this checklist to capture follow-up tasks after flashing the base micro-ROS firmware.

## Pending
- [ ] Integrate ESC and gear outputs (`ESC1_PIN`, `ESC2_PIN`, `GEAR_PIN`) with safety interlocks and ROS interfaces.
- [ ] Add parameter handling (deadzone, trims) via micro-ROS services or dynamic parameters.
- [ ] Publish diagnostic status (watchdog trips, transport health) on a dedicated topic.
- [ ] Implement current limiting or slew-rate limiting based on `SLEW_DEFAULT`.
- [ ] Write hardware tests for encoder direction verification and offset tuning (`OFFSET_*`).

## Completed
- [x] Set up micro-ROS transport (USB CDC) with watchdog-safe motor control loop.
- [x] Provide documentation for SDK setup, build, flash, and agent launch.
