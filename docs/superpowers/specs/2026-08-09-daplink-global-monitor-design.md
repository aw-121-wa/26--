# DAPLink Global Monitor Design

## Goal

Continuously observe the vehicle without halting the STM32 core while it drives.

## Architecture

The firmware emits one compact `MON` frame every 100 ms through existing USART2
at 230400 baud. The DAPLink virtual COM port exposes that stream as `COM3`.
A host-side Python program reads `COM3`, validates and decodes each frame,
prints a concise live status line, and appends complete frames to CSV.

The firmware frame contains timestamp, round and route index, previous/current/
next nodes, current-node parameters, chassis mode and stop reason, primary and
cross line-sensor state, yaw/pitch/roll, heading target, gyro/line PID output,
target and measured motor speeds, and mileage.

## Constraints

- No GDB polling or debugger breakpoints during vehicle motion.
- Reuse USART2; do not change vehicle control logic.
- Sampling period is fixed at 100 ms (10 Hz).
- Host default is `COM3` at 230400 baud; command-line options can override both.
- Host output must be usable on Windows; WSL is not required for COM access.
- The host must survive malformed/incomplete serial lines and reconnect after
  a virtual-COM disconnect.

## Verification

- Host parser tests cover valid frames, malformed frames, and node-name mapping.
- Debug firmware builds successfully.
- With DAPLink connected, the host receives `MON` frames from `COM3` while the
  vehicle runs, and a CSV file is produced without pausing the vehicle.
