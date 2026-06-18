# SBG Ellipse-D — GUI Dashboard

A graphical (PySide6) dashboard for live debugging **and** configuring the SBG
Ellipse-D over a serial connection.

## Architecture
```
 ┌──────────────┐  text commands (stdin)   ┌────────────┐  serial   ┌───────────┐
 │  dashboard.py│ ───────────────────────▶ │ sbg-bridge │ ───────▶ │ Ellipse-D │
 │  (PySide6)   │ ◀─────────────────────── │   (C)      │ ◀─────── │   IMU/INS │
 └──────────────┘   JSON telemetry (stdout)└────────────┘           └───────────┘
```
- **sbg-bridge** (C, links the system-wide `sbgECom` library) does all serial
  I/O and binary-protocol decoding, emitting one JSON object per log on stdout
  and accepting text commands on stdin.
- **dashboard.py** runs the bridge as a subprocess and renders the GUI — no
  Python serial/protocol code, so nothing to keep in sync with the device firmware.

## Requirements
- PySide6 (already installed: `python3 -c "import PySide6"`)
- The bridge depends on the system-wide sbgECom install (`/usr/local`).

## Run
```bash
./run.sh                       # defaults to /dev/ttyUSB0 @ 921600
./run.sh /dev/ttyUSB0 921600   # explicit
```
`run.sh` builds the bridge on first run. Port/baud can also be changed in the
toolbar before pressing **Connect**.

## Features
- Artificial-horizon attitude indicator + heading compass
- Value cards: roll/pitch/yaw, lat/lon/alt, velocity, GNSS fix, dual-antenna heading
- Health LEDs (power/IMU/GPS/settings/temperature) and EKF validity (att/hdg/pos)
- Color-coded EKF solution mode and RTK fix quality
- Rolling accelerometer & gyroscope plots
- Per-log data-rate readout
- Configuration: motion profile, dual-antenna lever arms, enable default outputs,
  save-to-flash + reboot, restore factory defaults

## Note on serial permissions
Your user must be in the `dialout` group to open `/dev/ttyUSB0`:
```bash
sudo usermod -aG dialout $USER   # then log out/in (or: newgrp dialout)
```
