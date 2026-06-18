# SBG Ellipse-D Tools

Connect to, configure, and live-debug an **SBG Systems Ellipse-D** (dual-antenna
GNSS/INS) over a serial connection. Built on the official
[sbgECom](https://github.com/SBG-Systems/sbgECom) C library.

The Ellipse-D is part of the **ELLIPSE series**, so it is configured through the
binary `sbgEComCmd*` API — **no extra libraries beyond sbgECom are required**.
(The REST-API path is only for the High-Performance INS line: Ekinox/Apogee/Quanta.)

## What's included

| Tool | Language | Purpose |
|------|----------|---------|
| [`imu-serial-reader`](imu-serial-reader/) | C | Minimal example: open serial, print IMU + Euler data |
| [`ellipse-d-config`](ellipse-d-config/)   | C | Interactive text-menu full configuration |
| [`ellipse-d-dashboard`](ellipse-d-dashboard/) | C | Full-screen terminal (TUI) live dashboard + config |
| [`sbg-gui`](sbg-gui/) | C + Python (PySide6) | Graphical windowed dashboard + config |

The C bridge in `sbg-gui` streams decoded telemetry as JSON to the PySide6 GUI,
which keeps all binary-protocol parsing inside the proven C library.

---

## 1. Prerequisites

### Common
- An SBG **Ellipse-D** connected via its USB-serial cable (FTDI FT232 chip).
- The [sbgECom](https://github.com/SBG-Systems/sbgECom) library installed
  (instructions below).
- CMake ≥ 3.16 and a C/C++ compiler.

### Ubuntu / Linux
```bash
sudo apt update
sudo apt install -y git cmake build-essential
# For the graphical GUI (sbg-gui):
sudo apt install -y python3 python3-pip
pip3 install --user PySide6        # or: sudo apt install python3-pyside6 (if packaged)
```

### Windows
- [Git for Windows](https://git-scm.com/download/win)
- [CMake](https://cmake.org/download/) (add to PATH)
- A C/C++ toolchain — either **Visual Studio** (Desktop development with C++) or
  **MSYS2/MinGW-w64**.
- [FTDI VCP driver](https://ftdichip.com/drivers/vcp-drivers/) so the device
  appears as a `COMx` port.
- For the GUI: [Python 3](https://www.python.org/downloads/) then `pip install PySide6`.

---

## 2. Install the sbgECom library

Both platforms build sbgECom with CMake and install it so these tools can
`find_package(sbgECom)`.

### Ubuntu / Linux
```bash
git clone https://github.com/SBG-Systems/sbgECom.git
cd sbgECom
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
sudo cmake --install build              # installs to /usr/local
```
> No sudo? Install to your home prefix instead and point CMake at it later:
> `cmake --install build --prefix ~/.local`
> then build the tools with `-DCMAKE_PREFIX_PATH=$HOME/.local`.

### Windows (PowerShell, Visual Studio toolchain)
```powershell
git clone https://github.com/SBG-Systems/sbgECom.git
cd sbgECom
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
cmake --install build --prefix "C:/sbgECom"     # choose an install prefix
```
When building the tools, pass `-DCMAKE_PREFIX_PATH=C:/sbgECom`.

---

## 3. Build these tools

### Ubuntu / Linux
```bash
git clone <YOUR-REPO-URL> sbg-ellipse-d-tools
cd sbg-ellipse-d-tools
./build.sh                      # builds all C tools into each tool's build/ dir
```
or manually per tool:
```bash
cmake -S ellipse-d-dashboard -B ellipse-d-dashboard/build -DCMAKE_BUILD_TYPE=Release
cmake --build ellipse-d-dashboard/build
```

### Windows
```powershell
cmake -S ellipse-d-dashboard -B ellipse-d-dashboard/build -DCMAKE_PREFIX_PATH=C:/sbgECom
cmake --build ellipse-d-dashboard/build --config Release
```

---

## 4. Serial port permissions  ⚠️

### Ubuntu / Linux — add yourself to the `dialout` group
By default `/dev/ttyUSB0` is owned by `root:dialout`, so a normal user gets
**"unable to open the serial port"**. Fix it **once**:
```bash
sudo usermod -aG dialout $USER
```
Then **log out and back in** (group membership only applies to new sessions),
or apply it to the current terminal without logging out:
```bash
newgrp dialout
```
Verify: `groups | grep dialout`.

> **Do NOT run the tools with `sudo`.** The GUI's PySide6 is installed for your
> user, not root, so `sudo` breaks it with `ModuleNotFoundError: No module named
> 'PySide6'`. Use the `dialout` group instead.

Find your device:
```bash
ls /dev/ttyUSB* /dev/ttyACM*
dmesg | grep -i tty            # shows which port appeared on plug-in
```

### Windows — no group needed
Windows has no `dialout` group. Permission/"access denied" almost always means:
1. **Wrong COM name** — the device is `COMx`, not `/dev/ttyUSB0`. Find it:
   ```powershell
   Get-CimInstance Win32_SerialPort | Select DeviceID, Description
   ```
2. **COM10 and higher need the `\\.\` prefix** — pass `\\.\COM12`, not `COM12`.
3. **Another program holds the port** — close SBG Qinertia, PuTTY, Tera Term,
   Arduino Serial Monitor, etc. Only one program can open a COM port at a time.
4. **No COM port at all** → install the FTDI VCP driver.

---

## 5. Run

| | Ubuntu / Linux | Windows |
|---|---|---|
| Minimal reader | `./imu-serial-reader/build/imu-serial-reader /dev/ttyUSB0 921600` | `imu-serial-reader\build\Release\imu-serial-reader.exe \\.\COM3 921600` |
| Text config    | `./ellipse-d-config/build/ellipse-d-config /dev/ttyUSB0 921600` | `...\ellipse-d-config.exe \\.\COM3 921600` |
| Terminal dashboard | `./ellipse-d-dashboard/build/ellipse-d-dashboard /dev/ttyUSB0 921600` | `...\ellipse-d-dashboard.exe \\.\COM3 921600` |
| **Graphical GUI** | `cd sbg-gui && ./run.sh /dev/ttyUSB0 921600` | `cd sbg-gui` then `python dashboard.py \\.\COM3 921600` |

The default Ellipse serial baudrate is typically **921600**. If a tool connects
but no data appears, click/enable the default outputs, or try a different baud
(e.g. `115200`).

---

## Notes
- These tools target firmware ELLIPSE v3.x+ command semantics.
- Lever-arm and alignment values must be measured for your installation —
  especially the **secondary GNSS antenna lever arm** for reliable dual-antenna
  heading.
- sbgECom is © SBG Systems, MIT-licensed. See their repository for its license.
