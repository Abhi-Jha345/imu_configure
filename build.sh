#!/usr/bin/env bash
# Build all C tools. Pass extra CMake args through, e.g.:
#   ./build.sh -DCMAKE_PREFIX_PATH=$HOME/.local
set -e
cd "$(dirname "$0")"

for tool in imu-serial-reader ellipse-d-config ellipse-d-dashboard sbg-gui/bridge; do
    echo "==> Building $tool"
    cmake -S "$tool" -B "$tool/build" -DCMAKE_BUILD_TYPE=Release "$@" >/dev/null
    cmake --build "$tool/build"
done

echo
echo "Done. Binaries are in each tool's build/ directory."
echo "Run the GUI with:  cd sbg-gui && ./run.sh /dev/ttyUSB0 921600"
