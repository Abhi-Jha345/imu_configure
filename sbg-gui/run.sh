#!/usr/bin/env bash
# Build the C bridge if needed, then launch the PySide6 dashboard.
# Usage: ./run.sh [SERIAL_DEVICE] [BAUDRATE]
set -e
cd "$(dirname "$0")"

PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-921600}"

if [ ! -x bridge/build/sbg-bridge ]; then
    echo "Building sbg-bridge…"
    cmake -S bridge -B bridge/build -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build bridge/build >/dev/null
fi

exec python3 dashboard.py "$PORT" "$BAUD"
