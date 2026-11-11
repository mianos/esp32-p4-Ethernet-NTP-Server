#!/usr/bin/env bash
# Build wrapper for the ESP32-P4 Ethernet demo.
#   ./build.sh          build only
#   ./build.sh flash    build + flash over the USB-UART (CH343) port
#   ./build.sh monitor  build + flash + serial monitor (Ctrl-] to exit)
#   ./build.sh erase    erase the whole 32 MB flash
#
# Override the ESP-IDF install with IDF_PATH=... and the serial node with
# PORT=... ; both are guessed if unset.
set -e
set -o pipefail
export IDF_PATH=${IDF_PATH:-$HOME/.espressif/v6.0.1/esp-idf}
. "$IDF_PATH/export.sh" >/dev/null 2>&1
cd "$(dirname "$0")"
# On macOS use the /dev/cu.* node: opening /dev/tty.* can report EBUSY.
PORT=${PORT:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true)}

if [ ! -f sdkconfig ]; then
    idf.py set-target esp32p4
fi

idf.py build 2>&1 | tee /tmp/p4_ntp_build.log | tail -n 30

if [ -n "$1" ] && [ -z "$PORT" ]; then
    echo "build.sh: no /dev/cu.usbmodem* node found; set PORT=/dev/cu.usbmodemXXXX" >&2
    exit 1
fi

case "$1" in
    flash)   idf.py -p "$PORT" flash ;;
    monitor) idf.py -p "$PORT" flash monitor ;;
    erase)   idf.py -p "$PORT" erase-flash ;;
esac
