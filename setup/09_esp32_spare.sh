#!/usr/bin/env bash
# Written 2026-10-10: make room for a second (spare) ESP32-S3 on the Pi without it ever taking
# /dev/esp32 from the robot's own board.
#   sudo setup/09_esp32_spare.sh [ROBOT_SERIAL]   (default A4:CB:8F:CA:3A:BC, the original board)
#   after a board swap: sudo setup/09_esp32_spare.sh 94:A9:90:CE:20:AC   (the board now on the robot)
# The old rules gave /dev/esp32 to ANY Espressif native-USB device (303a) and to any CH34x UART
# (1a86): with two boards plugged in, the link went to whichever enumerated last, and the next
# navigation start would have talked to a board with no motors.
#   /dev/esp32        only the robot's board, by its USB serial (its MAC: A4:CB:8F:CA:3A:BC)
#   /dev/esp32_spare  any other ESP32-S3 on native USB
#   /dev/esp32_uart   any board's UART USB-C port (CH34x), never /dev/esp32
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo: sudo $0"; exit 1; }
RULES=/etc/udev/rules.d/99-campus-bot.rules
ROBOT_SERIAL="${1:-A4:CB:8F:CA:3A:BC}"
[[ "$ROBOT_SERIAL" =~ ^([0-9A-F]{2}:){5}[0-9A-F]{2}$ ]] || { echo "serial must look like 94:A9:90:CE:20:AC"; exit 1; }
cp -n "$RULES" "$RULES.bak-before-spare"
sed -i '/idVendor}=="303a"/d; /idVendor}=="1a86"/d' "$RULES"
cat >> "$RULES" <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{serial}=="$ROBOT_SERIAL", SYMLINK+="esp32", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{serial}!="$ROBOT_SERIAL", SYMLINK+="esp32_spare", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="esp32_uart", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
sleep 1
echo "rules now:"; grep -v '^#' "$RULES" | grep -v '^$'
ls -la /dev/esp32* 2>/dev/null || true
echo "old rules kept at $RULES.bak-before-spare"
