#!/usr/bin/env bash
# Written 2026-10-01: build (or flash) the ESP32 firmware in firmware/campus_bot_base.
#
#   firmware/build.sh           # build only (safe; the first build takes ~15–25 min)
#   firmware/build.sh upload    # flash it (asks for confirmation; back the chip up first)
#
# micro-ROS must be built WITHOUT the ROS 2 environment. CMake turns every .../bin on PATH into a
# search prefix, finds /opt/ros/jazzy, and mixes the Pi's ROS packages into the ESP32 build
# ("No 'rosidl_typesupport_cpp' found"). ~/.bashrc sources ROS in every terminal, so this runs
# PlatformIO in a clean environment instead.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/campus_bot_base"

args=(run)
if [[ "${1:-}" == upload ]]; then
  cat <<'EOF'
This REPLACES the firmware on the robot's ESP32. The source here is a reconstruction: pins and
constants marked VERIFY in include/config.h are guesses (see firmware/README.md).
Before flashing: back up the chip (bash setup/04_backup_esp32.sh), stop everything (run/stop.sh),
and lift the wheels off the ground.
EOF
  read -r -p "Type 'flash' to continue: " answer
  [[ "$answer" == flash ]] || { echo "Not flashed."; exit 1; }
  args=(run -t upload --upload-port /dev/esp32)
fi

# micro_ros_platformio sources PROJECT_CORE_DIR/penv/bin/activate; a pipx install doesn't have it.
if [[ ! -e "$HOME/.platformio/penv" && -d "$HOME/.local/share/pipx/venvs/platformio" ]]; then
  ln -s "$HOME/.local/share/pipx/venvs/platformio" "$HOME/.platformio/penv"
fi

exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LANG=C.UTF-8 TERM="${TERM:-xterm}" \
  PATH="$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  pio "${args[@]}"
