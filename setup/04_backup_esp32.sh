#!/usr/bin/env bash
# 04_backup_esp32.sh — save a byte-for-byte copy of the firmware currently on the ESP32-S3.
#
# The firmware source was lost with the SD card, so the chip holds the only working copy of the
# motor / odometry / IMU / micro-ROS code. Back it up before anything reflashes the board.
# Reading is safe: esptool resets the board into its bootloader, reads, and resets it back.
#
#   Plug the ESP32 in by its native USB-C port (/dev/esp32, Espressif 303a:1001; esptool reads
#   and flashes through it), then:
#     bash 04_backup_esp32.sh              # finds the port by itself
#     bash 04_backup_esp32.sh /dev/ttyUSB1 # or name it
#
# Output: ~/recovered/esp32-<timestamp>/  full flash image + sha256 + strings (topic names, log text).
# Keep it out of git: the image can contain WiFi credentials.
set -Eeuo pipefail

log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
die()  { printf '\033[1;31m[fail] %s\033[0m\n' "$*"; exit 1; }
trap 'die "line $LINENO: \"$BASH_COMMAND\" failed."' ERR

# Ubuntu 24.04's apt esptool (4.7) ships without the ESP32-S3 stub flasher and dies with
# "stub_flasher_32s3.json not found", so prefer the pip copy in ~/.venvs/esptool.
ESPTOOL="${ESPTOOL:-}"
[[ -z "$ESPTOOL" && -x "$HOME/.venvs/esptool/bin/esptool" ]] && ESPTOOL="$HOME/.venvs/esptool/bin/esptool"
[[ -n "$ESPTOOL" ]] || die "esptool not found. Run: python3 -m venv ~/.venvs/esptool && ~/.venvs/esptool/bin/pip install esptool"

PORT="${1:-}"
if [[ -z "$PORT" ]]; then
  if [[ -e /dev/esp32 ]]; then
    PORT="$(readlink -f /dev/esp32)"
  else
    LIDAR="$(readlink -f /dev/lidar 2>/dev/null || true)"
    PORTS=()
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
      [[ -e "$dev" && "$dev" != "$LIDAR" ]] && PORTS+=("$dev")
    done
    case ${#PORTS[@]} in
      0) die "No serial port found. Plug the ESP32's native USB-C port into the Pi." ;;
      1) PORT="${PORTS[0]}" ;;
      *) die "Several serial ports (${PORTS[*]}). Unplug the lidar, or re-run naming the ESP32 port: bash $0 /dev/ttyUSB1" ;;
    esac
  fi
fi
[[ -e "$PORT" ]] || die "$PORT doesn't exist."
if fuser -s "$(readlink -f "$PORT")" 2>/dev/null; then
  die "$PORT is in use by another program (micro-ROS agent?). Stop everything first: run/stop.sh"
fi
[[ -r "$PORT" && -w "$PORT" ]] || die "No permission on $PORT. Are you in the 'dialout' group? (02_setup_pi.sh adds you; reboot after it.)"

OUT="$HOME/recovered/esp32-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
chmod 700 "$HOME/recovered" "$OUT"

log "Chip + flash info from $PORT (nothing is written)"
if ! "$ESPTOOL" --chip esp32s3 -p "$PORT" flash_id 2>&1 | tee "$OUT/flash_id.txt"; then
  die "esptool couldn't talk to the chip. Hold BOOT, tap RESET, release BOOT, then re-run. Wrong port? Try the other one."
fi
SIZE_MB="$(grep -oE 'Detected flash size: [0-9]+MB' "$OUT/flash_id.txt" | grep -oE '[0-9]+' || true)"
[[ -n "$SIZE_MB" ]] || die "Couldn't read the flash size (see $OUT/flash_id.txt)."
BYTES=$((SIZE_MB * 1024 * 1024))

log "Reading all ${SIZE_MB} MB (~1–3 min)"
IMG="$OUT/esp32s3_full_flash_${SIZE_MB}MB.bin"
if ! "$ESPTOOL" --chip esp32s3 -p "$PORT" -b 460800 read_flash 0 "$BYTES" "$IMG"; then
  info "retrying at 115200 baud"
  "$ESPTOOL" --chip esp32s3 -p "$PORT" -b 115200 read_flash 0 "$BYTES" "$IMG"
fi
[[ "$(stat -c %s "$IMG")" -eq "$BYTES" ]] || die "Image is the wrong size — read was incomplete."
( cd "$OUT" && sha256sum "$(basename "$IMG")" > "$(basename "$IMG").sha256" )

# Printable strings: ROS topic/node names, log messages and config text help rewrite the source.
strings -n 6 "$IMG" > "$OUT/strings.txt"
grep -aiE 'cmd_vel|odom|imu|scan|bumper|watchdog|micro|rcl|node|topic|wheel|invert|campus|esp32|agent|/[a-z_]+' \
  "$OUT/strings.txt" | sort -u > "$OUT/strings_interesting.txt" || true

log "Saved to $OUT"
ls -la "$OUT" | sed 's/^/    /'
cat <<EOF

To restore this exact firmware later:
  $ESPTOOL --chip esp32s3 -p $PORT write_flash 0 "$IMG"
EOF
