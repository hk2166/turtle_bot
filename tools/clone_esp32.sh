#!/usr/bin/env bash
# Written 2026-10-10: copy the robot ESP32's original firmware onto a SPARE ESP32-S3.
#   bash tools/clone_esp32.sh [IMAGE]    default IMAGE: the newest ~/recovered/esp32-*/esp32s3_full_flash_16MB.bin
# Target: /dev/esp32_spare only (setup/09_esp32_spare.sh). Refuses the robot's own board by link AND by
# USB serial. Steps: check chip + 16 MB flash, back up the spare's own flash to ~/recovered/
# (never commit: a flash dump can hold WiFi credentials), write the image, verify it, reset.
set -euo pipefail
ROBOT_SERIAL='A4:CB:8F:CA:3A:BC'
ESPTOOL="$HOME/.venvs/esptool/bin/esptool"
die() { echo "✗ $*" >&2; exit 1; }
[[ -x "$ESPTOOL" ]] || die "esptool missing: python3 -m venv ~/.venvs/esptool && ~/.venvs/esptool/bin/pip install esptool"

IMAGE="${1:-$(ls -t "$HOME"/recovered/esp32-*/esp32s3_full_flash_16MB.bin 2>/dev/null | head -1)}"
[[ -f "$IMAGE" ]] || die "no image (expected ~/recovered/esp32-*/esp32s3_full_flash_16MB.bin)"
( cd "$(dirname "$IMAGE")" && sha256sum -c --quiet "$(basename "$IMAGE").sha256" ) || die "image checksum FAILED: $IMAGE"
echo "✓ image $IMAGE (checksum ok)"

[[ -e /dev/esp32_spare ]] || die "no /dev/esp32_spare: plug the spare's NATIVE USB-C port in (run sudo setup/09_esp32_spare.sh once first)"
PORT="$(readlink -f /dev/esp32_spare)"
[[ -e /dev/esp32 && "$PORT" == "$(readlink -f /dev/esp32)" ]] && die "/dev/esp32_spare IS the robot's board: refusing"
SERIAL="$(udevadm info -q property -n "$PORT" | sed -n 's/^ID_SERIAL_SHORT=//p')"
[[ "$SERIAL" == "$ROBOT_SERIAL" ]] && die "$PORT has the robot board's serial $SERIAL: refusing"
fuser -s "$PORT" 2>/dev/null && die "$PORT is in use by another program"
echo "✓ spare board on $PORT (serial $SERIAL), robot board untouched on $(readlink -f /dev/esp32 2>/dev/null || echo '?')"

INFO="$("$ESPTOOL" --port "$PORT" flash-id 2>&1)" || { echo "$INFO" | tail -5; die "can't talk to the spare"; }
echo "$INFO" | grep -q "ESP32-S3" || die "not an ESP32-S3: $(echo "$INFO" | grep -i 'chip' | head -1)"
echo "$INFO" | grep -q "Detected flash size: 16MB" || die "flash isn't 16 MB: $(echo "$INFO" | grep -i 'flash size')"
echo "✓ ESP32-S3 with 16 MB flash"

BK="$HOME/recovered/esp32-spare-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BK" && chmod 700 "$BK"
echo "→ backing up the spare's own flash to $BK (about 3 min) ..."
"$ESPTOOL" --port "$PORT" --baud 921600 read-flash 0 0x1000000 "$BK/spare_full_flash_16MB.bin" | tail -1
( cd "$BK" && sha256sum spare_full_flash_16MB.bin > spare_full_flash_16MB.bin.sha256 )

echo "→ writing the robot's firmware to the spare (about 3 min) ..."
"$ESPTOOL" --port "$PORT" --baud 921600 write-flash 0x0 "$IMAGE" | grep -E "Wrote|verified|Hash" || true
echo "→ verifying ..."
"$ESPTOOL" --port "$PORT" --baud 921600 verify-flash 0x0 "$IMAGE" | tail -2
"$ESPTOOL" --port "$PORT" --after hard-reset chip-id >/dev/null 2>&1 || true
echo "✓ done: the spare now runs the robot's original firmware. Its old contents: $BK"
