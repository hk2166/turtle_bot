#!/usr/bin/env bash
# Written 2026-10-01: one command for tools/check_motion.py (is LASER_YAW right? do lidar and odometry agree?)
#
#   run/check_motion.sh
#
# Starts the lidar if it isn't running (run/drive.sh brings up the ESP32 link), records in the
# background, and opens the drive window. Drive: ~1 m forward, ~1 m back, a slow quarter turn left, one right. Close the window
# (Esc) and the result prints. Needs ~1.5 m of clear floor around the robot.
set -Eeuo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

if pgrep -f '^([^ ]*/)?python3 ([^ ]*/)?(drive_hold|teleop_keys)\.py' >/dev/null; then
  err "A drive window or keyboard driver is already open: close it first (Esc), then re-run."; exit 1
fi

say "Lidar"
if wait_topic "$SCAN_TOPIC" 3; then ok "$SCAN_TOPIC already live"
else
  resolve_ports
  if port_busy "$LIDAR_DEV"; then exit 1; fi
  start_bg lidar ros2 run sllidar_ros2 sllidar_node --ros-args -p channel_type:=serial \
    -p serial_port:="$LIDAR_DEV" -p serial_baudrate:="$LIDAR_BAUD" -p frame_id:=laser \
    -p angle_compensate:=true -p scan_mode:="$LIDAR_SCAN_MODE"
  STARTED_LIDAR=1
  wait_topic "$SCAN_TOPIC" 20 || { err "no $SCAN_TOPIC: see $LOG_DIR/lidar.log"; stop_bg lidar; exit 1; }
  ok "lidar started"
fi

OUT="$LOG_DIR/check_motion.txt"
python3 tools/check_motion.py --seconds 900 >"$OUT" 2>&1 &
REC=$!
cleanup() {
  kill -TERM "$REC" 2>/dev/null || true
  wait "$REC" 2>/dev/null || true
  [[ "${STARTED_LIDAR:-0}" == 1 ]] && stop_bg lidar >/dev/null
  true
}
trap cleanup EXIT

say "Recording. In the drive window: ↑ ~4 s, ↓ ~4 s, ← a quarter turn, → a quarter turn. Then Esc."
run/drive.sh || true

say "Result"
kill -TERM "$REC" 2>/dev/null || true
wait "$REC" 2>/dev/null || true
sed -n '/^odometry:/,$p' "$OUT" | sed 's/^/    /'
grep -q '^odometry:' "$OUT" || { err "no result: see $OUT"; tail -5 "$OUT" | sed 's/^/      /'; }
info "(saved in $OUT)"
