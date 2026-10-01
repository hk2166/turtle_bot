#!/usr/bin/env bash
# Written 2026-10-01: see what the lidar is sending, live, in RViz and as numbers in this terminal.
#
#   run/lidar_view.sh            # driver + RViz + live numbers here
#   run/lidar_view.sh --no-rviz  # numbers only (works over SSH)
#
# RViz uses config/lidar_view.rviz: fixed frame = laser, so no TF or odometry is needed.
# Ctrl-C closes everything this script started. If run/remap.sh's bring-up is already running, it
# reuses that lidar driver instead of opening the port a second time.
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

RVIZ=1
[[ "${1:-}" == --no-rviz ]] && RVIZ=0

STARTED=()
cleanup() { for n in "${STARTED[@]}"; do stop_bg "$n"; done; }
trap cleanup EXIT

say "Lidar driver"
if is_running bringup; then
  ok "using the lidar from run/remap.sh's bring-up"
elif is_running lidar; then
  ok "lidar driver already running"
else
  resolve_ports
  [[ -e "$LIDAR_DEV" ]] || { err "lidar port $LIDAR_DEV not found. Plug in the CP2102 stick."; exit 1; }
  start_bg lidar ros2 run sllidar_ros2 sllidar_node --ros-args \
    -p channel_type:=serial -p serial_port:="$LIDAR_DEV" -p serial_baudrate:="$LIDAR_BAUD" \
    -p frame_id:=laser -p angle_compensate:=true -p scan_mode:="$LIDAR_SCAN_MODE"
  STARTED+=(lidar)
  info "sllidar_node on $LIDAR_DEV @ $LIDAR_BAUD (log: $LOG_DIR/lidar.log)"
fi
wait_topic "$SCAN_TOPIC" 15 || { err "no $SCAN_TOPIC. Last lines of $LOG_DIR/lidar.log:"; tail -5 "$LOG_DIR/lidar.log" >&2; exit 1; }
ok "$SCAN_TOPIC live"

if (( RVIZ )); then
  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    warn "no display (SSH?): skipping RViz"
  else
    start_bg rviz_lidar rviz2 -d "$PROJECT_DIR/config/lidar_view.rviz"
    STARTED+=(rviz_lidar)
    ok "RViz opened (grid = 1 m; the red axis is the lidar's front)"
  fi
fi

say "Live data (Ctrl-C to quit)"
sleep 1
python3 tools/scan_monitor.py --topic "$SCAN_TOPIC"
