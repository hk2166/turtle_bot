#!/usr/bin/env bash
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
# The mast camera (Logitech Brio 100). It's kept OFF while mapping (frees CPU and WiFi); remap.sh turns
# it off, and this turns it back on afterwards.
#   run/camera.sh [on|off|status]   (default: on)     Publishes /image_raw, frame `camera`.
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
DEV="${CAMERA_DEV:-/dev/video0}"

case "${1:-on}" in
  on)
    if is_running camera; then ok "camera already on"; exit 0; fi
    [[ -e "$DEV" ]] || { err "$DEV not found. Is the Brio plugged in? (v4l2-ctl --list-devices)"; exit 1; }
    start_bg camera ros2 run v4l2_camera v4l2_camera_node --ros-args \
      -p video_device:="$DEV" -p image_size:="[640,480]" -p camera_frame_id:=camera
    sleep 3
    if is_running camera; then ok "camera on ($DEV → /image_raw)"; else err "camera failed: see $LOG_DIR/camera.log"; exit 1; fi ;;
  off)
    stop_bg camera
    pkill -INT -f v4l2_camera_node 2>/dev/null || true
    ok "camera off" ;;
  status)
    if is_running camera || pgrep -f v4l2_camera_node >/dev/null; then echo "camera: on"; else echo "camera: off"; fi ;;
  *) echo "usage: $0 [on|off|status]"; exit 2 ;;
esac
