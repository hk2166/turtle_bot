#!/usr/bin/env bash
# Rewritten 2026-10-01: the camera is now nodes/camera_node.py (AprilTag detection + light preview).
# The mast camera (Logitech Brio 100). run/remap.sh starts it for mapping (records the tags) and for
# --nav (localizes with them); this script runs it by hand.
#   run/camera.sh [on|off|status]   (default: on)
#   on: /tags (tag poses, frame camera_optical), /camera/image/compressed (2 Hz preview), /camera/camera_info
# Only one program can use the camera: face/verify.py, tools/calibrate_camera.py and
# tools/camera_pose.py need it off.
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
DEV="${CAMERA_DEV:-/dev/video0}"
NODE='^([^ ]*/)?(python3 ([^ ]*/)?camera_node\.py|v4l2_camera_node)( |$)'

case "${1:-on}" in
  on)
    if is_running camera; then ok "camera already on"; exit 0; fi
    [[ -e "$DEV" ]] || { err "$DEV not found. Is the Brio plugged in? (v4l2-ctl --list-devices)"; exit 1; }
    # reserve it: the dashboard (EYES feed) lets go within a second and then shows our preview instead
    mkdir -p "$RUN_DIR" && touch "$RUN_DIR/camera_reserved"
    for _ in $(seq 20); do
      holders="$(fuser "$DEV" 2>/dev/null | tr -s ' ' ',' | sed 's/^,//; s/,$//')"
      [[ -z "$holders" ]] && break
      sleep 0.25
    done
    if [[ -n "$holders" ]]; then
      err "$DEV is in use by:"; ps -o pid=,args= -p "$holders" | cut -c1-100 | sed 's/^/        /' >&2; exit 1
    fi
    start_bg camera python3 "$PROJECT_DIR/nodes/camera_node.py" --ros-args \
      -p device:="$DEV" -p width:="$CAMERA_WIDTH" -p height:="$CAMERA_HEIGHT" -p tag_size:="$TAG_SIZE"
    if wait_topic /tags 30; then ok "camera on ($DEV → /tags, preview /camera/image/compressed)"
    else err "camera started but no /tags: see $LOG_DIR/camera.log"; stop_bg camera; exit 1; fi
    if [[ ! -f "$PROJECT_DIR/config/camera_intrinsics.yaml" ]]; then
      warn "camera not calibrated yet: tools/calibrate_camera.py (tag distances ~5% off until then)"
    fi ;;
  off)
    stop_bg camera
    pkill -INT -f "$NODE" 2>/dev/null || true
    ok "camera off" ;;
  status)
    if is_running camera || pgrep -f "$NODE" >/dev/null; then echo "camera: on"; else echo "camera: off"; fi ;;
  *) echo "usage: $0 [on|off|status]"; exit 2 ;;
esac
