#!/usr/bin/env bash
# Written 2026-10-06: every robot control that runs as a script, in one menu.
#
#   run/bot.sh            the menu (works on the desktop, over VNC/RustDesk and over plain SSH)
#
# Each choice runs one of the existing run/*.sh or tools/*.py entries in this terminal and comes
# back to the menu when it finishes (Ctrl-C inside a tool stops that tool, not the menu).
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

status() {
  local run="" n
  for n in bringup agent slam nav loc tags camera rviz foxglove lidar odom_tf; do
    is_running "$n" && run+="$n "
  done
  echo
  printf '\033[1m── Campus Bot ──────────────────────────────────────────\033[0m\n'
  printf '  power    %s\n' "$(run/power_check.sh --brief 2>/dev/null || echo '?')"
  printf '  wifi     %s\n' "$(iw dev wlan0 link 2>/dev/null | awk '/signal/{print $2 " dBm"}' || echo '?')"
  printf '  running  %s\n' "${run:-nothing}"
  printf '  maps     %s\n' "$(ls maps/*_clean.yaml 2>/dev/null | sed 's|maps/||; s|_clean.yaml||' | tr '\n' ' ' || echo none)"
  if python3 tools/pose_tool.py get --timeout 1 2>/dev/null; then :; fi | awk '{printf "  pose     x %s  y %s  yaw %s rad\n",$1,$2,$3}'
}

pick_map() {  # sets AREA, or returns 1
  local maps=() m
  for m in maps/*_clean.yaml; do [[ -e "$m" ]] && maps+=("$(basename "${m%_clean.yaml}")"); done
  ((${#maps[@]})) || { err "no saved maps yet: map first (option 1)"; return 1; }
  if ((${#maps[@]} == 1)); then AREA="${maps[0]}"; info "map: $AREA"; return 0; fi
  select AREA in "${maps[@]}"; do [[ -n "${AREA:-}" ]] && return 0; done
  return 1
}

run_one() { echo; "$@"; echo; read -r -p "  (enter = back to the menu) " _ || true; }

while true; do
  status
  cat <<'MENU'

  MAPPING & DRIVING                    CHECKS & CALIBRATION
   1) map an area (drive + save)        7) hardware check (ports, lidar, ESP32)
   2) navigate a saved map              8) lidar live view
   3) drive with the keyboard           9) lidar vs odometry check (LASER_YAW)
                                       10) wheel calibration: straight
  WHILE NAVIGATING                     11) wheel calibration: spin
   4) find/reset the robot's pose      12) column mask calibration
   5) send a goal (forward / turn)     13) power watch (live 5 V rail)
   6) reopen RViz with the saved view
                                        0) STOP EVERYTHING          q) quit
MENU
  read -r -p "  choice: " c || exit 0
  case "$c" in
    1) read -r -p "  area name (e.g. office): " AREA
       [[ "$AREA" =~ ^[A-Za-z0-9_-]+$ ]] && run_one run/remap.sh "$AREA" --no-camera || err "bad name" ;;
    2) pick_map && run_one run/nav.sh "$AREA" --no-camera ;;
    3) run_one run/drive.sh ;;
    4) read -r -p "  rough position known? enter = where mapping started, w = search wide: " w
       [[ "$w" == w ]] && run_one run/reset_pose.sh --wide || run_one run/reset_pose.sh ;;
    5) read -r -p "  goal (f 1.0 = forward m · t 90 = turn deg · X Y = map spot): " g rest
       case "$g" in
         f) run_one python3 tools/goto.py --forward $rest ;;
         t) run_one python3 tools/goto.py --turn $rest ;;
         *) run_one python3 tools/goto.py $g $rest ;;
       esac ;;
    6) run_one run/reset_pose.sh --view ;;
    7) run_one run/hw_check.sh ;;
    8) run_one run/lidar_view.sh ;;
    9) run_one run/check_motion.sh ;;
    10) run_one python3 tools/calibrate_odom.py straight --distance 3 ;;
    11) run_one python3 tools/calibrate_odom.py spin ;;
    12) run_one python3 tools/calibrate_mask.py ;;
    13) run_one run/power_check.sh --watch ;;
    0) run_one run/stop.sh ;;
    q|Q) exit 0 ;;
    *) err "pick a number from the menu" ;;
  esac
done
