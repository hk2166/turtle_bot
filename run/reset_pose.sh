#!/usr/bin/env bash
# Written 2026-10-01: put the robot's pose (and the RViz view) right again while navigating.
#
#   run/reset_pose.sh                 robot is near its mapping START spot (within ~1 m, ±30°): the
#                                     lidar scan is fitted to the map there and AMCL is set to the fit
#   run/reset_pose.sh X Y YAW_DEG     same, around a rough guess: map position X, Y (m), facing YAW_DEG
#                                     (0 = the map's x axis, the red TF arrow; 90 = its y axis, green)
#   run/reset_pose.sh ... --exact     set exactly that pose, no fitting
#   run/reset_pose.sh --wide ...      search ±2.5 m and ±180° instead (slower, ~10 s)
#   run/reset_pose.sh --view          RViz only: reopen it with the saved top-down view. Dragging in
#                                     RViz moves only the VIEW (it starts in "Move Camera" mode), not
#                                     the robot; select "2D Pose Estimate" first to set a pose.
#
# AMCL keeps refining the pose from the lidar as the robot moves.
set -Eeuo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

EXACT=0 WIDE=0 ARGS=()
for a in "$@"; do
  case "$a" in
    --view)
      stop_bg rviz >/dev/null
      use_robot_screen || { err "no desktop to show RViz on (log in on the monitor, or use VNC)"; exit 1; }
      start_bg rviz rviz2 -d "$PROJECT_DIR/config/campus_bot.rviz"
      ok "RViz reopened with the saved top-down view"; exit 0 ;;
    --exact) EXACT=1 ;;
    --wide) WIDE=1 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) ARGS+=("$a") ;;
  esac
done
case ${#ARGS[@]} in
  0) X=0 Y=0 YAW_DEG=0 ;;
  3) X="${ARGS[0]}" Y="${ARGS[1]}" YAW_DEG="${ARGS[2]}" ;;
  *) sed -n '2,18p' "$0"; exit 2 ;;
esac

if ! ros2 lifecycle get /amcl 2>/dev/null | grep -q '^active'; then
  err "AMCL isn't running: start navigation first (run/nav.sh <area>)"; exit 1
fi
before="$(python3 tools/pose_tool.py get --timeout 2 2>/dev/null || echo 'none')"
YAW="$(python3 -c "import math,sys; print(math.radians(float(sys.argv[1])))" "$YAW_DEG")"
if (( ! EXACT )); then
  RANGE=(--radius 1.0 --angle 30); (( WIDE )) && RANGE=(--radius 2.5 --angle 180)
  info "fitting the scan to the map around x $X, y $Y, ${YAW_DEG}° …"
  if fit="$(python3 tools/fit_pose.py "$X" "$Y" "$YAW_DEG" "${RANGE[@]}")"; then
    read -r FX FY FYAW SCORE <<<"$fit"
    pct="$(python3 -c "print(round(100*float('$SCORE')))")"
    if python3 -c "import sys; sys.exit(0 if float('$SCORE') >= 0.6 else 1)"; then
      ok "best fit: x $FX  y $FY  $(python3 -c "import math; print(round(math.degrees($FYAW)))")°  ($pct% of the laser points on map walls)"
      X="$FX" Y="$FY" YAW="$FYAW"
    else
      warn "no good fit nearby (best $pct% of points on walls): setting the pose as given."
      warn "Is the robot really there? Try --wide, or give a closer X Y YAW_DEG."
    fi
  else
    warn "fitting failed: setting the pose as given"
  fi
fi
python3 tools/pose_tool.py set "$X" "$Y" "$YAW"
sleep 2
after="$(python3 tools/pose_tool.py get --timeout 3 2>/dev/null || echo 'none')"
info "pose before: $before"
if [[ "$after" == none ]]; then err "AMCL didn't take the pose (see $LOG_DIR/loc.log)"; exit 1; fi
ok "pose now:    $after   (x y yaw in radians)"
info "In RViz the red laser points should sit on the black walls."
