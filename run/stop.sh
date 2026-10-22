#!/usr/bin/env bash
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
# Stop the robot and tear everything down: zero velocity first, then Nav2, SLAM, RViz, camera, odom TF,
# the static TFs, scan_filter.py, the micro-ROS agent and the lidar.
# The original missed scan_filter.py and 3 static TF publishers; this version also sweeps for strays
# by name, so a crashed or orphaned process can't keep publishing.
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

# Processes are matched on how their command line STARTS (the executable, or "python3 <script>.py").
# A loose `pkill -f micro_ros_agent` also kills any shell, editor or log viewer whose command line
# merely mentions the word: on 2026-10-01 that killed a running hw_check.sh and its caller.
DRIVERS='^([^ ]*/)?python3 ([^ ]*/)?(teleop_keys|drive_hold)\.py'
AGENT='^([^ ]*/)?micro_ros_agent( |$)|^([^ ]*/)?python3 ([^ ]*/)?sim_robot\.py'
STRAYS=(
  "$DRIVERS"
  "^([^ ]*/)?python3 ([^ ]*/)?(scan_monitor|odom_to_tf|scan_filter|sim_robot|camera_node|tag_map|auto_clear|pose_keeper|unstick$( (( KEEPDASH )) || echo '|dashboard' ))\.py"
  '^([^ ]*/)?(rviz2|micro_ros_agent|sllidar_node|async_slam_toolbox_node|static_transform_publisher|v4l2_camera_node|map_saver_cli|component_container[^ ]*|lifecycle_manager|foxglove_bridge)( |$)'
  '^/opt/ros/jazzy/lib/(nav2_|opennav_)[^ ]*'
  '^([^ ]*/)?python3 [^ ]*/ros2 (launch [^ ]*campus_bot_fw|launch nav2_bringup|run (micro_ros_agent|sllidar_ros2|v4l2_camera|nav2_map_server|slam_toolbox))'
)

KEEPDASH=0
[[ "${1:-}" == --keep-dashboard ]] && KEEPDASH=1   # the web panel stays up (it drives these restarts)

say "Stopping Campus Bot"
# 1. Anything that commands the motors goes first, so nothing overrides the zero command.
pkill -INT -f "$DRIVERS" 2>/dev/null || true
stop_bg nav; stop_bg loc
# 2. Zero velocity while the agent is still up to carry it to the ESP32.
if is_running bringup || is_running agent || pgrep -f "$AGENT" >/dev/null; then
  timeout 6 ros2 topic pub -t 5 -r 10 "$CMD_VEL_TOPIC" geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 \
    && ok "zero velocity sent on $CMD_VEL_TOPIC" || warn "couldn't send zero velocity (the 300 ms watchdog stops the motors anyway)"
fi
# 3. Everything we started.
NAMES=(slam rviz rviz_lidar tags camera foxglove auto pose_keeper unstick odom_tf lidar agent hw_lidar hw_agent bringup)
(( KEEPDASH )) || NAMES+=(dashboard buzzer face)   # face service: like the dashboard, outlives nav restarts
for name in "${NAMES[@]}"; do stop_bg "$name"; done
# 4. Strays (crashed or orphaned runs), matched precisely (see STRAYS above).
found=0
for p in "${STRAYS[@]}"; do
  if pgrep -f "$p" >/dev/null; then
    pgrep -af "$p" | cut -c1-110 | sed 's/^/      stray: /'
    pkill -INT -f "$p" 2>/dev/null || true
    found=1
  fi
done
if (( found )); then
  sleep 2
  for p in "${STRAYS[@]}"; do pkill -TERM -f "$p" 2>/dev/null || true; done
  sleep 1
  for p in "${STRAYS[@]}"; do pkill -KILL -f "$p" 2>/dev/null || true; done
fi
left="$(for p in "${STRAYS[@]}"; do pgrep -af "$p"; done | sort -u || true)"
if [[ -n "$left" ]]; then warn "still running:"; echo "$left" | sed 's/^/      /'; else ok "all stopped"; fi
