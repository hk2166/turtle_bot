#!/usr/bin/env bash
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
# Stop the robot and tear everything down: zero velocity first, then Nav2, SLAM, RViz, camera, odom TF,
# the static TFs, scan_filter.py, the micro-ROS agent and the lidar.
# The original missed scan_filter.py and 3 static TF publishers; this version also sweeps for strays
# by name, so a crashed or orphaned process can't keep publishing.
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

say "Stopping Campus Bot"
# 1. Anything that commands the motors goes first, so nothing overrides the zero command.
pkill -INT -f 'teleop_keys.py|drive_hold.py' 2>/dev/null || true
stop_bg nav
# 2. Zero velocity while the agent is still up to carry it to the ESP32.
if is_running bringup || pgrep -f micro_ros_agent >/dev/null || pgrep -f sim_robot.py >/dev/null; then
  timeout 6 ros2 topic pub -t 5 -r 10 "$CMD_VEL_TOPIC" geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 \
    && ok "zero velocity sent on $CMD_VEL_TOPIC" || warn "couldn't send zero velocity (the 300 ms watchdog stops the motors anyway)"
fi
# 3. Everything we started.
for name in slam rviz rviz_lidar camera odom_tf lidar agent bringup; do stop_bg "$name"; done
# 4. Strays by name.
PATTERNS=(teleop_keys.py drive_hold.py scan_monitor.py async_slam_toolbox_node rviz2 v4l2_camera_node odom_to_tf.py
          static_transform_publisher scan_filter.py sim_robot.py micro_ros_agent sllidar_node
          map_saver_cli nav2_ component_container lifecycle_manager opennav_docking
          'ros2 launch .*campus_bot_fw' 'ros2 launch nav2_bringup')
for p in "${PATTERNS[@]}"; do pkill -INT -f "$p" 2>/dev/null && info "stray: $p"; done
sleep 2
for p in "${PATTERNS[@]}"; do pkill -KILL -f "$p" 2>/dev/null || true; done
left="$(pgrep -af "$(IFS='|'; echo "${PATTERNS[*]}")" | grep -v -e pgrep -e 'stop.sh' || true)"
if [[ -n "$left" ]]; then warn "still running:"; echo "$left" | sed 's/^/      /'; else ok "all stopped"; fi
