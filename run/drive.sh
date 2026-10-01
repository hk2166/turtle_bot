#!/usr/bin/env bash
# Written 2026-10-01: hold-to-drive keyboard control of the real robot.
#
#   run/drive.sh [--speed 0.25] [--turn 0.6]
#
# Starts the micro-ROS agent for the ESP32 (unless run/remap.sh's bring-up already has it), waits for
# /odom, then opens the hold-to-drive window (tools/drive_hold.py). The robot moves only while a key is
# held. Closing the window stops the robot; the agent is stopped too if this script started it.
# Run it on the Pi desktop (monitor or xrdp). Plain SSH has no window to catch key presses.
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

STARTED_AGENT=0
cleanup() {
  timeout 3 ros2 topic pub --once "$CMD_VEL_TOPIC" geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  (( STARTED_AGENT )) && stop_bg agent
}
trap cleanup EXIT

say "ESP32 link"
if is_running bringup; then
  ok "using the agent from run/remap.sh's bring-up"
else
  resolve_ports
  [[ -e "$ESP32_DEV" ]] || { err "ESP32 not found ($ESP32_DEV). Plug in its native USB-C port."; exit 1; }
  if ! is_running agent; then
    start_bg agent ros2 run micro_ros_agent micro_ros_agent serial --dev "$ESP32_DEV" -b "$AGENT_BAUD"
    STARTED_AGENT=1
  fi
  info "agent on $ESP32_DEV @ $AGENT_BAUD (log: $LOG_DIR/agent.log)"
fi

if ! wait_topic "$ODOM_TOPIC" 6; then
  # The ESP32 only reconnects after a reset if the agent came up after it: pulse RTS (= RESET).
  info "no $ODOM_TOPIC yet, resetting the ESP32 so it reconnects"
  python3 -c "import serial,time; s=serial.Serial('${ESP32_DEV:-/dev/esp32}'); s.rts=True; time.sleep(0.1); s.rts=False" 2>/dev/null || true
  wait_topic "$ODOM_TOPIC" 10 || { err "no $ODOM_TOPIC from the ESP32. See $LOG_DIR/agent.log"; exit 1; }
fi
ok "ESP32 connected ($ODOM_TOPIC live)"

say "Motors: wheels off the ground for the first try. Pack 22–30 V. Hold a key to move, let go to stop."
python3 tools/drive_hold.py "$@"
