#!/usr/bin/env bash
# Written 2026-10-01: hold-to-drive keyboard control of the real robot.
#
#   run/drive.sh [--speed 0.25] [--turn 0.6]
#   run/drive.sh --terminal            # keyboard in this terminal instead (automatic over plain SSH)
#
# Connects to the ESP32 (starts the micro-ROS agent unless run/remap.sh's bring-up already has it,
# and resets the ESP32 safely if it doesn't answer), then opens the hold-to-drive WINDOW
# (tools/drive_hold.py). Click that window and hold the keys there: the robot moves only while a key
# is held. Closing the window stops the robot; the agent is stopped too if this script started it.
# Run it on the Pi desktop (monitor or xrdp). Plain SSH has no window to catch key presses.
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

TERMINAL=0
ARGS=()
for a in "$@"; do [[ "$a" == --terminal ]] && TERMINAL=1 || ARGS+=("$a"); done
if (( ! TERMINAL )) && [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  TERMINAL=1   # plain SSH: no window possible
fi
if (( TERMINAL )) && [[ ! -t 0 ]]; then err "--terminal needs an interactive terminal"; exit 1; fi

STARTED_AGENT=0 LINK_UP=0
cleanup() {
  # 3 zeros, not 1: the ESP32's cmd_vel subscriber is best-effort, so a single message can be lost.
  if (( LINK_UP )); then
    timeout 4 ros2 topic pub -w 1 -t 3 -r 10 "$CMD_VEL_TOPIC" geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  fi
  if (( STARTED_AGENT )); then stop_bg agent; fi
}
trap cleanup EXIT

start_agent() {  # sets AGENT_FROM to the log line where this agent's output starts
  AGENT_FROM=$(( $(wc -l <"$LOG_DIR/agent.log" 2>/dev/null || echo 0) + 1 ))
  start_bg agent ros2 run micro_ros_agent micro_ros_agent serial --dev "$ESP32_DEV" -b "$AGENT_BAUD"
  STARTED_AGENT=1
}

say "ESP32 link"
resolve_ports
if is_running bringup && [[ "$(cat "$RUN_DIR/bringup.mode" 2>/dev/null)" == sim ]]; then
  err "A SIMULATED bring-up (run/remap.sh --sim) is running: this would drive the simulator."
  err "Stop it first: run/stop.sh"
  exit 1
elif is_running bringup; then
  ok "using the agent from run/remap.sh's bring-up"
  if ! wait_topic "$ODOM_TOPIC" 5; then
    info "no $ODOM_TOPIC: resetting the ESP32 so it reconnects"
    esp32_reset "$ESP32_DEV" || true
    wait_topic "$ODOM_TOPIC" 15 || { err "still no $ODOM_TOPIC. See $LOG_DIR/bringup.log"; exit 1; }
  fi
else
  [[ -e "$ESP32_DEV" ]] || { err "ESP32 not found ($ESP32_DEV). Plug in its native USB-C port (Espressif 303a:1001)."; exit 1; }
  stop_bg agent >/dev/null   # a leftover agent from a crashed run would hold the port
  if port_busy "$ESP32_DEV"; then exit 1; fi
  start_agent
  info "agent on $ESP32_DEV @ $AGENT_BAUD (log: $LOG_DIR/agent.log)"
  if ! agent_session "$LOG_DIR/agent.log" "$AGENT_FROM" 3; then
    # Not answering: in the ROM download loader, or waiting on an old session. Reset it properly
    # (with the agent stopped, so the reset can read the boot banner and confirm the firmware started).
    info "ESP32 not answering: resetting it"
    stop_bg agent >/dev/null
    if ! esp32_reset "$ESP32_DEV"; then
      err "The ESP32 won't boot its firmware. Unplug its USB for 3 s, plug it back in, and re-run."
      exit 1
    fi
    start_agent
    if ! agent_session "$LOG_DIR/agent.log" "$AGENT_FROM" 15; then
      err "No micro-ROS session from the ESP32 after a reset. Last agent lines:"
      tail -5 "$LOG_DIR/agent.log" | sed 's/\x1b\[[0-9;]*m//g; s/^/      /' >&2
      err "Check that the cable is in the ESP32's native USB-C port and AGENT_BAUD in config/robot.env."
      exit 1
    fi
  fi
  ok "micro-ROS session up"
  wait_topic "$ODOM_TOPIC" 10 || { err "session up but no $ODOM_TOPIC. See $LOG_DIR/agent.log"; exit 1; }
fi
ok "ESP32 connected ($ODOM_TOPIC live)"
LINK_UP=1

say "Driving"
info "First time: wheels off the ground. Pack 22–30 V."
if (( TERMINAL )); then
  # A terminal (SSH) only ever sees key presses and their auto-repeat, never the release, so this
  # driver keeps moving only while repeats keep arriving: it stops 0.6 s after the last one. That also
  # makes it stop by itself when the WiFi drops.
  info "Terminal driving: hold ↑↓←→ (or i , j l). It stops 0.6 s after you let go. q quits."
  trap ':' INT   # Ctrl-C goes to the driver, which ramps the robot to a stop
  python3 nodes/teleop_keys.py --purpose drive --topic "$CMD_VEL_TOPIC" --speed "$MAP_LIN_SPEED" \
    --turn "$MAP_ANG_SPEED" --max-speed "$MAX_LIN_SPEED" --max-turn "$MAX_ANG_SPEED" \
    --accel "$MAX_LIN_ACCEL" --ang-accel "$MAX_ANG_ACCEL" || true
  trap on_int INT
else
  info "A window called 'Campus Bot: hold to drive' opens now. CLICK IT, then hold the keys THERE"
  info "(keys typed in this terminal do nothing). ↑↓←→ / WASD drive, SPACE stop, Esc quit."
  [[ -n "${XRDP_SESSION:-}" ]] && info "Remote desktop: the robot stops by itself if the link goes silent for 1 s."
  python3 tools/drive_hold.py "${ARGS[@]}"
fi
