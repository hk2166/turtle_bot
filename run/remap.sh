#!/usr/bin/env bash
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
# The whole mapping flow in one command:
#
#   run/remap.sh <area> [--nav] [--sim] [--no-rviz] [--yes]
#
#   1. power check        2. sensors: lidar + scan_filter + micro-ROS agent + static TFs
#   3. odom → base_link   4. camera off + pre-flight (the readiness sweep)
#   5. slam_toolbox + RViz
#   6. you drive with the keyboard. Press q → saves maps/<area>.{yaml,pgm}, maps/<area>_clean.{yaml,pgm},
#      maps/<area>.posegraph/.data (to extend the map later) and maps/<area>_report.txt
#   --nav      then go straight into AMCL + Nav2 on the clean map (set goals in RViz: "Nav2 Goal")
#   --sim      no hardware: nodes/sim_robot.py fakes the lidar and ESP32 (for testing)
#   --no-rviz  don't open RViz (e.g. over plain SSH)
#   --yes      don't stop to ask when the pre-flight isn't all green
#
# Idempotent: re-running (after a reboot, a crash or Ctrl-C) re-checks everything and reuses the
# sensor bring-up if it's already healthy. run/stop.sh tears it all down.
set -Eeuo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

AREA="" NAV=0 SIM=0 RVIZ=1 YES=0
for a in "$@"; do
  case "$a" in
    --nav) NAV=1 ;; --sim) SIM=1 ;; --no-rviz) RVIZ=0 ;; --yes|-y) YES=1 ;;
    -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
    -*) echo "Unknown option $a (see --help)" >&2; exit 2 ;;
    *) [[ -z "$AREA" ]] || { echo "Only one area name, please" >&2; exit 2; }; AREA="$a" ;;
  esac
done
[[ -n "$AREA" ]] || { sed -n '2,21p' "$0"; exit 2; }
[[ "$AREA" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "Area name: letters, digits, _ and - only" >&2; exit 2; }
[[ -z "${DISPLAY:-}" ]] && RVIZ=0
MAP="$PROJECT_DIR/maps/$AREA"

ask() {  # ask "question" default(y|n) → true for yes
  local reply def="$2"
  (( YES )) && [[ "$def" == y ]] && return 0
  read -r -p "    $1 " reply || reply=""
  reply="${reply:-$def}"
  [[ "$reply" =~ ^[Yy] ]]
}

# ------------------------------------------------------------------------------------------------
say "1/6 Power"
if run/power_check.sh --brief | sed 's/^/    /'; then :; else
  warn "5 V rail not clean: the Pi may brown out mid-run. Check the pack (≥ 22 V) and the 5 V buck."
  (( SIM )) || ask "Continue anyway? [y/N]" n || exit 1
fi

# ------------------------------------------------------------------------------------------------
say "2/6 Sensors"
MODE_FILE="$RUN_DIR/bringup.mode"
WANT_MODE=$([[ $SIM == 1 ]] && echo sim || echo real)
if is_running bringup && [[ "$(cat "$MODE_FILE" 2>/dev/null)" == "$WANT_MODE" ]] \
   && wait_topic "$SCAN_FILTERED_TOPIC" 5; then
  ok "sensor bring-up already running ($WANT_MODE), reusing it"
else
  stop_bg bringup; stop_bg odom_tf
  if (( SIM )); then
    LIDAR_DEV=sim ESP32_DEV=sim
    info "SIMULATION: no hardware used"
  else
    resolve_ports
    [[ -e "$LIDAR_DEV" ]] || { err "Lidar port $LIDAR_DEV not found. Plug in the CP2102 stick (ls /dev/ttyUSB*)."; exit 1; }
    [[ -e "$ESP32_DEV" ]] || { err "ESP32 port $ESP32_DEV not found. Plug in the ESP32's UART USB-C port."; exit 1; }
    info "lidar $LIDAR_DEV @ $LIDAR_BAUD   ESP32 $ESP32_DEV @ $AGENT_BAUD"
  fi
  start_bg bringup ros2 launch "$PROJECT_DIR/launch/bringup.launch.py" \
    sim:=$([[ $SIM == 1 ]] && echo true || echo false) \
    lidar_port:="$LIDAR_DEV" lidar_baud:="$LIDAR_BAUD" lidar_scan_mode:="$LIDAR_SCAN_MODE" \
    esp32_port:="$ESP32_DEV" agent_baud:="$AGENT_BAUD" \
    scan_topic:="$SCAN_TOPIC" scan_filtered_topic:="$SCAN_FILTERED_TOPIC" \
    laser_x:="$LASER_X" laser_y:="$LASER_Y" laser_z:="$LASER_Z" laser_yaw:="$LASER_YAW" \
    imu_x:="$IMU_X" imu_y:="$IMU_Y" imu_z:="$IMU_Z" \
    camera_x:="$CAMERA_X" camera_y:="$CAMERA_Y" camera_z:="$CAMERA_Z" camera_pitch:="$CAMERA_PITCH"
  echo "$WANT_MODE" >"$MODE_FILE"
  if wait_topic "$SCAN_FILTERED_TOPIC" 25; then ok "lidar scanning, columns filtered"
  else
    err "No $SCAN_FILTERED_TOPIC after 25 s. Last lines of $LOG_DIR/bringup.log:"
    tail -15 "$LOG_DIR/bringup.log" | sed 's/^/      /'
    err "Check: lidar 5 V (red) and GND, yellow→RXD, green→TXD, baud 460800."
    exit 1
  fi
fi
if wait_topic "$ODOM_TOPIC" 20; then ok "$ODOM_TOPIC live (ESP32 connected)"
else
  err "No $ODOM_TOPIC after 20 s: the ESP32 hasn't connected to the micro-ROS agent."
  err "Press the ESP32's RST/EN button (its client only looks for the agent at boot), then re-run."
  err "Still nothing? Check AGENT_BAUD in config/robot.env against the firmware, and $LOG_DIR/bringup.log."
  exit 1
fi

# ------------------------------------------------------------------------------------------------
say "3/6 odom → base_link"
if is_running odom_tf; then ok "odom_to_tf.py running"
elif python3 tools/pose_tool.py probe odom base_link --timeout 3; then ok "provided by the firmware"
else
  start_bg odom_tf python3 nodes/odom_to_tf.py --ros-args -p odom_topic:="$ODOM_TOPIC"
  python3 tools/pose_tool.py probe odom base_link --timeout 5 && ok "odom_to_tf.py broadcasting it" \
    || { err "odom → base_link still missing: see $LOG_DIR/odom_tf.log"; exit 1; }
fi

# ------------------------------------------------------------------------------------------------
say "4/6 Pre-flight"
run/camera.sh off >/dev/null
if python3 tools/preflight.py $([[ $SIM == 1 ]] && echo --sim) --lidar-port "${LIDAR_DEV:-}"; then :; else
  ask "Pre-flight is NOT green. Map anyway? [y/N]" n || { info "Fix it and re-run; the sensors stay up (run/stop.sh to stop)."; exit 1; }
fi

# ------------------------------------------------------------------------------------------------
say "5/6 SLAM + RViz"
stop_bg nav; stop_bg slam   # a fresh map every run
start_bg slam ros2 launch "$PROJECT_DIR/launch/slam.launch.py"
if wait_topic /map 30; then ok "slam_toolbox mapping (config/slam_toolbox.yaml)"
else err "slam_toolbox published no /map in 30 s: see $LOG_DIR/slam.log"; exit 1; fi
if (( RVIZ )); then
  is_running rviz || start_bg rviz rviz2 -d "$PROJECT_DIR/config/campus_bot.rviz"
  ok "RViz open"
fi

# ------------------------------------------------------------------------------------------------
say "6/6 Drive and map '$AREA'"
cat <<'TIPS'
    Office plan: perimeter first, then the desk aisles and the café corner; finish where you started
    so the loop closes. Slow and 0.5–1 m off the glass room (matte tape at ~215 mm). Turn gently.
    Map when the floor is quiet (people and chairs leave ghosts). Watch the pack voltage (≥ 22 V).
TIPS
trap ':' INT      # Ctrl-C goes to the teleop (which stops the robot), not to this script
set +e
python3 nodes/teleop_keys.py --topic "$CMD_VEL_TOPIC" --speed "$MAP_LIN_SPEED" --turn "$MAP_ANG_SPEED" \
  --max-speed "$MAX_LIN_SPEED" --max-turn "$MAX_ANG_SPEED" --accel "$MAX_LIN_ACCEL" --ang-accel "$MAX_ANG_ACCEL"
rc=$?
set -e
trap - INT
if (( rc != 0 )); then
  ask "Save the map as '$AREA'? [Y/n]" y || { info "Not saved. SLAM is still running; run/stop.sh to stop."; exit 0; }
fi

# ------------------------------------------------------------------------------------------------
say "Saving maps/$AREA"
if [[ -f "$MAP.yaml" ]]; then
  PREV="$PROJECT_DIR/maps/.prev/$AREA-$(date +%Y%m%d-%H%M%S)"
  mkdir -p "$PREV"
  mv "$MAP".* "$MAP"_clean.* "$MAP"_report.txt "$PREV"/ 2>/dev/null || true
  info "previous '$AREA' map moved to maps/.prev/$(basename "$PREV")"
fi
ros2 run nav2_map_server map_saver_cli -f "$MAP" --ros-args -p save_map_timeout:=10.0 >"$LOG_DIR/map_saver.log" 2>&1 \
  || { err "map_saver failed: see $LOG_DIR/map_saver.log. SLAM is still running: fix and save by hand:"
       err "  ros2 run nav2_map_server map_saver_cli -f $MAP"; exit 1; }
ok "maps/$AREA.yaml + .pgm"
if timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "{filename: '$MAP'}" >/dev/null 2>&1 && [[ -f "$MAP.posegraph" ]]; then
  ok "maps/$AREA.posegraph (continue this map later with slam_toolbox)"
else warn "pose graph not saved (optional)"; fi
say "Cleaning + stats"
python3 tools/clean_map.py "$MAP.yaml" | sed 's/^/    /'
ok "maps/${AREA}_clean.yaml + .pgm, maps/${AREA}_report.txt"
(( RVIZ )) && info "Look at it: eog maps/${AREA}_clean.pgm"

# ------------------------------------------------------------------------------------------------
if (( NAV )); then
  say "Autonomy: AMCL + Nav2 on maps/${AREA}_clean.yaml"
  POSE="$(python3 tools/pose_tool.py get || true)"
  stop_bg slam
  start_bg nav ros2 launch nav2_bringup bringup_launch.py \
    map:="${MAP}_clean.yaml" params_file:="$PROJECT_DIR/config/nav2_params.yaml" \
    use_sim_time:=False autostart:=True use_composition:=True
  if timeout 40 bash -c 'until ros2 node list 2>/dev/null | grep -q /amcl; do sleep 1; done'; then
    sleep 5
    if [[ -n "$POSE" ]]; then
      # shellcheck disable=SC2086
      python3 tools/pose_tool.py set $POSE && ok "initial pose = where mapping ended ($POSE)"
    else warn "set the initial pose in RViz (2D Pose Estimate)"; fi
    ok "Nav2 up. In RViz: 'Nav2 Goal' to send the robot somewhere. Logs: $LOG_DIR/nav.log"
  else
    err "Nav2 didn't come up: see $LOG_DIR/nav.log"
  fi
  info "Stop everything with: run/stop.sh"
else
  stop_bg slam
  ok "Done. Sensors still running (re-run remap.sh to map again). Stop everything: run/stop.sh"
  info "Camera back on after mapping: run/camera.sh on"
fi
