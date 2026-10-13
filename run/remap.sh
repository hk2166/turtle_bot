#!/usr/bin/env bash
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
# The whole mapping flow in one command:
#
#   run/remap.sh <area> [--nav | --nav-only] [--no-camera] [--sim] [--no-rviz] [--yes]
#
#   1. power check        2. sensors: lidar + scan_filter + micro-ROS agent + static TFs
#   3. odom → base_link   4. pre-flight (the readiness sweep), then the camera + tag recorder
#   5. slam_toolbox + RViz
#   6. you drive with the keyboard. Press q → saves maps/<area>.{yaml,pgm}, maps/<area>_clean.{yaml,pgm},
#      maps/<area>.posegraph/.data (to extend the map later), maps/<area>_report.txt and
#      maps/<area>_tags.yaml (the AprilTags the camera saw, in map coordinates)
#   --nav       then go straight into AMCL + Nav2 on the clean map (set goals in RViz: "Nav2 Goal").
#               With maps/<area>_tags.yaml, the camera sets AMCL's pose when it has none or is lost.
#   --nav-only  no mapping: AMCL + Nav2 on the SAVED maps/<area>_clean.yaml (run/nav.sh <area>)
#   --continue  don't start fresh: reload maps/<area>.posegraph and EXTEND the map, then drive only
#               the areas that need improving. Saving replaces maps/<area> (old files go to maps/.prev).
#               The robot starts on the ORIGINAL mapping start spot, unless you give
#   --start=X,Y,YAW_DEG  where it stands on that map instead (rough is fine: it's refined by fitting
#               the live scan to the saved map). Read X,Y,YAW off the dashboard POSE while navigating.
#   --no-camera mapping/navigation without the camera and tags
#   --sim       no hardware: nodes/sim_robot.py fakes the lidar and ESP32 (for testing; no camera)
#   --no-rviz   don't open RViz (e.g. over plain SSH)
#   --yes       don't stop to ask when the pre-flight isn't all green
#
# Idempotent: re-running (after a reboot, a crash or Ctrl-C) re-checks everything and reuses the
# sensor bring-up if it's already healthy. run/stop.sh tears it all down.
set -Eeuo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

AREA="" NAV=0 NAV_ONLY=0 CAMERA=1 SIM=0 RVIZ=1 YES=0 CONT=0 PREP_ONLY=0 START=""
for a in "$@"; do
  case "$a" in
    --nav) NAV=1 ;; --nav-only) NAV=1 NAV_ONLY=1 ;; --no-camera) CAMERA=0 ;; --continue) CONT=1 ;;
    --prepare-only) PREP_ONLY=1 ;;
    --sim) SIM=1 ;; --no-rviz) RVIZ=0 ;; --yes|-y) YES=1 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    --start=*) START="${a#--start=}" ;;
    -*) echo "Unknown option $a (see --help)" >&2; exit 2 ;;
    *) [[ -z "$AREA" ]] || { echo "Only one area name, please" >&2; exit 2; }; AREA="$a" ;;
  esac
done
[[ -n "$AREA" ]] || { sed -n '2,30p' "$0"; exit 2; }
[[ "$AREA" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "Area name: letters, digits, _ and - only" >&2; exit 2; }
if (( RVIZ )) && ! use_robot_screen; then RVIZ=0; fi
MAP="$PROJECT_DIR/maps/$AREA"
TAGS_REC="$RUN_DIR/tags_recording.yaml"
(( SIM )) && CAMERA=0
if (( NAV_ONLY )) && [[ ! -f "${MAP}_clean.yaml" ]]; then
  echo "No saved map maps/${AREA}_clean.yaml. Map it first: run/remap.sh $AREA" >&2; exit 1
fi

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
    # The other tools' drivers (run/lidar_view.sh, run/drive.sh, hw_check) would share the ports with
    # this bring-up and split the data: stop them, then make sure nothing else holds a port.
    for n in lidar rviz_lidar agent hw_lidar hw_agent; do is_running "$n" && { stop_bg "$n" >/dev/null; info "stopped $n (needs its port)"; }; done
    for d in "$LIDAR_DEV" "$ESP32_DEV"; do if port_busy "$d"; then exit 1; fi; done
    [[ -e "$LIDAR_DEV" ]] || { err "Lidar port $LIDAR_DEV not found. Plug in the CP2102 stick (ls /dev/ttyUSB*)."; exit 1; }
    [[ -e "$ESP32_DEV" ]] || { err "ESP32 port $ESP32_DEV not found. Plug in the ESP32's native USB-C port (303a:1001)."; exit 1; }
    info "lidar $LIDAR_DEV @ $LIDAR_BAUD   ESP32 $ESP32_DEV @ $AGENT_BAUD"
    # the C1 sometimes boots into "internal error" (health 2) and sllidar_node just exits; its own
    # RESET command clears it (2026-10-13), so check health first and reset when needed
    if python3 tools/lidar_reset.py --port "$LIDAR_DEV" --baud "$LIDAR_BAUD" | sed 's/^/    /'; then :
    else warn "lidar still unhealthy after a software reset: unplug its USB stick for 5 s"; fi
  fi
  start_bg bringup ros2 launch "$PROJECT_DIR/launch/bringup.launch.py" \
    sim:=$([[ $SIM == 1 ]] && echo true || echo false) \
    lidar_port:="$LIDAR_DEV" lidar_baud:="$LIDAR_BAUD" lidar_scan_mode:="$LIDAR_SCAN_MODE" \
    esp32_port:="$ESP32_DEV" agent_baud:="$AGENT_BAUD" \
    scan_topic:="$SCAN_TOPIC" scan_filtered_topic:="$SCAN_FILTERED_TOPIC" \
    scan_min_range:="${SCAN_MIN_RANGE:-0.25}" \
    laser_x:="$LASER_X" laser_y:="$LASER_Y" laser_z:="$LASER_Z" laser_yaw:="$LASER_YAW" \
    imu_x:="$IMU_X" imu_y:="$IMU_Y" imu_z:="$IMU_Z" \
    camera_x:="$CAMERA_X" camera_y:="$CAMERA_Y" camera_z:="$CAMERA_Z" \
    camera_roll:="${CAMERA_ROLL:-0.0}" camera_pitch:="$CAMERA_PITCH" camera_yaw:="${CAMERA_YAW:-0.0}"
  echo "$WANT_MODE" >"$MODE_FILE"
  if wait_topic "$SCAN_FILTERED_TOPIC" 25; then ok "lidar scanning, columns filtered"
  else
    err "No $SCAN_FILTERED_TOPIC after 25 s. Last lines of $LOG_DIR/bringup.log:"
    tail -15 "$LOG_DIR/bringup.log" | sed 's/^/      /'
    err "Check: lidar 5 V (red) and GND, yellow→RXD, green→TXD, baud 460800."
    exit 1
  fi
fi
# Foxglove on the Mac/phone (README → Remote access): a websocket bridge on port 8765. Optional:
# installed by  sudo apt install ros-jazzy-foxglove-bridge
if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1 && ! is_running foxglove; then
  start_bg foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
  ok "Foxglove bridge on ws://$(hostname).local:8765 (view/drive from the Mac, no VNC)"
fi
if ! is_running dashboard; then
  start_bg dashboard python3 "$PROJECT_DIR/nodes/dashboard.py"
  ok "web mission control on http://$(hostname).local:8088"
fi
if wait_topic "$ODOM_TOPIC" 8; then ok "$ODOM_TOPIC live (ESP32 connected)"
elif (( SIM )); then err "No $ODOM_TOPIC from sim_robot.py: see $LOG_DIR/bringup.log"; exit 1
else
  # Not answering: sitting in the ROM download loader, or tied to an old agent session. A safe
  # reset (tools/esp32_reset.py) boots its firmware, which then connects to this agent.
  info "no $ODOM_TOPIC yet: resetting the ESP32"
  esp32_reset "${ESP32_DEV:-/dev/esp32}" || true
  if wait_topic "$ODOM_TOPIC" 15; then ok "$ODOM_TOPIC live (ESP32 connected after a reset)"
  else
    err "No $ODOM_TOPIC: the ESP32 hasn't connected to the micro-ROS agent."
    err "Check its cable is in the NATIVE USB-C port (lsusb shows 303a:1001), then run:"
    err "  run/stop.sh && python3 tools/esp32_reset.py     (prints whether the firmware boots)"
    err "Agent output: $LOG_DIR/bringup.log"
    exit 1
  fi
fi

# ------------------------------------------------------------------------------------------------
say "3/6 odom → base_link"
if is_running odom_tf; then ok "odom_to_tf.py running"
elif python3 tools/pose_tool.py probe odom base_link --timeout 3; then ok "provided by the firmware"
else
  start_bg odom_tf python3 nodes/odom_to_tf.py --ros-args -p odom_topic:="$ODOM_TOPIC" \
    -p odom_lin_scale:="${ODOM_LIN_SCALE:-1.0}" -p odom_ang_scale:="${ODOM_ANG_SCALE:-1.0}"
  python3 tools/pose_tool.py probe odom base_link --timeout 5 && ok "odom_to_tf.py broadcasting it" \
    || { err "odom → base_link still missing: see $LOG_DIR/odom_tf.log"; exit 1; }
fi

# ------------------------------------------------------------------------------------------------
say "4/6 Pre-flight"
stop_bg tags
run/camera.sh off >/dev/null
if python3 tools/preflight.py $([[ $SIM == 1 ]] && echo --sim) --lidar-port "${LIDAR_DEV:-}"; then :; else
  ask "Pre-flight is NOT green. Map anyway? [y/N]" n || { info "Fix it and re-run; the sensors stay up (run/stop.sh to stop)."; exit 1; }
fi
CAM_UP=0
if (( CAMERA )); then
  if run/camera.sh on; then CAM_UP=1
  else warn "no camera: carrying on without tag recording/localization"; fi
fi

if (( ! NAV_ONLY )); then   # ---------------------------- mapping (steps 5–6 and saving) ----------------

# ------------------------------------------------------------------------------------------------
say "5/6 SLAM + RViz"
stop_bg nav; stop_bg slam   # a fresh map every run (unless --continue)
SLAM_PARAMS="$PROJECT_DIR/config/slam_toolbox.yaml"
if (( CONT )); then
  [[ -f "$PROJECT_DIR/maps/$AREA.posegraph" ]] || { err "no maps/$AREA.posegraph to continue from"; exit 1; }
  SX=0 SY=0 SYAW=0
  if [[ -n "$START" ]]; then
    IFS=, read -r GX GY GYAW <<<"$START"
    GYAW="${GYAW:-}"
    if [[ -z "$GYAW" ]]; then
      info "no heading given: searching all directions around ($GX, $GY)"
      FITARGS=("$GX" "$GY" 0 --radius 1.0 --angle 180)
    else
      FITARGS=("$GX" "$GY" "$GYAW" --radius 1.0 --angle 30)
    fi
    info "refining the start by fitting the live scan to maps/${AREA}_clean ..."
    if fit="$(python3 tools/fit_pose.py "${FITARGS[@]}" --map-file "$PROJECT_DIR/maps/${AREA}_clean.yaml" 2>/dev/null)"; then
      read -r FX FY FYAW SCORE <<<"$fit"
      PCT="$(python3 -c "print(round(100*$SCORE))")"
      if (( PCT >= 60 )); then
        SX="$FX" SY="$FY" SYAW="$FYAW"
        ok "start = ($FX, $FY, $(python3 -c "import math; print(round(math.degrees($FYAW)))")°), $PCT% of the scan on map walls"
      elif [[ -n "$GYAW" ]]; then
        warn "weak fit ($PCT%): using your numbers as given"
        SX="$GX" SY="$GY" SYAW="$(python3 -c "import math; print(math.radians($GYAW))")"
      else
        err "weak fit ($PCT%) and no heading given: re-run with --start=X,Y,YAW_DEG"; exit 1
      fi
    else
      err "scan fit failed (lidar up?)"; exit 1
    fi
    ask "Use that start? [y/N]" n || exit 1
  else
    info "CONTINUING maps/$AREA from the ORIGINAL mapping start spot (the map's 0,0, facing the original"
    info "direction). Starting elsewhere? Re-run with --start=X,Y[,YAW_DEG]."
    ask "Is the robot there now? [y/N]" n || { info "Place it first, then re-run."; exit 1; }
  fi
  SLAM_PARAMS="$RUN_DIR/slam_continue.yaml"
  python3 - "$PROJECT_DIR/config/slam_toolbox.yaml" "$PROJECT_DIR/maps/$AREA" "$SLAM_PARAMS" "$SX" "$SY" "$SYAW" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
node = next(iter(cfg))
cfg[node]['ros__parameters'].update({'map_file_name': sys.argv[2],
                                     'map_start_pose': [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]})
yaml.safe_dump(cfg, open(sys.argv[3], 'w'))
PY
  ok "slam_toolbox will reload the saved pose graph and extend it"
fi
start_bg slam ros2 launch "$PROJECT_DIR/launch/slam.launch.py" params_file:="$SLAM_PARAMS"
if wait_topic /map 30; then ok "slam_toolbox mapping (config/slam_toolbox.yaml)"
else err "slam_toolbox published no /map in 30 s: see $LOG_DIR/slam.log"; exit 1; fi
if (( RVIZ )); then
  is_running rviz || start_bg rviz rviz2 -d "$PROJECT_DIR/config/campus_bot.rviz"
  ok "RViz open"
fi
if (( CAM_UP )); then
  rm -f "$TAGS_REC"
  start_bg tags python3 nodes/tag_map.py --ros-args -p mode:=record -p tags_file:="$TAGS_REC" \
    -p odom_topic:="$ODOM_TOPIC"
  ok "camera recording AprilTags into the map (log: $LOG_DIR/tags.log)"
fi

if (( PREP_ONLY )); then
  ok "prepared: sensors + SLAM are running. Drive (dashboard MANUAL tab), then save:"
  info "  run/map_ctl.sh save $AREA      (or discard: run/map_ctl.sh discard)"
  exit 0
fi

# ------------------------------------------------------------------------------------------------
say "6/6 Drive and map '$AREA'"
cat <<'TIPS'
    Office plan: perimeter first, then the desk aisles and the café corner; finish where you started
    so the loop closes. Slow and 0.5–1 m off the glass room (matte tape at ~215 mm). Turn gently.
    Map when the floor is quiet (people and chairs leave ghosts). Watch the pack voltage (≥ 22 V).
    Tags: drive SLOWLY past each one (they're only recorded below 0.3 m/s, within 3 m), and pause
    facing each pair for a second or two.
TIPS
trap ':' INT      # Ctrl-C goes to the teleop (which stops the robot), not to this script
set +e
python3 nodes/teleop_keys.py --topic "$CMD_VEL_TOPIC" --speed "$MAP_LIN_SPEED" --turn "$MAP_ANG_SPEED" \
  --max-speed "$MAX_LIN_SPEED" --max-turn "$MAX_ANG_SPEED" --accel "$MAX_LIN_ACCEL" --ang-accel "$MAX_ANG_ACCEL"
rc=$?
set -e
trap on_int INT   # back to lib.sh's handler
if (( rc != 0 )); then
  ask "Save the map as '$AREA'? [Y/n]" y || { info "Not saved. SLAM is still running; run/stop.sh to stop."; exit 0; }
fi

# ------------------------------------------------------------------------------------------------
say "Saving maps/$AREA"
if [[ -f "$MAP.yaml" ]]; then
  PREV="$PROJECT_DIR/maps/.prev/$AREA-$(date +%Y%m%d-%H%M%S)"
  mkdir -p "$PREV"
  mv "$MAP".* "$MAP"_clean.* "$MAP"_report.txt "$MAP"_tags.yaml "$PREV"/ 2>/dev/null || true
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
if is_running tags; then
  stop_bg tags >/dev/null          # writes its last summary on the way out
  ntags="$(python3 -c 'import sys,yaml; print(len((yaml.safe_load(open(sys.argv[1])) or {}).get("tags") or {}))' "$TAGS_REC" 2>/dev/null || echo 0)"
  if (( ntags > 0 )); then
    cp "$TAGS_REC" "${MAP}_tags.yaml"
    ok "maps/${AREA}_tags.yaml: $ntags tag(s) $(python3 -c 'import sys,yaml; print(sorted((yaml.safe_load(open(sys.argv[1])) or {}).get("tags") or {}))' "$TAGS_REC")"
  else
    warn "no AprilTags recorded (none up, or never passed slowly within 3 m): navigation works, without camera help"
  fi
fi

fi   # ---------------------------------------------------- end of mapping (NAV_ONLY skips it) --------

# ------------------------------------------------------------------------------------------------
if (( NAV )); then
  NAVMAP="${MAP}_clean.yaml"
  if [[ -f "$PROJECT_DIR/config/fence_${AREA}.yaml" ]]; then
    python3 tools/fence_map.py bake "$AREA" >/dev/null && NAVMAP="${MAP}_fenced.yaml" \
      && ok "virtual boundary applied (config/fence_${AREA}.yaml)"
  fi
  say "Autonomy: AMCL + Nav2 on $(basename "$NAVMAP")"
  POSE=""
  (( NAV_ONLY )) || POSE="$(python3 tools/pose_tool.py get || true)"
  stop_bg slam; stop_bg nav; stop_bg loc
  if (( RVIZ )) && ! is_running rviz; then start_bg rviz rviz2 -d "$PROJECT_DIR/config/campus_bot.rviz"; fi
  # One composed process (separate processes loaded the Pi to a load average of 20). Until the robot
  # has a pose the planner waits for it (initial_transform_timeout in config/nav2_params.yaml).
  # launch/nav2.launch.py = the stock bringup_launch.py with a 10 s lifecycle bond timeout.
  start_bg nav ros2 launch "$PROJECT_DIR/launch/nav2.launch.py" \
    map:="$NAVMAP" params_file:="$PROJECT_DIR/config/nav2_params.yaml" \
    use_sim_time:=False autostart:=True use_composition:=True
  if ! timeout --foreground 60 bash -c 'until ros2 lifecycle get /amcl 2>/dev/null | grep -q "^active"; do sleep 1; done'; then
    err "AMCL didn't come up: see $LOG_DIR/nav.log"; exit 1
  fi
  ok "map server + AMCL up (log: $LOG_DIR/nav.log)"
  if [[ -n "$POSE" ]]; then
    # shellcheck disable=SC2086
    python3 tools/pose_tool.py set $POSE && ok "initial pose = where mapping ended ($POSE)"
  fi
  if [[ "${AUTO_CLEAR:-1}" == 1 ]]; then
    stop_bg auto >/dev/null
    start_bg auto python3 nodes/auto_clear.py
    ok "idle behaviour on: boxed in + idle ~1 min → moves itself to open floor (AUTO_CLEAR=0 disables)"
  fi
  stop_bg pose_keeper >/dev/null
  start_bg pose_keeper python3 nodes/pose_keeper.py --ros-args -p map:="$AREA"
  # After a reboot: the pose saved while driving, accepted only if the live scan fits the map there
  # (the robot may have been carried while off). Robots don't move while off, so age doesn't matter.
  LAST="$RUN_DIR/last_pose_$AREA.yaml"
  if [[ -z "$POSE" && -f "$LAST" ]] && ! python3 tools/pose_tool.py probe map base_link --timeout 1; then
    read -r LX LY LYAW < <(python3 -c 'import sys,yaml,math; d=yaml.safe_load(open(sys.argv[1])); print(d["x"], d["y"], math.degrees(d["yaw"]))' "$LAST")
    if FIT=$(python3 tools/fit_pose.py "$LX" "$LY" "$LYAW" --radius 1.0 --angle 25 2>/dev/null) \
       && read -r FX FY FYAW FSC <<<"$FIT" && python3 -c "import sys; sys.exit(float(sys.argv[1]) < 0.7)" "$FSC"; then
      if python3 tools/pose_tool.py set "$FX" "$FY" "$FYAW"; then
        ok "pose restored from before the restart: ($FX, $FY), lidar agrees ($FSC of the scan on walls)"
      else
        warn "pose ($FX, $FY) found but AMCL didn't take it: set it by hand"
      fi
    else
      warn "saved pose ($LX, $LY) doesn't match the lidar (${FSC:-no fit}): robot moved while off? set it by hand"
    fi
  fi
  if (( CAM_UP )) && [[ -f "${MAP}_tags.yaml" ]]; then
    stop_bg tags >/dev/null
    start_bg tags python3 nodes/tag_map.py --ros-args -p mode:=localize -p tags_file:="${MAP}_tags.yaml" \
      -p odom_topic:="$ODOM_TOPIC"
    ok "camera localization on: a tag pair in view sets/corrects AMCL's pose (log: $LOG_DIR/tags.log)"
  fi
  if ! python3 tools/pose_tool.py probe map base_link --timeout 3; then
    say "Waiting for the robot's pose"
    info "Robot near where mapping started?  run/reset_pose.sh   (fits the scan to the map there)"
    if (( RVIZ )); then info "Or in RViz: click '2D Pose Estimate' FIRST, then click where the robot is and drag the way it faces."
    else info "Or open RViz on the robot's desktop (VNC / monitor): rviz2 -d config/campus_bot.rviz"; fi
    (( CAM_UP )) && [[ -f "${MAP}_tags.yaml" ]] && info "Or turn the robot to face a tag pair."
    info "(waits up to 30 min; Ctrl-C stops waiting, Nav2 keeps waiting for a pose)"
    python3 tools/pose_tool.py probe map base_link --timeout 1800 || { err "no pose set"; exit 1; }
  fi
  ok "robot pose known: $(python3 tools/pose_tool.py get 2>/dev/null)"
  if timeout --foreground 90 bash -c 'until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do sleep 1; done'; then
    ok "Nav2 up. In RViz: 'Nav2 Goal' to send the robot somewhere. Logs: $LOG_DIR/nav.log"
  else
    err "Nav2's navigation nodes didn't all start: see $LOG_DIR/nav.log"
  fi
  info "Stop everything with: run/stop.sh"
else
  stop_bg slam
  ok "Done. Sensors still running (re-run remap.sh to map again). Stop everything: run/stop.sh"
fi
