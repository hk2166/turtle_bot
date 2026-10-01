#!/usr/bin/env bash
# Written 2026-09-30 (new tool, not a reconstruction): first contact with the hardware.
# Run this the first time the lidar and ESP32 are plugged in, before run/remap.sh. It answers the
# questions config/robot.env can only guess at:
#
#   1. USB serial devices: which is the lidar (CP2102), which is the ESP32, and do the udev names work?
#   2. Lidar alone: does sllidar get scans at 460800?
#   3. ESP32 alone: which port (and, for a UART bridge, which baud) its micro-ROS client talks on.
#      It resets the board only if the firmware doesn't connect by itself.
#   4. ESP32 topics: odom / IMU / cmd_vel names and message types.
#   5. Suggested robot.env values. With --apply they're written into config/robot.env.
#
#   run/hw_check.sh [--apply] [--skip-lidar] [--skip-esp32] [--bauds "115200 921600 ..."]
#
# It stops anything already running first (run/stop.sh) because it needs the serial ports to itself.
# Nothing is written to the ESP32. The resets go through tools/esp32_reset.py (same as the RESET button).
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

APPLY=0 DO_LIDAR=1 DO_ESP=1
BAUDS="115200 921600 460800 230400 2000000"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) APPLY=1 ;; --skip-lidar) DO_LIDAR=0 ;; --skip-esp32) DO_ESP=0 ;;
    --bauds) BAUDS="$2"; shift ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "Unknown option $1" >&2; exit 2 ;;
  esac
  shift
done
declare -A SUGGEST=()
REPORT="$LOG_DIR/hw_check-$(date +%Y%m%d-%H%M%S).txt"
exec > >(tee "$REPORT") 2>&1

bash run/stop.sh >/dev/null 2>&1 || true
# Our test drivers run detached (setsid): make sure Ctrl-C or an error never leaves them behind.
trap 'stop_bg hw_lidar >/dev/null; stop_bg hw_agent >/dev/null' EXIT
for d in /dev/lidar /dev/esp32; do
  if [[ -e "$d" ]] && port_busy "$d"; then err "hw_check needs the serial ports to itself."; exit 1; fi
done

# ------------------------------------------------------------------------------------------------
say "1/5 USB serial devices"
info "groups: $(id -nG | tr ' ' '\n' | grep -xE 'dialout|video' | tr '\n' ' ')(need dialout)"
id -nG | grep -qw dialout || err "not in 'dialout': sudo usermod -aG dialout $USER, then log out and in"
systemctl is-active --quiet brltty 2>/dev/null && warn "brltty is running and may grab USB-serial adapters (02_setup_pi.sh disables it)"
declare -A KIND=() SERIAL=()
PORTS=()
for dev in /dev/ttyUSB* /dev/ttyACM* ${HW_EXTRA_PORTS:-}; do   # HW_EXTRA_PORTS: extra ports (for testing with socat ptys)
  [[ -e "$dev" ]] || continue
  PORTS+=("$dev")
  props="$(udevadm info -q property -n "$dev" 2>/dev/null)"
  vid="$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$props")"; pid="$(sed -n 's/^ID_MODEL_ID=//p' <<<"$props")"
  ser="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"$props")"; model="$(sed -n 's/^ID_MODEL=//p' <<<"$props")"
  case "$vid:$pid" in
    10c4:ea60) kind="CP2102 (lidar stick, or the ESP32 if its bridge is also a CP2102)" ;;
    1a86:*)    kind="WCH CH34x (ESP32 board UART: firmware log only, not micro-ROS)" ;;
    303a:*)    kind="Espressif native USB (ESP32-S3)" ;;
    0403:*)    kind="FTDI" ;;
    *)         kind="unknown" ;;
  esac
  KIND[$dev]="$vid:$pid"; SERIAL[$dev]="$ser"
  info "$dev  $vid:$pid  $model  serial=${ser:-none}  → $kind"
done
((${#PORTS[@]})) || { err "No /dev/ttyUSB* or /dev/ttyACM*. Plug in the lidar stick and the ESP32 (native USB-C port), then re-run."; exit 1; }
for n in lidar esp32; do
  if [[ -e /dev/$n ]]; then ok "/dev/$n → $(readlink -f /dev/$n)"; else warn "/dev/$n missing (udev rule didn't match)"; fi
done
if [[ -e /dev/lidar && -e /dev/esp32 && "$(readlink -f /dev/lidar)" == "$(readlink -f /dev/esp32)" ]]; then
  err "/dev/lidar and /dev/esp32 are the same port"
fi
CP=()
for d in "${PORTS[@]}"; do [[ "${KIND[$d]}" == 10c4:ea60 ]] && CP+=("$d"); done
if (( ${#CP[@]} > 1 )); then
  warn "Two CP2102s: the udev rule can't tell them apart by type. Pin them by serial number:"
  for d in "${CP[@]}"; do info "  $d serial ${SERIAL[$d]:-<none — then use ATTRS{devpath} / the USB port>}"; done
  info "  edit /etc/udev/rules.d/99-campus-bot.rules: add ATTRS{serial}==\"<serial>\" to the lidar line,"
  info "  and a second line with the ESP32's serial → SYMLINK+=\"esp32\"; then: sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

# ------------------------------------------------------------------------------------------------
LIDAR_OK=""
if (( DO_LIDAR )); then
  say "2/5 Lidar alone (sllidar_ros2 @ $LIDAR_BAUD)"
  # Candidates: /dev/lidar, then every CP2102, then everything.
  CANDS=()
  [[ -e /dev/lidar ]] && CANDS+=("$(readlink -f /dev/lidar)")
  CANDS+=("${CP[@]}" "${PORTS[@]}")
  declare -A TRIED=()
  for dev in "${CANDS[@]}"; do
    [[ -n "${TRIED[$dev]:-}" ]] && continue; TRIED[$dev]=1
    # Never open the ESP32's native port with the lidar driver: sllidar drops DTR on open, which
    # with RTS up holds the S3 in reset, and the port keeps that state after close (-hupcl).
    if [[ "${KIND[$dev]:-}" == 303a:* || "$dev" == "$(readlink -f /dev/esp32 2>/dev/null)" ]]; then continue; fi
    info "trying $dev …"
    start_bg hw_lidar ros2 run sllidar_ros2 sllidar_node --ros-args \
      -p serial_port:="$dev" -p serial_baudrate:="$LIDAR_BAUD" -p frame_id:=laser -p angle_compensate:=true
    if wait_topic "$SCAN_TOPIC" 12; then
      hz="$(timeout 6 ros2 topic hz "$SCAN_TOPIC" 2>/dev/null | sed -n 's/.*average rate: \([0-9.]*\).*/\1/p' | tail -1)"
      ok "lidar on $dev, scanning at ${hz:-?} Hz"
      grep -hE 'SLLIDAR S/N|Firmware Ver|Hardware Rev|current scan mode|sample rate|scan frequency' "$LOG_DIR/hw_lidar.log" | tail -6 | sed 's/^/      /'
      LIDAR_OK="$dev"
      stop_bg hw_lidar >/dev/null
      break
    fi
    stop_bg hw_lidar >/dev/null
    grep -hiE 'error|fail|timeout|cannot' "$LOG_DIR/hw_lidar.log" | tail -2 | sed 's/^/      /'
  done
  if [[ -z "$LIDAR_OK" ]]; then
    err "No scans on any port. Check: motor spinning? Red→5V, Black→GND, Yellow(lidar TX)→RXD, Green(lidar RX)→TXD."
    err "'operation timeout' usually means TX/RX swapped or no 5 V. Log: $LOG_DIR/hw_lidar.log"
  elif [[ ! -e /dev/lidar ]]; then
    SUGGEST[LIDAR_PORT]="$LIDAR_OK"
  fi
fi

# ------------------------------------------------------------------------------------------------
ESP_PORT="" ESP_BAUD=""
if (( DO_ESP )); then
  say "3/5 ESP32 micro-ROS link"
  lt="$(readlink -f "${LIDAR_OK:-/dev/lidar}" 2>/dev/null || true)"
  CANDS=()
  [[ -e /dev/esp32 ]] && CANDS+=("$(readlink -f /dev/esp32)")
  for d in "${PORTS[@]}"; do [[ "$d" != "$lt" ]] && CANDS+=("$d"); done
  declare -A SEEN=()
  # AGENT_BAUD from robot.env goes first.
  ORDER="$AGENT_BAUD $(tr ' ' '\n' <<<"$BAUDS" | grep -vx "$AGENT_BAUD" | tr '\n' ' ')"
  # hw_wait SECS: 2 = entities created (topics exist), 1 = session but no entities yet, 0 = nothing
  hw_wait() {
    local end=$((SECONDS + $1))
    while (( SECONDS < end )); do
      grep -qE 'create_datawriter|create_datareader' "$LOG_DIR/hw_agent.log" && return 2
      sleep 0.5
    done
    grep -q 'session established' "$LOG_DIR/hw_agent.log" && return 1
    return 0
  }
  for dev in "${CANDS[@]}"; do
    [[ -n "${SEEN[$dev]:-}" ]] && continue; SEEN[$dev]=1
    bauds="$ORDER"
    # Native USB (Espressif 303a): the baud rate is ignored, so one try settles it. Sweeping bauds
    # here only produced a lucky "921600" on 2026-10-01 and a wrong AGENT_BAUD suggestion.
    [[ "${KIND[$dev]:-}" == 303a:* ]] && bauds="$AGENT_BAUD"
    for baud in $bauds; do
      printf '    %s @ %-8s' "$dev" "$baud"
      : >"$LOG_DIR/hw_agent.log"
      start_bg hw_agent ros2 run micro_ros_agent micro_ros_agent serial --dev "$dev" -b "$baud" -v4
      # An idle firmware connects to a new agent in < 1 s (measured), so only reset if it doesn't.
      # tools/esp32_reset.py never arms the S3's download mode (a plain RTS pulse with DTR up can
      # leave it in the ROM loader). After a reset the firmware needs ~3 s to boot, and creating its
      # topics can take another ~15 s when the agent is slow, so wait for the topics, not the session.
      hw_wait 4; r=$?
      if (( r < 2 )); then
        printf 'resetting… '
        esp32_reset "$dev" >/dev/null || true
        hw_wait 25; r=$?
      fi
      if (( r == 2 )); then
        echo "✓ connected, topics created"
        ESP_PORT="$dev"; ESP_BAUD="$baud"
        break 2
      fi
      (( r == 1 )) && echo "session but no topics after 25 s (agent too slow? see $LOG_DIR/hw_agent.log)" || echo "–"
      stop_bg hw_agent >/dev/null
    done
  done
  if [[ -z "$ESP_PORT" ]]; then
    err "The ESP32 never connected. Possibilities:"
    err "  • the cable is in the board's UART USB-C port: micro-ROS runs on the NATIVE port (lsusb: 303a:1001)"
    err "  • it's stuck in the ROM loader: python3 tools/esp32_reset.py   (prints the boot mode)"
    err "  • still nothing: unplug the ESP32's USB for 3 s, plug it back in, re-run"
  else
    if [[ "${KIND[$ESP_PORT]:-}" == 303a:* ]]; then ok "ESP32 micro-ROS on $ESP_PORT (native USB: the baud setting doesn't matter)"
    else ok "ESP32 micro-ROS on $ESP_PORT at $ESP_BAUD baud"
         [[ "$ESP_BAUD" != "$AGENT_BAUD" ]] && SUGGEST[AGENT_BAUD]="$ESP_BAUD"; fi
    if [[ ! -e /dev/esp32 || "$(readlink -f /dev/esp32)" != "$ESP_PORT" ]]; then SUGGEST[ESP32_PORT]="$ESP_PORT"; fi

    # --------------------------------------------------------------------------------------------
    say "4/5 What the ESP32 publishes and subscribes"
    # DDS discovery of the agent's topics takes a few seconds: poll instead of a fixed sleep.
    for _ in $(seq 15); do
      ros2 topic list -t 2>/dev/null | grep 'nav_msgs/msg/Odometry' >/dev/null && break
      sleep 1
    done
    info "nodes: $(ros2 node list 2>/dev/null | tr '\n' ' ')"
    TOPICS="$(ros2 topic list -t 2>/dev/null | grep -vE '^/(parameter_events|rosout) ')"
    echo "$TOPICS" | sed 's/^/      /'
    pick() { grep -F "[$1]" <<<"$TOPICS" | head -1 | cut -d' ' -f1; }
    odom="$(pick nav_msgs/msg/Odometry)"; imu="$(pick sensor_msgs/msg/Imu)"
    cmd="$(pick geometry_msgs/msg/Twist)"; cmds="$(pick geometry_msgs/msg/TwistStamped)"
    if [[ -n "$odom" ]]; then ok "odometry: $odom"; [[ "$odom" != "$ODOM_TOPIC" ]] && SUGGEST[ODOM_TOPIC]="$odom"
    else warn "no Odometry topic"; fi
    if [[ -n "$imu" ]]; then ok "IMU: $imu"; [[ "$imu" != "$IMU_TOPIC" ]] && SUGGEST[IMU_TOPIC]="$imu"
    else warn "no Imu topic (heading may be fused inside the firmware only)"; fi
    if [[ -n "$cmd" ]]; then ok "velocity commands: $cmd (Twist)"; [[ "$cmd" != "$CMD_VEL_TOPIC" ]] && SUGGEST[CMD_VEL_TOPIC]="$cmd"
    elif [[ -n "$cmds" ]]; then
      err "velocity commands are TwistStamped on $cmds: teleop_keys.py sends Twist. Tell Claude to switch it."
    else warn "no velocity-command topic seen"; fi
    for t in $odom $imu; do
      hz="$(timeout 6 ros2 topic hz "$t" 2>/dev/null | sed -n 's/.*average rate: \([0-9.]*\).*/\1/p' | tail -1)"
      info "$t: ${hz:-no messages} Hz"
    done
    # Compare the odom stamp with the Pi's clock (a firmware without time sync stamps in 1970).
    osec="$(timeout 4 ros2 topic echo --once --field header.stamp.sec "$odom" 2>/dev/null | head -1 | tr -dc '0-9')"
    if [[ -n "$osec" ]] && (( osec < $(date +%s) - 5 || osec > $(date +%s) + 5 )); then
      warn "odom stamps are $(( $(date +%s) - osec )) s off the Pi's clock (no time sync in the firmware?). odom_to_tf.py restamps them."
    elif [[ -n "$osec" ]]; then info "odom stamps match the Pi's clock"
    fi
    info "odom frames: $(timeout 4 ros2 topic echo --once --no-arr "$odom" 2>/dev/null | grep -E 'frame_id' | tr -s ' ' | tr '\n' ' ')"
    if python3 tools/pose_tool.py probe odom base_link --timeout 3; then info "the firmware broadcasts odom → base_link TF itself"
    else info "no odom → base_link TF from the firmware (remap.sh will start odom_to_tf.py)"; fi
    stop_bg hw_agent >/dev/null
  fi
fi

# ------------------------------------------------------------------------------------------------
say "5/5 config/robot.env"
if ((${#SUGGEST[@]} == 0)); then
  ok "nothing to change"
else
  for k in "${!SUGGEST[@]}"; do info "$k=${SUGGEST[$k]}   (now: $(grep -E "^$k=" config/robot.env | cut -d= -f2 | awk '{print $1}'))"; done
  if (( APPLY )); then
    cp config/robot.env "config/robot.env.bak.$(date +%H%M%S)"
    for k in "${!SUGGEST[@]}"; do
      sed -i -E "s|^($k=)[^ #]*|\1${SUGGEST[$k]}|" config/robot.env
    done
    ok "written to config/robot.env (backup alongside)"
  else
    info "re-run with --apply to write these, or edit config/robot.env by hand"
  fi
fi
bash run/stop.sh >/dev/null 2>&1 || true
info "Report saved: $REPORT"
info "Next: bash setup/04_backup_esp32.sh (if not done), then run/remap.sh test"
