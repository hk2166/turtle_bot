# RECONSTRUCTED (2026-09-30): shared helpers for run/*.sh. Sourced, not run.
# Background processes are started in their own process group (setsid) with their PID in
# ~/.campus_bot/run/<name>.pid and their output in ~/.campus_bot/logs/<name>.log, so stop.sh can
# always find and stop them, even after the terminal that started them is gone.

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="$HOME/.campus_bot/run"
LOG_DIR="$HOME/.campus_bot/logs"
mkdir -p "$RUN_DIR" "$LOG_DIR" "$PROJECT_DIR/maps"

# ROS setup files read unset variables: never source them under `set -u`.
_had_u=0; [[ $- == *u* ]] && _had_u=1; set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
[[ -f "$HOME/microros_ws/install/local_setup.bash" ]] && source "$HOME/microros_ws/install/local_setup.bash"
(( _had_u )) && set -u
# shellcheck disable=SC1091
source "$PROJECT_DIR/config/robot.env"

# Ctrl-C ends the script (its EXIT trap still runs). `timeout` puts its command in a process group of
# its own, so without --foreground (below) and this trap, Ctrl-C during a wait was silently ignored.
on_int() { echo; err "interrupted"; exit 130; }
trap on_int INT

say()  { printf '\033[1;34m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '    \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '    \033[33m! %s\033[0m\n' "$*"; }
err()  { printf '    \033[31m✗ %s\033[0m\n' "$*" >&2; }

BOOT_ID_FILE=/proc/sys/kernel/random/boot_id

is_running() {  # is_running NAME
  local pid f="$RUN_DIR/$1.pid" b="$RUN_DIR/$1.boot" boot
  pid="$(cat "$f" 2>/dev/null)" || return 1
  # a pid file from before the last reboot is stale even if some other process now has that PID.
  # Compare the kernel boot id, not times: the Pi has no battery clock, so its wall clock JUMPS at boot
  # when NTP syncs, and files written just before the jump looked "older than the boot" — which made
  # remap.sh start a second dashboard and face.sh think the face service was missing (2026-10-26).
  if [[ -f "$b" ]]; then
    [[ "$(cat "$b" 2>/dev/null)" == "$(cat "$BOOT_ID_FILE")" ]] || return 1
  else                                            # older pid file without a boot id
    boot=$(( $(date +%s) - $(cut -d. -f1 /proc/uptime) ))
    (( $(stat -c %Y "$f") >= boot )) || return 1
  fi
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

start_bg() {  # start_bg NAME CMD...   (log: $LOG_DIR/NAME.log)
  local name="$1"; shift
  if is_running "$name"; then info "$name already running"; return 0; fi
  printf '\n===== %s  %s\n' "$(date '+%F %T')" "$*" >>"$LOG_DIR/$name.log"
  rm -f "$RUN_DIR/$name.pid"
  cat "$BOOT_ID_FILE" >"$RUN_DIR/$name.boot"     # which boot this pid belongs to (see is_running)
  # setsid may fork (then $! is the wrong PID), so the child records its own PID, which is also
  # its process-group ID, before exec'ing the command.
  setsid bash -c 'echo $$ >"$0"; exec "$@"' "$RUN_DIR/$name.pid" "$@" >>"$LOG_DIR/$name.log" 2>&1 < /dev/null &
  local i
  for i in $(seq 20); do [[ -s "$RUN_DIR/$name.pid" ]] && break; sleep 0.05; done
}

stop_bg() {  # stop_bg NAME : SIGINT the whole process group, then TERM, then KILL
  local name="$1" pid i
  pid="$(cat "$RUN_DIR/$name.pid" 2>/dev/null)" || return 0
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for i in $(seq 50); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    kill -TERM -- "-$pid" 2>/dev/null || true
    for i in $(seq 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    kill -KILL -- "-$pid" 2>/dev/null || true
    info "stopped $name"
  fi
  rm -f "$RUN_DIR/$name.pid"
}

wait_topic() {  # wait_topic TOPIC SECONDS : true once one message arrives
  local topic="$1" secs="$2" end=$((SECONDS + $2))
  while (( SECONDS < end )); do
    timeout --foreground 5 ros2 topic echo --once --no-arr "$topic" >/dev/null 2>&1 && return 0
    sleep 0.5
  done
  return 1
}

esp32_reset() {  # esp32_reset [DEV] : safe reset through the S3's native USB (see tools/esp32_reset.py)
  local out rc=0
  out="$(python3 "$PROJECT_DIR/tools/esp32_reset.py" "${1:-${ESP32_DEV:-/dev/esp32}}" 2>&1)" || rc=$?
  sed 's/^/    /' <<<"$out"
  return "$rc"
}

agent_session() {  # agent_session LOG FROM_LINE SECS : true once the agent logs a client session after FROM_LINE
  local log="$1" from="$2" end=$((SECONDS + $3))
  while (( SECONDS < end )); do
    # not grep -q: under pipefail an early grep exit can SIGPIPE tail and turn a match into a failure
    tail -n +"$from" "$log" 2>/dev/null | grep 'session established' >/dev/null && return 0
    sleep 0.5
  done
  return 1
}

port_busy() {  # port_busy DEV : true, and says who, if another program already has DEV open
  local dev pids
  dev="$(readlink -f "$1" 2>/dev/null)" || return 1
  [[ -e "$dev" ]] || return 1
  pids="$(fuser "$dev" 2>/dev/null | tr -s ' ' ',' | sed 's/^,//; s/,$//')"
  [[ -n "$pids" ]] || return 1
  err "$1 is already open in another program (two programs on one serial port split its data):"
  ps -o pid=,args= -p "$pids" 2>/dev/null | cut -c1-100 | sed 's/^/        /' >&2
  err "close that program first, or stop everything with run/stop.sh"
  return 0
}

use_robot_screen() {  # over SSH (no DISPLAY): show windows on the robot's own desktop (monitor / VNC) if one is up
  [[ -n "${DISPLAY:-}" ]] && return 0
  local xa
  [[ -S /tmp/.X11-unix/X0 ]] || return 1
  for xa in "/run/user/$(id -u)/gdm/Xauthority" /run/user/"$(id -u)"/.mutter-Xwaylandauth.*; do
    if [[ -r "$xa" ]]; then
      export DISPLAY=:0 XAUTHORITY="$xa"
      info "windows (RViz) open on the robot's own screen: the monitor, or VNC"
      return 0
    fi
  done
  return 1
}

resolve_ports() {  # sets LIDAR_DEV and ESP32_DEV from LIDAR_PORT / ESP32_PORT (auto = udev names first)
  local c lt et
  if [[ "$ESP32_PORT" != auto ]]; then ESP32_DEV="$ESP32_PORT"
  elif [[ -e /dev/esp32 ]]; then ESP32_DEV=/dev/esp32
  else ESP32_DEV=""; fi
  if [[ "$LIDAR_PORT" != auto ]]; then LIDAR_DEV="$LIDAR_PORT"
  elif [[ -e /dev/lidar ]]; then LIDAR_DEV=/dev/lidar
  else
    LIDAR_DEV=/dev/ttyUSB0
    et="$(readlink -f "${ESP32_DEV:-/nonexistent}" 2>/dev/null || true)"
    for c in /dev/ttyUSB*; do [[ -e "$c" && "$(readlink -f "$c")" != "$et" ]] && { LIDAR_DEV="$c"; break; }; done
  fi
  lt="$(readlink -f "$LIDAR_DEV" 2>/dev/null || echo "$LIDAR_DEV")"
  if [[ -z "$ESP32_DEV" ]]; then
    for c in /dev/ttyACM* /dev/ttyUSB*; do
      [[ -e "$c" && "$(readlink -f "$c")" != "$lt" ]] && { ESP32_DEV="$c"; break; }
    done
  fi
  ESP32_DEV="${ESP32_DEV:-/dev/ttyACM0}"
  if [[ "$(readlink -f "$ESP32_DEV" 2>/dev/null)" == "$lt" ]]; then
    err "The lidar and the ESP32 resolve to the same port ($lt). Both USB bridges are probably CP2102:"
    err "pin each by serial number in /etc/udev/rules.d/99-campus-bot.rules, or set LIDAR_PORT/ESP32_PORT in config/robot.env."
    return 1
  fi
}
