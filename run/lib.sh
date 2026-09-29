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

say()  { printf '\033[1;34m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '    \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '    \033[33m! %s\033[0m\n' "$*"; }
err()  { printf '    \033[31m✗ %s\033[0m\n' "$*" >&2; }

is_running() {  # is_running NAME
  local pid
  pid="$(cat "$RUN_DIR/$1.pid" 2>/dev/null)" || return 1
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

start_bg() {  # start_bg NAME CMD...   (log: $LOG_DIR/NAME.log)
  local name="$1"; shift
  if is_running "$name"; then info "$name already running"; return 0; fi
  printf '\n===== %s  %s\n' "$(date '+%F %T')" "$*" >>"$LOG_DIR/$name.log"
  rm -f "$RUN_DIR/$name.pid"
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
    timeout 5 ros2 topic echo --once --no-arr "$topic" >/dev/null 2>&1 && return 0
    sleep 0.5
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
