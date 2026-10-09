#!/usr/bin/env bash
# Written 2026-10-09: what the robot runs by itself at boot (systemd user service campus-bot.service,
# installed by setup/07_autostart.sh). Dashboard first, then navigation on the last map picked on the
# dashboard; the pose comes back from nodes/pose_keeper.py's file if the lidar agrees.
#   systemctl --user status campus-bot      journalctl --user -u campus-bot
#   systemctl --user stop campus-bot        stops everything (run/stop.sh)
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR" || exit 1

# USB serial devices can enumerate after the user manager starts
for dev in /dev/lidar /dev/esp32; do
  for _ in $(seq 60); do [[ -e "$dev" ]] && break; sleep 1; done
  [[ -e "$dev" ]] && ok "$dev present" || warn "$dev missing after 60 s: navigation will report it"
done

run/dashboard.sh on
# the robot's voice (nodes/buzzer.py); like the dashboard it outlives navigation restarts
if [[ "${BUZZER:-1}" == 1 ]] && ! is_running buzzer; then
  start_bg buzzer env BUZZER_MOVING="${BUZZER_MOVING:-1}" python3 nodes/buzzer.py
fi

AREA="$(cat "$RUN_DIR/dash_map" 2>/dev/null)"
if [[ -z "$AREA" || ! -f "maps/${AREA}_clean.yaml" ]]; then
  warn "no saved map selected on the dashboard: navigation not started (pick one and NAVIGATE)"
else
  say "navigation on $AREA"
  # same log as the dashboard's NAVIGATE, so its job line and log tabs show this start too
  run/nav.sh "$AREA" --yes --no-rviz >"$LOG_DIR/nav_run.log" 2>&1 < /dev/null \
    && ok "navigation up on $AREA (log: $LOG_DIR/nav_run.log)" \
    || warn "navigation start reported a problem: see $LOG_DIR/nav_run.log"
fi

# Stay alive: the stack's processes live in this service; stopping the service stops them.
exec sleep infinity
