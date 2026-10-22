#!/usr/bin/env bash
# Written 2026-10-09: what the robot runs by itself at boot (systemd user service campus-bot.service,
# installed by setup/07_autostart.sh). Dashboard first, then navigation on the last map picked on the
# dashboard; the pose comes back from nodes/pose_keeper.py's file if the lidar agrees.
#   systemctl --user status campus-bot      journalctl --user -u campus-bot
#   systemctl --user stop campus-bot        stops everything (run/stop.sh)
# The whole script is one { ... } block: bash parses it completely before running anything, so
# editing this file while the service runs can't break it (it did on 2026-10-10: bash kept reading
# the changed file from its old offset after nav.sh returned, hit a syntax error, and the service
# stopped the whole stack).
{
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
  start_bg buzzer env BUZZER_MOVING="${BUZZER_MOVING:-1}" BUZZER_TYPE="${BUZZER_TYPE:-active}" BUZZER_LINES="${BUZZER_LINES:-front}" \
    python3 nodes/buzzer.py
fi
# face recognition (face/face_service.py): delivery identify/log + the office self-enrolment page on
# https://<robot>.local:8443/enroll. Optional: the robot runs fine without it (FACE=0 to skip).
if [[ "${FACE:-1}" == 1 ]]; then
  run/face.sh on || warn "face service didn't start (see $LOG_DIR/face.log): deliveries still work"
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
}
