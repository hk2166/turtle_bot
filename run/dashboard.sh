#!/usr/bin/env bash
# Written 2026-10-06: the web mission-control panel (nodes/dashboard.py).
#   run/dashboard.sh [on|off|status]     default on → http://resp-desktop.local:8088
# Needs the firewall opened once:  sudo ufw allow 8088/tcp
# Anyone on the office WiFi who can reach the page can DRIVE the robot: it has no password.
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
case "${1:-on}" in
  on)
    is_running dashboard && { ok "dashboard already on"; exit 0; }
    start_bg dashboard python3 "$PROJECT_DIR/nodes/dashboard.py"
    sleep 3
    if is_running dashboard; then
      ok "dashboard: http://$(hostname).local:8088  (http://$(hostname -I | awk '{print $1}'):8088)"
      info "if it doesn't load from the Mac: sudo ufw allow 8088/tcp"
    else err "didn't start: see $LOG_DIR/dashboard.log"; tail -5 "$LOG_DIR/dashboard.log" >&2; exit 1; fi ;;
  off) stop_bg dashboard; ok "dashboard off" ;;
  status) is_running dashboard && echo "dashboard: on" || echo "dashboard: off" ;;
  *) echo "usage: $0 [on|off|status]"; exit 2 ;;
esac
