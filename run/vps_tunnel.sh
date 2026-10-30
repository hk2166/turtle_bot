#!/usr/bin/env bash
# Written 2026-10-30: the dashboard on the internet, through the VPS.
#   run/vps_tunnel.sh [on|off|status|run]
# An SSH reverse tunnel carries the dashboard's internet-facing copy (127.0.0.1:8090 on the Pi, where
# EVERYTHING needs the admin login; see REMOTE_PORT in nodes/dashboard.py) to 127.0.0.1:$VPS_TUNNEL_PORT
# on the VPS, and Caddy there serves it as https://<vps>/ with a real certificate.
# Never point the tunnel at 8088/8444: visitors arrive from 127.0.0.1 and would be trusted as the robot.
#
# Settings (NOT in the repo): ~/.campus_bot/vps.env
#   VPS_HOST=1.2.3.4   VPS_USER=root   VPS_SSH_PORT=22   VPS_TUNNEL_PORT=18090
# Key: ~/.ssh/campus_bot_vps (its .pub must be in the VPS user's ~/.ssh/authorized_keys).
# The service (campus-bot-tunnel.service, systemd --user) keeps it up across WiFi drops and reboots.
set -uo pipefail
ENV="$HOME/.campus_bot/vps.env"
KEY="$HOME/.ssh/campus_bot_vps"
UNIT=campus-bot-tunnel.service

load() {
  [[ -f "$ENV" ]] || { echo "no $ENV yet (VPS_HOST, VPS_USER)"; exit 1; }
  # shellcheck disable=SC1090
  source "$ENV"
  : "${VPS_HOST:?set VPS_HOST in $ENV}" "${VPS_USER:?set VPS_USER in $ENV}"
  VPS_SSH_PORT="${VPS_SSH_PORT:-22}"; VPS_TUNNEL_PORT="${VPS_TUNNEL_PORT:-18090}"
}

case "${1:-on}" in
  run)    # what the service runs: one ssh, the service restarts it when it drops
    load
    exec ssh -N -i "$KEY" -p "$VPS_SSH_PORT" \
      -o BatchMode=yes -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new \
      -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes \
      -R "127.0.0.1:$VPS_TUNNEL_PORT:127.0.0.1:${DASH_REMOTE_PORT:-8090}" "$VPS_USER@$VPS_HOST" ;;
  on)
    load
    mkdir -p "$HOME/.config/systemd/user"
    cat > "$HOME/.config/systemd/user/$UNIT" <<EOF
[Unit]
Description=Campus Bot dashboard tunnel to the VPS (run/vps_tunnel.sh)
After=network-online.target

[Service]
ExecStart=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/vps_tunnel.sh run
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
EOF
    systemctl --user daemon-reload
    systemctl --user enable --now "$UNIT" && echo "tunnel on: ${VPS_URL:-$VPS_HOST}" ;;
  off)    systemctl --user disable --now "$UNIT" && echo "tunnel off" ;;
  status) systemctl --user --no-pager status "$UNIT" | head -5 ;;
  *)      echo "usage: $0 [on|off|status]"; exit 2 ;;
esac
