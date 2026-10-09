#!/usr/bin/env bash
# Written 2026-10-09: start the robot by itself at boot, no monitor, no login.
#   setup/07_autostart.sh            install + enable (no sudo needed for this part)
#   setup/07_autostart.sh --undo     disable + remove
# Installs two systemd USER services (setup/systemd/):
#   campus-bot.service     dashboard + navigation on the last map (run/autostart.sh)
#   claude-remote.service  `claude remote-control` in this folder, restarted if it ever stops
# One-time steps only you can do (printed below if still missing):
#   sudo loginctl enable-linger resp        user services start at boot without anyone logging in
#   cd ~/Desktop/campus_bot_fw && claude    accept the folder-trust question once, then /exit
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS=(campus-bot.service claude-remote.service)
DEST="$HOME/.config/systemd/user"

if [[ "${1:-}" == --undo ]]; then
  for u in "${UNITS[@]}"; do systemctl --user disable --now "$u" 2>/dev/null || true; rm -f "$DEST/$u"; done
  systemctl --user daemon-reload
  echo "autostart removed (linger left as is: sudo loginctl disable-linger $USER to undo that)"
  exit 0
fi

mkdir -p "$DEST"
for u in "${UNITS[@]}"; do install -m 644 "$HERE/systemd/$u" "$DEST/$u"; done
systemctl --user daemon-reload
systemctl --user enable "${UNITS[@]}"
echo "enabled: ${UNITS[*]}"

[[ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" == yes ]] \
  || echo "TODO (once): sudo loginctl enable-linger $USER     <- without it nothing starts until someone logs in"
echo "start now without rebooting:  systemctl --user start ${UNITS[*]}"
