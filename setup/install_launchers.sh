#!/usr/bin/env bash
# Written 2026-09-30: double-click launchers for the Pi's desktop (handy over xrdp), plus the same
# entries in the app grid (search "Campus Bot"). Safe to re-run.
set -euo pipefail
P="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APPS="$HOME/.local/share/applications"
mkdir -p "$APPS"

make() {  # make FILE "Name" "Comment" icon ACTION   (ACTION: see run/desktop_action.sh)
  local f="$1" name="$2" comment="$3" icon="$4" cmd="$5"
  cat >"$APPS/$f" <<DESK
[Desktop Entry]
Type=Application
Name=$name
Comment=$comment
Icon=$icon
Terminal=false
Exec=gnome-terminal --title="$name" -- $P/run/desktop_action.sh $cmd
Categories=Utility;
DESK
  chmod +x "$APPS/$f"
  cp "$APPS/$f" "$HOME/Desktop/$f"
  chmod +x "$HOME/Desktop/$f"
  gio set "$HOME/Desktop/$f" metadata::trusted true 2>/dev/null || true
}

make campusbot-1-hwcheck.desktop "Campus Bot 1 - Hardware check" "Find the lidar and ESP32, test each, suggest robot.env" \
  utilities-system-monitor hwcheck
make campusbot-2-map.desktop "Campus Bot 2 - Map an area" "Pre-flight, SLAM, keyboard driving; q saves" \
  applications-science map
make campusbot-5-drive.desktop "Campus Bot 3 - Drive (hold keys)" "Hold arrow keys / WASD to drive; let go to stop" \
  input-keyboard drive
make campusbot-6-lidar.desktop "Campus Bot - Lidar view" "Live lidar in RViz plus numbers" \
  applications-science lidar
make campusbot-3-stop.desktop "Campus Bot - STOP" "Zero velocity and stop everything" \
  process-stop stop
make campusbot-4-power.desktop "Campus Bot - Power watch" "Watch the Pi 5 V rail for dips" \
  battery power
echo "Launchers installed on the Desktop and in the app grid."
echo "If the desktop icons show a red X or open as text: right-click → 'Allow Launching'."
