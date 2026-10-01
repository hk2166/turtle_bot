#!/usr/bin/env bash
# Written 2026-09-30: what the desktop launchers (setup/install_launchers.sh) run inside a terminal.
#   run/desktop_action.sh hwcheck|map|drive|lidar|stop|power
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
case "${1:-}" in
  hwcheck)
    # Ask first, so the check (and its ESP32 reset) runs once, not twice.
    read -r -p "If it suggests new config/robot.env values, write them automatically? [y/N] " a
    if [[ "$a" =~ ^[Yy] ]]; then run/hw_check.sh --apply; else run/hw_check.sh; fi ;;
  map)
    read -r -p "Area name [office]: " a
    run/remap.sh "${a:-office}" ;;
  drive) run/drive.sh ;;
  lidar) run/lidar_view.sh ;;
  stop)  run/stop.sh ;;
  power) run/power_check.sh --watch ;;
  *) echo "usage: $0 hwcheck|map|drive|lidar|stop|power"; exit 2 ;;
esac
echo
read -r -p "Press Enter to close this window " _
