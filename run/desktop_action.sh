#!/usr/bin/env bash
# Written 2026-09-30: what the desktop launchers (setup/install_launchers.sh) run inside a terminal.
#   run/desktop_action.sh hwcheck|map|stop|power
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
case "${1:-}" in
  hwcheck)
    run/hw_check.sh
    echo
    read -r -p "Write the suggested values into config/robot.env? [y/N] " a
    [[ "$a" =~ ^[Yy] ]] && run/hw_check.sh --apply --skip-lidar ;;
  map)
    read -r -p "Area name [office]: " a
    run/remap.sh "${a:-office}" ;;
  stop)  run/stop.sh ;;
  power) run/power_check.sh --watch ;;
  *) echo "usage: $0 hwcheck|map|stop|power"; exit 2 ;;
esac
echo
read -r -p "Press Enter to close this window " _
