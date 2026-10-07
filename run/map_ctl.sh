#!/usr/bin/env bash
# Written 2026-10-07: mapping control for the web dashboard (and handy from a terminal).
#
#   run/map_ctl.sh start <name>    stop navigation (dashboard stays), bring sensors + SLAM up for a
#                                  NEW map <name>; drive from the dashboard's MANUAL tab
#   run/map_ctl.sh save <name>     save maps/<name> (+ _clean, posegraph, report), then stop SLAM
#   run/map_ctl.sh discard         stop SLAM without saving
set -Eeuo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
cd "$PROJECT_DIR"

case "${1:-}" in
  start)
    AREA="${2:-}"
    [[ "$AREA" =~ ^[A-Za-z0-9_-]+$ ]] || { err "bad map name '$AREA' (letters, digits, _ and -)"; exit 2; }
    run/stop.sh --keep-dashboard
    exec run/remap.sh "$AREA" --no-camera --no-rviz --yes --prepare-only
    ;;
  save)
    AREA="${2:-}"
    [[ "$AREA" =~ ^[A-Za-z0-9_-]+$ ]] || { err "bad map name '$AREA'"; exit 2; }
    is_running slam || { err "SLAM isn't running: nothing to save"; exit 1; }
    MAP="$PROJECT_DIR/maps/$AREA"
    say "Saving maps/$AREA"
    if [[ -f "$MAP.yaml" ]]; then
      PREV="$PROJECT_DIR/maps/.prev/$AREA-$(date +%Y%m%d-%H%M%S)"
      mkdir -p "$PREV"
      mv "$MAP".* "$MAP"_clean.* "$MAP"_report.txt "$MAP"_tags.yaml "$PREV"/ 2>/dev/null || true
      info "previous '$AREA' map moved to maps/.prev/$(basename "$PREV")"
    fi
    ros2 run nav2_map_server map_saver_cli -f "$MAP" --ros-args -p save_map_timeout:=10.0 \
      >"$LOG_DIR/map_saver.log" 2>&1 || { err "map_saver failed: see $LOG_DIR/map_saver.log"; exit 1; }
    ok "maps/$AREA.yaml + .pgm"
    if timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
         "{filename: '$MAP'}" >/dev/null 2>&1 && [[ -f "$MAP.posegraph" ]]; then
      ok "maps/$AREA.posegraph (extend it later with --continue)"
    else warn "pose graph not saved (optional)"; fi
    python3 tools/clean_map.py "$MAP.yaml" | sed 's/^/    /'
    ok "maps/${AREA}_clean.yaml ready to navigate"
    stop_bg slam
    ;;
  discard)
    stop_bg slam
    ok "SLAM stopped, nothing saved (sensors stay up)"
    ;;
  *) sed -n '2,9p' "$0"; exit 2 ;;
esac
