#!/usr/bin/env bash
# Written 2026-10-01: navigate on a map that's already saved (no re-mapping).
#
#   run/nav.sh <area> [--no-camera] [--no-rviz] [--yes]
#
# Same bring-up and pre-flight as run/remap.sh, then AMCL + Nav2 on maps/<area>_clean.yaml. With
# maps/<area>_tags.yaml the camera finds the robot's pose by itself as soon as it sees a tag pair;
# otherwise set it in RViz (2D Pose Estimate). Send goals with "Nav2 Goal". Stop: run/stop.sh
exec "$(dirname "${BASH_SOURCE[0]}")/remap.sh" "$@" --nav-only
