#!/usr/bin/env bash
# Written 2026-10-22: pin the SECOND camera (the HIKVISION DS-U02 for face recognition) to a stable
# /dev/face_cam, so it never gets the Brio's /dev/video0 and the face service always finds it.
#
#   sudo bash setup/10_face_cam.sh            # run AFTER plugging the HIKVISION in
#
# The Brio 100 (Logitech, idVendor 046d) stays on /dev/video0 for AprilTags + the EYES preview; this
# rule gives the other USB camera the name /dev/face_cam. Safe to re-run.
set -Eeuo pipefail
[[ $EUID -eq 0 ]] || { echo "Run it with sudo:  sudo bash $0"; exit 1; }
RULES=/etc/udev/rules.d/99-campus-face-cam.rules
log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '    \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '    \033[33m! %s\033[0m\n' "$*"; }

log "Finding the non-Brio USB camera"
# capture-capable /dev/video* whose vendor is NOT Logitech (046d = the Brio)
cand=""
for dev in /dev/video*; do
  [[ -e "$dev" ]] || continue
  v4l2-ctl -d "$dev" --all 2>/dev/null | grep -q 'Video Capture' || continue
  vid="$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p')"
  [[ "$vid" == "046d" || -z "$vid" ]] && continue      # skip the Brio and non-USB nodes
  cand="$dev"; break
done
[[ -n "$cand" ]] || { warn "no second USB camera found. Plug the HIKVISION into a USB port and re-run."; \
  info "attached cameras:"; v4l2-ctl --list-devices 2>/dev/null | sed 's/^/        /'; exit 1; }

VID="$(udevadm info -q property -n "$cand" | sed -n 's/^ID_VENDOR_ID=//p')"
PID="$(udevadm info -q property -n "$cand" | sed -n 's/^ID_MODEL_ID=//p')"
SER="$(udevadm info -q property -n "$cand" | sed -n 's/^ID_SERIAL_SHORT=//p')"
MODEL="$(udevadm info -q property -n "$cand" | sed -n 's/^ID_MODEL=//p')"
info "picked $cand  →  vendor=$VID product=$PID serial=${SER:-<none>} model=${MODEL:-?}"

log "Writing $RULES"
# Match the capture node of this camera. Prefer the serial when the camera exposes one (survives a
# different USB port); fall back to vendor:product. ENV{ID_V4L_CAPABILITIES} ':capture:' picks the
# real capture node, not the metadata node some UVC cameras also expose.
if [[ -n "$SER" ]]; then
  MATCH="ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", ATTRS{serial}==\"$SER\""
else
  MATCH="ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\""
  warn "this camera reports no serial — the rule matches vendor:product only (fine unless you add a 2nd identical one)"
fi
cat > "$RULES" <<EOF
# Campus Bot: the HIKVISION face camera -> /dev/face_cam (the Brio, 046d, stays on /dev/video0).
SUBSYSTEM=="video4linux", $MATCH, ENV{ID_V4L_CAPABILITIES}=="*:capture:*", SYMLINK+="face_cam"
EOF
cat "$RULES" | sed 's/^/        /'

log "Reloading udev"
udevadm control --reload-rules
udevadm trigger --subsystem-match=video4linux
sleep 1
if [[ -e /dev/face_cam ]]; then
  ok "/dev/face_cam -> $(readlink -f /dev/face_cam)"
  info "start the service:  run/face.sh on   (health: curl -s localhost:8070/health)"
else
  warn "/dev/face_cam not present yet — unplug/replug the camera, or reboot, then check: ls -l /dev/face_cam"
fi
