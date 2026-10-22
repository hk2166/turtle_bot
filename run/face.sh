#!/usr/bin/env bash
# Written 2026-10-22: run the warm face-recognition service (face/face_service.py), which the delivery
# flow calls to verify a recipient or identify a sender.
#   run/face.sh [on|off|status]        default on → http://127.0.0.1:8070 (localhost only)
# It runs in the face venv (insightface/onnxruntime), owns the SECOND camera (/dev/face_cam — see
# setup/10_face_cam.sh), and keeps the model loaded. It does NOT touch the Brio (/dev/video0), so it
# can run alongside run/camera.sh and navigation.
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

VENV_PY="${FACE_VENV:-$HOME/.venvs/face}/bin/python"
PORT="${FACE_PORT:-8070}"

case "${1:-on}" in
  on)
    is_running face && { ok "face service already on"; exit 0; }
    [[ -x "$VENV_PY" ]] || { err "face venv missing: $VENV_PY"; \
      info "create it: python3 -m venv ~/.venvs/face && ~/.venvs/face/bin/pip install -r face/requirements.txt"; exit 1; }
    # the service finds the HIKVISION itself (any USB camera that isn't the Brio); /dev/face_cam is optional
    [[ -e /dev/face_cam ]] || ls /sys/class/video4linux/ >/dev/null 2>&1 || warn "no camera found: the service starts without one"
    start_bg face "$VENV_PY" "$PROJECT_DIR/face/face_service.py"
    for _ in $(seq 20); do
      curl -fsS "http://127.0.0.1:$PORT/health" >/dev/null 2>&1 && break; sleep 0.25
    done
    if curl -fsS "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; then
      h="$(curl -fsS "http://127.0.0.1:$PORT/health" 2>/dev/null)"
      ok "face service on :$PORT  ($h)"
      info "the model finishes loading in the background (first run downloads buffalo_s ~120 MB)"
    else err "face service didn't come up: see $LOG_DIR/face.log"; tail -5 "$LOG_DIR/face.log" >&2; stop_bg face; exit 1; fi ;;
  off) stop_bg face; ok "face service off" ;;
  status)
    if is_running face; then
      echo "face: on  $(curl -fsS "http://127.0.0.1:$PORT/health" 2>/dev/null || echo '(not answering /health yet)')"
    else echo "face: off"; fi ;;
  *) echo "usage: $0 [on|off|status]"; exit 2 ;;
esac
