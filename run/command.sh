#!/usr/bin/env bash
# Written 2026-10-29: the voice/typed command understanding service (nodes/command_service.py, Claude).
#   run/command.sh [on|off|status]      127.0.0.1:8071 (localhost only; the dashboard calls it)
# Runs in ~/.venvs/llm (the anthropic SDK). API key: ~/.campus_bot/anthropic.key (never in the repo).
set -uo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

VENV_PY="${LLM_VENV:-$HOME/.venvs/llm}/bin/python"
PORT="${COMMAND_PORT:-8071}"
KEY="$HOME/.campus_bot/anthropic.key"
export STT_MODEL="${STT_MODEL:-base.en}" STT_THREADS="${STT_THREADS:-2}"   # from config/robot.env
export STT_OPENAI_MODEL="${STT_OPENAI_MODEL:-gpt-4o-transcribe}" STT_GROQ_MODEL="${STT_GROQ_MODEL:-whisper-large-v3-turbo}"
# Cloud speech keys: ~/.campus_bot/stt.env (read by the service itself; never in the repo)

case "${1:-on}" in
  on)
    is_running command && { ok "command service already on"; exit 0; }
    [[ -x "$VENV_PY" ]] || { err "venv missing: $VENV_PY"; \
      info "create it: python3 -m venv ~/.venvs/llm && ~/.venvs/llm/bin/pip install anthropic"; exit 1; }
    [[ -s "$KEY" || -n "${ANTHROPIC_API_KEY:-}" ]] || warn "no API key yet in $KEY: commands will say so until it's added (no restart needed)"
    start_bg command "$VENV_PY" "$PROJECT_DIR/nodes/command_service.py"
    for _ in $(seq 60); do curl -fsS "http://127.0.0.1:$PORT/health" >/dev/null 2>&1 && break; sleep 0.5; done
    if curl -fsS "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; then
      ok "command service on :$PORT  ($(curl -fsS "http://127.0.0.1:$PORT/health"))"
    else err "command service didn't come up: see $LOG_DIR/command.log"; tail -5 "$LOG_DIR/command.log" >&2; stop_bg command; exit 1; fi ;;
  off) stop_bg command; ok "command service off" ;;
  status)
    if is_running command; then echo "command: on  $(curl -fsS "http://127.0.0.1:$PORT/health" 2>/dev/null)"
    else echo "command: off"; fi ;;
  *) echo "usage: $0 [on|off|status]"; exit 2 ;;
esac
