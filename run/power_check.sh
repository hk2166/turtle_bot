#!/usr/bin/env bash
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — the original was lost; RECOVERY.md infers it used these
# same two vcgencmd calls on the Pi 5.
# Is the Pi's 5 V supply clean? The Pi browns out when the pack sags, which kills a mapping run.
#
#   run/power_check.sh            full report
#   run/power_check.sh --brief    one line (used by the pre-flight); exit 1 if undervoltage now or 5 V low,
#                                 exit 2 if it couldn't read the rail at all (never "clean" when unknown)
#   run/power_check.sh --watch    print the 5 V rail every second and flag dips (Ctrl-C to stop)
set -uo pipefail
MIN_5V=4.85   # the Pi 5 flags undervoltage at ~4.63 V; below ~4.85 V the buck or wiring is marginal

command -v vcgencmd >/dev/null || { echo "vcgencmd missing (apt install libraspberrypi-bin)"; exit 2; }

read_5v() { vcgencmd pmic_read_adc EXT5V_V 2>/dev/null | sed -n 's/.*=\([0-9.]*\)V.*/\1/p'; }
throttled() { vcgencmd get_throttled 2>/dev/null | sed -n 's/throttled=//p'; }

decode() {  # decode HEX → words
  local t=$(( $1 )) out=()
  (( t & 0x1 ))     && out+=("UNDERVOLTAGE NOW")
  (( t & 0x2 ))     && out+=("ARM freq capped now")
  (( t & 0x4 ))     && out+=("THROTTLED now")
  (( t & 0x8 ))     && out+=("soft temp limit now")
  (( t & 0x10000 )) && out+=("undervoltage since boot")
  (( t & 0x20000 )) && out+=("freq capped since boot")
  (( t & 0x40000 )) && out+=("throttled since boot")
  (( t & 0x80000 )) && out+=("soft temp limit since boot")
  ((${#out[@]})) && (IFS=,; echo "${out[*]}") || echo "clean"
}

case "${1:-}" in
  --brief)
    v="$(read_5v)"; t="$(throttled)"
    if [[ -z "$v" && -z "$t" ]]; then
      echo "5 V rail: couldn't read it (vcgencmd failed; is $USER in the 'video' group?)"
      exit 2
    fi
    msg="5 V rail ${v:-?} V, throttled=${t:-?} ($([[ -n "$t" ]] && decode "$t" || echo "throttle state unknown"))"
    echo "$msg"
    [[ -n "$t" ]] && (( t & 0x1 )) && exit 1
    [[ -n "$v" ]] && awk -v v="$v" -v m="$MIN_5V" 'BEGIN{exit !(v < m)}' && exit 1
    [[ -n "$t" ]] && (( t & 0x10000 )) && exit 1   # it dipped since boot: worth a warning
    exit 0 ;;
  --watch)
    lo=9
    echo "Watching the 5 V rail (flag < $MIN_5V V). Ctrl-C to stop."
    while true; do
      v="$(read_5v)"; t="$(throttled)"
      lo="$(awk -v a="$lo" -v b="${v:-9}" 'BEGIN{print (b<a)?b:a}')"
      flag=""; awk -v v="${v:-9}" -v m="$MIN_5V" 'BEGIN{exit !(v < m)}' && flag="  <-- DIP"
      printf '%s  5V=%s V  min=%s V  throttled=%s%s\n' "$(date +%T)" "${v:-?}" "$lo" "${t:-?}" "$flag"
      sleep 1
    done ;;
  *)
    t="$(throttled)"
    echo "5 V rail (EXT5V_V): $(read_5v) V   (want ≥ $MIN_5V V)"
    echo "get_throttled:      ${t:-?}  → $([[ -n "$t" ]] && decode "$t" || echo "couldn't read it")"
    echo "Core temp:          $(vcgencmd measure_temp | cut -d= -f2)"
    echo "Pack: keep it within 22–30 V (stop at 22 V until the 19.8 V figure is confirmed)."
    ;;
esac
