#!/usr/bin/env bash
# Written 2026-10-09: hardware PWM on header pin 32 (GPIO12) for the buzzers (nodes/buzzer.py).
#   sudo setup/08_buzzer.sh          enable (then reboot once)
#   sudo setup/08_buzzer.sh --undo   remove both pieces
# 1. /boot/firmware/config.txt: dtoverlay=pwm,pin=12,func=4  (RP1 PWM0 channel 0 on GPIO12, Alt0)
# 2. campus-bot-pwm.service (root, at boot): exports that channel and lets group dialout (user resp
#    is in it) set its period/duty/enable, so the buzzer node needs no root.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo: sudo $0 $*"; exit 1; }
CFG=/boot/firmware/config.txt
LINE='dtoverlay=pwm,pin=12,func=4'
UNIT=/etc/systemd/system/campus-bot-pwm.service

if [[ "${1:-}" == --undo ]]; then
  sed -i "\|^$LINE\$|d; \|^# campus bot buzzer|d" "$CFG"
  systemctl disable --now campus-bot-pwm.service 2>/dev/null || true
  rm -f "$UNIT"; systemctl daemon-reload
  echo "removed; reboot to release GPIO12"; exit 0
fi

grep -qx "$LINE" "$CFG" || printf '\n# campus bot buzzer (setup/08_buzzer.sh)\n%s\n' "$LINE" >> "$CFG"
cat > "$UNIT" <<'EOF'
[Unit]
Description=Campus Bot: buzzer PWM (GPIO12) usable by group dialout
After=sysinit.target

[Service]
Type=oneshot
RemainAfterExit=yes
# the RP1 PWM0 block (1f00098000.pwm) is the one on the header; its chip number varies by boot
ExecStart=/bin/sh -c 'for c in /sys/class/pwm/pwmchip*; do \
  case "$(readlink -f $c/device)" in *98000.pwm) \
    [ -d $c/pwm0 ] || echo 0 > $c/export; sleep 0.2; \
    chgrp dialout $c/pwm0/period $c/pwm0/duty_cycle $c/pwm0/enable; \
    chmod g+w $c/pwm0/period $c/pwm0/duty_cycle $c/pwm0/enable; \
    ln -sfn $c/pwm0 /run/campus-bot-buzzer; echo "buzzer pwm: $c/pwm0";; esac; done; \
  [ -e /run/campus-bot-buzzer ] || { echo "no RP1 PWM0 found: reboot after setup?"; exit 1; }'

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable campus-bot-pwm.service
echo "done. Reboot once (sudo reboot); after that /run/campus-bot-buzzer points at the PWM channel."
