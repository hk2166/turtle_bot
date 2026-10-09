#!/usr/bin/env bash
# Written 2026-10-09: hardware PWM on header pins 32 + 33 (GPIO12 front, GPIO13 rear) for the buzzers.
#   sudo setup/08_buzzer.sh          enable (then reboot once)
#   sudo setup/08_buzzer.sh --undo   remove all three pieces
# 1. /boot/firmware/config.txt: dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#    (RP1 PWM0 channel 0 on GPIO12, channel 1 on GPIO13, both Alt0)
# 2. /usr/local/sbin/campus-bot-pwm (root-owned, so the root service can't be edited from the repo):
#    exports both channels, lets group dialout (user resp is in it) set period/duty/enable, and
#    links them as /run/campus-bot-buzzer/front and /run/campus-bot-buzzer/rear.
# 3. campus-bot-pwm.service runs it at boot.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo: sudo $0 $*"; exit 1; }
CFG=/boot/firmware/config.txt
LINE='dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4'
HELPER=/usr/local/sbin/campus-bot-pwm
UNIT=/etc/systemd/system/campus-bot-pwm.service

if [[ "${1:-}" == --undo ]]; then
  sed -i "\|^dtoverlay=pwm.*pin=12,func=4|d; \|^# campus bot buzzer|d" "$CFG"
  systemctl disable --now campus-bot-pwm.service 2>/dev/null || true
  rm -f "$UNIT" "$HELPER"; systemctl daemon-reload
  echo "removed; reboot to release GPIO12/13"; exit 0
fi

sed -i "\|^dtoverlay=pwm,pin=12,func=4\$|d" "$CFG"    # the 1-channel version of this script
grep -qx "$LINE" "$CFG" || printf '\n# campus bot buzzer (setup/08_buzzer.sh)\n%s\n' "$LINE" >> "$CFG"

cat > "$HELPER" <<'EOF'
#!/bin/sh
# Installed by campus_bot_fw/setup/08_buzzer.sh. Run as root at boot by campus-bot-pwm.service.
# The RP1 PWM0 block (...98000.pwm) drives the header pins; its pwmchip number can change per boot.
mkdir -p /run/campus-bot-buzzer
for c in /sys/class/pwm/pwmchip*; do
  case "$(readlink -f "$c/device")" in
    *98000.pwm)
      for n in 0 1; do [ -d "$c/pwm$n" ] || echo "$n" > "$c/export"; done
      sleep 0.3
      for n in 0 1; do
        for f in period duty_cycle enable; do chgrp dialout "$c/pwm$n/$f"; chmod g+w "$c/pwm$n/$f"; done
      done
      ln -sfn "$c/pwm0" /run/campus-bot-buzzer/front
      ln -sfn "$c/pwm1" /run/campus-bot-buzzer/rear
      echo "buzzer pwm: front $c/pwm0, rear $c/pwm1" ;;
  esac
done
[ -e /run/campus-bot-buzzer/front ] || { echo "no RP1 PWM0 found: reboot after setup?"; exit 1; }
EOF
chmod 755 "$HELPER"

cat > "$UNIT" <<EOF
[Unit]
Description=Campus Bot: buzzer PWM (GPIO12 front, GPIO13 rear) usable by group dialout
After=sysinit.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=$HELPER

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable campus-bot-pwm.service
echo "done. Reboot once (sudo reboot); after that /run/campus-bot-buzzer/{front,rear} are the two channels."
