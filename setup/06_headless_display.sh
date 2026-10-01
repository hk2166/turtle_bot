#!/usr/bin/env bash
# Written 2026-10-01: keep the robot's desktop at 1920x1080 with NO monitor plugged in, so the VNC
# screen share (setup/x11vnc-session) looks the same with or without one.
#
#   sudo bash setup/06_headless_display.sh                 then reboot
#   sudo bash setup/06_headless_display.sh --autologin     also log resp in automatically at boot (Xorg)
#   sudo bash setup/06_headless_display.sh --undo          then reboot (removes both)
#
# --autologin: with no monitor nobody types the password at the login screen, so there is no desktop
# and no VNC after a reboot. This logs the user straight into "Ubuntu on Xorg" (Wayland off: x11vnc
# can't share Wayland). Trade-off: anyone with a monitor and keyboard on the robot gets the desktop.
#
# Without a monitor the HDMI port reads "disconnected", the desktop changes size, and the VNC picture
# and mouse go out of line. The kernel option video=HDMI-A-1:1920x1080@60D forces that port on ("D")
# at 1920x1080, whatever is plugged in. A monitor still works on it, at 1920x1080. It's the port
# next to the USB-C power socket (HDMI0). An HDMI dummy plug (~₹300) does the same job in hardware.
set -Eeuo pipefail
[[ $EUID -eq 0 ]] || { echo "Run it with sudo:  sudo bash $0 $*"; exit 1; }
F=/boot/firmware/cmdline.txt
OPT='video=HDMI-A-1:1920x1080@60D'
[[ -f "$F" ]] || { echo "$F not found"; exit 1; }
(( $(wc -l <"$F") <= 1 )) || { echo "$F has more than one line: not touching it"; exit 1; }
line="$(head -n1 "$F")"
cp -a "$F" "$F.bak-$(date +%Y%m%d-%H%M%S)"     # if the Pi won't boot: copy the .bak back from another computer

G=/etc/gdm3/custom.conf
U="${SUDO_USER:-resp}"
if [[ "${1:-}" == --undo ]]; then
  new="$(sed -E 's/ ?video=HDMI-A-1:[^ ]*//g' <<<"$line")"
  printf '%s\n' "$new" >"$F"
  [[ -f "$G" ]] && sed -i '/^# >>> campus_bot headless/,/^# <<< campus_bot headless/d' "$G"
  echo "Removed the forced HDMI mode and the auto-login. Reboot to apply."
  exit 0
fi
if [[ "${1:-}" == --autologin ]]; then
  [[ -f "$G" ]] || { echo "$G not found"; exit 1; }
  cp -a "$G" "$G.bak-$(date +%Y%m%d-%H%M%S)"
  # GDM's file is a GLib key file: comments must be on their own lines, never after a value
  sed -i '/^# >>> campus_bot headless/,/^# <<< campus_bot headless/d' "$G"
  sed -i "/^\[daemon\]/a # >>> campus_bot headless (setup/06_headless_display.sh)\nAutomaticLoginEnable=true\nAutomaticLogin=$U\nWaylandEnable=false\n# <<< campus_bot headless" "$G"
  echo "$G [daemon]:"; sed -n '/^\[daemon\]/,/^\[/p' "$G" | grep -v '^#'
fi
if grep -q 'video=HDMI-A-1:' <<<"$line"; then
  new="$(sed -E "s/video=HDMI-A-1:[^ ]*/$OPT/" <<<"$line")"
else
  new="$line $OPT"
fi
printf '%s\n' "$new" >"$F"
echo "$F now:"
cat "$F"
echo
echo "Reboot to apply (sudo reboot). Afterwards, with or without a monitor:"
echo "  DISPLAY=:0 xrandr | head -3     → HDMI-1 connected 1920x1080"
