#!/usr/bin/env bash
# Written 2026-10-01: make the robot fully usable with no monitor or keyboard — SSH for the terminal and
# remote desktop (xrdp, RDP on port 3389) for the screen, both over the office WiFi.
#
#   sudo bash setup/05_remote_access.sh        (safe to re-run; changes only what isn't set yet)
#
# 02_setup_pi.sh step 8 already installed SSH, xrdp and avahi (resp-desktop.local). This adds what a
# robot that boots with nobody logged in, and drives around on WiFi, needs:
#   - WiFi that comes up at boot and never stops retrying (NetworkManager gives up after 4 by default)
#   - WiFi power saving OFF: on the Pi's brcmfmac chip it adds 100+ ms latency spikes and dropouts
#   - SSH keepalives, so a session lost to a WiFi drop is noticed in ~1 min, not hours
#   - no "authentication required" colour-profile pop-ups inside the remote desktop
#   - firewall openings (only if ufw is active), and `iw` to check the WiFi link
# No passwords or keys are stored by this script.
set -Eeuo pipefail
[[ $EUID -eq 0 ]] || { echo "Run it with sudo:  sudo bash $0"; exit 1; }
U="${SUDO_USER:-resp}"
log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '    \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '    \033[33m! %s\033[0m\n' "$*"; }
have_pkg() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; }

log "Packages"
missing=()
for p in openssh-server xrdp xorgxrdp avahi-daemon iw; do have_pkg "$p" || missing+=("$p"); done
if ((${#missing[@]})); then
  info "installing: ${missing[*]}"
  DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}" \
    || { apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}"; }
fi
ok "openssh-server, xrdp, xorgxrdp, avahi-daemon, iw"

log "SSH (port 22)"
systemctl enable --now ssh.socket >/dev/null 2>&1 || systemctl enable --now ssh
cat >/etc/ssh/sshd_config.d/10-campus-bot.conf <<'EOF'
# campus_bot (setup/05_remote_access.sh): notice clients lost to a WiFi drop within ~1 minute, so their
# programs get SIGHUP instead of hanging on for hours.
ClientAliveInterval 15
ClientAliveCountMax 4
EOF
install -d -m 0755 /run/sshd          # sshd -t needs it; with on-demand SSH it may not exist yet
if sshd -t; then ok "sshd config valid (keepalive 15 s × 4)"
else warn "sshd rejected the config: removing the campus_bot drop-in"; rm -f /etc/ssh/sshd_config.d/10-campus-bot.conf; fi
info "password login: $(sshd -T 2>/dev/null | awk '/^passwordauthentication/{print $2}')   root login: $(sshd -T 2>/dev/null | awk '/^permitrootlogin/{print $2}')"

log "Remote desktop (xrdp, port 3389)"
adduser xrdp ssl-cert >/dev/null 2>&1 || true          # lets xrdp read the TLS key
systemctl enable --now xrdp >/dev/null
# GNOME in an xrdp session otherwise asks for a password to create colour profiles, every login.
cat >/etc/polkit-1/rules.d/45-campus-bot-colord.rules <<'EOF'
// campus_bot (setup/05_remote_access.sh): no colour-manager password prompts in remote desktop sessions
polkit.addRule(function (action, subject) {
  if (action.id.indexOf("org.freedesktop.color-manager.") === 0 && subject.isInGroup("users")) {
    return polkit.Result.YES;
  }
});
EOF
if grep -qs '>>> campus_bot xrdp >>>' "/home/$U/.xsessionrc"; then ok "GNOME session settings in ~$U/.xsessionrc"
else warn "~$U/.xsessionrc has no campus_bot block: re-run 02_setup_pi.sh step 8, or the remote desktop may be black"; fi
systemctl is-active --quiet xrdp && ok "xrdp running" || warn "xrdp isn't running: journalctl -u xrdp"

log "WiFi"
CONN="$(nmcli -t -f NAME,TYPE connection show --active | awk -F: '$2=="802-11-wireless"{print $1; exit}')"
DEV="$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')"
# Power saving off for every WiFi network (the office one, or a phone hotspot at a demo).
cat >/etc/NetworkManager/conf.d/99-campus-bot-wifi-powersave-off.conf <<'EOF'
# campus_bot (setup/05_remote_access.sh): WiFi power saving off (overrides Ubuntu's default "3" = on).
[connection]
wifi.powersave = 2
EOF
if [[ -n "$CONN" ]]; then
  nmcli connection modify "$CONN" connection.autoconnect yes connection.autoconnect-retries 0 \
    802-11-wireless.powersave 2
  ok "'$CONN': connects at boot, retries forever, power saving off"
  perms="$(nmcli -g connection.permissions connection show "$CONN")"
  flags="$(nmcli -g 802-11-wireless-security.psk-flags connection show "$CONN" 2>/dev/null || echo 0)"
  [[ -z "$perms" ]] || warn "'$CONN' is limited to user(s) $perms: it won't connect until they log in"
  [[ "$flags" == 0 || -z "$flags" ]] || warn "'$CONN' keeps its password in a user keyring: it won't connect at boot"
else
  warn "no WiFi connection is active right now: connect to the office WiFi, then re-run this"
fi
if [[ -n "$DEV" ]]; then
  iw dev "$DEV" set power_save off 2>/dev/null || true     # now, without reconnecting
  info "$DEV power save: $(iw dev "$DEV" get power_save 2>/dev/null | awk '{print $NF}')"
  iw dev "$DEV" link 2>/dev/null | grep -E 'SSID|signal|tx bitrate' | sed 's/^\s*/    /' || true
fi

log "Firewall"
if ufw status 2>/dev/null | grep -q 'Status: active'; then
  ufw allow 22/tcp >/dev/null; ufw allow 3389/tcp >/dev/null; ok "ufw: 22 and 3389 open"
else
  info "ufw not active: nothing to open"
fi

log "How to connect"
IP="$(ip -4 -o addr show "${DEV:-wlan0}" 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
info "Terminal:        ssh $U@$(hostname).local        (or ssh $U@${IP:-<ip>})"
info "Remote desktop:  Windows App (Mac) / Remote Desktop → PC name $(hostname).local (or ${IP:-<ip>}), user $U"
if loginctl list-sessions --no-legend 2>/dev/null | awk -v u="$U" '$3==u && $4=="seat0"' | grep -q .; then
  warn "$U is logged in on the robot's own screen right now. xrdp shows a BLACK screen for a user who is"
  warn "also logged in locally: log out there (or reboot without logging in) before using remote desktop."
fi
info "Security: SSH and remote desktop accept $U's password from anywhere on the office WiFi."
info "Use a strong password (passwd), and preferably SSH keys (see README → Remote access)."
