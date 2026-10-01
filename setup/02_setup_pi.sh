#!/usr/bin/env bash
# 02_setup_pi.sh — one-time setup of a fresh Ubuntu 24.04 Desktop (arm64) Raspberry Pi 5 for Campus Bot.
#
# Installs/configures: ROS 2 Jazzy desktop + slam_toolbox + Nav2 + teleop + IMU/camera packages,
# the RPLIDAR C1 driver (sllidar_ros2, built in ~/ros2_ws), the micro-ROS agent (built in ~/microros_ws),
# udev names for the lidar/ESP32 serial ports, headless access (xrdp remote desktop + SSH),
# robot-friendly desktop settings (no sleep, no automount), the face-verification Python venv,
# and ESP32 tools (esptool, PlatformIO).
#
# Usage (from a terminal on the Pi's desktop, as your normal user — NOT with sudo):
#     bash 02_setup_pi.sh [--hostname resp-desktop] [--usb-max-current] [--skip-face] [--skip-firmware-tools]
#
#   --hostname NAME      set the hostname (the old Pi was resp-desktop → reachable as resp-desktop.local)
#   --usb-max-current    let the USB ports supply up to 1.6 A total (default 600 mA without an official
#                        PD supply). Lidar + webcam + ESP32 can exceed 600 mA. Only use it if the 5 V buck
#                        can deliver ~5 A.
#   --skip-face          don't build the InsightFace/FastAPI venv
#   --skip-firmware-tools  don't install esptool / PlatformIO
#
# Safe to re-run: every step checks before it changes anything. Takes 45–90 min the first time.
# Full log: ~/campus_bot_setup.log
set -Eeuo pipefail

HOSTNAME_WANTED=""
USB_MAX_CURRENT=0
SKIP_FACE=0
SKIP_FW=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --hostname) HOSTNAME_WANTED="${2:?--hostname needs a value}"; shift 2 ;;
    --usb-max-current) USB_MAX_CURRENT=1; shift ;;
    --skip-face) SKIP_FACE=1; shift ;;
    --skip-firmware-tools) SKIP_FW=1; shift ;;
    -h|--help) sed -n '2,23p' "$0"; exit 0 ;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
done

ROS_DISTRO=jazzy
PROJECT_DIR="$HOME/Desktop/campus_bot_fw"
ROS_WS="$HOME/ros2_ws"
MICROROS_WS="$HOME/microros_ws"
FACE_VENV="$HOME/.venvs/face"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$HOME/campus_bot_setup.log"
STEPS=12
exec > >(tee -a "$LOG") 2>&1

WARNINGS=()
step() { printf '\n\033[1;34m==> [%s] (%s/%s) %s\033[0m\n' "$(date +%H:%M:%S)" "$1" "$STEPS" "$2"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$*"; WARNINGS+=("$*"); }
die()  { printf '\033[1;31m[fail] %s\033[0m\n' "$*"; exit 1; }
trap 'die "line $LINENO: \"$BASH_COMMAND\" failed — see $LOG. Fix it and re-run; finished steps are skipped."' ERR

apt_get() { sudo env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l apt-get -o DPkg::Lock::Timeout=900 "$@"; }
# ROS setup files read unset variables, so never source them under `set -u`.
ros_source() { set +u; # shellcheck disable=SC1090
  source "$1"; set -u; }
have_pkg() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; }

# ---------------------------------------------------------------------------------------------
step 1 "Preflight"
[[ $EUID -ne 0 ]] || die "Run as your normal user (e.g. resp), not root/sudo. The script calls sudo itself."
# shellcheck disable=SC1091
. /etc/os-release
[[ "${VERSION_CODENAME:-}" == noble ]] || die "Needs Ubuntu 24.04 (noble); this is ${PRETTY_NAME:-unknown}."
[[ "$(uname -m)" == aarch64 ]] || warn "Expected aarch64 (Raspberry Pi 5); this is $(uname -m)."
MODEL="$(tr -d '\0' </proc/device-tree/model 2>/dev/null || echo unknown)"
[[ "$MODEL" == *"Raspberry Pi 5"* ]] || warn "Board reports '$MODEL', expected a Raspberry Pi 5."
info "Board: $MODEL | OS: $PRETTY_NAME | user: $USER"
sudo -v
# Keep sudo alive for the long installs.
( trap - ERR; while kill -0 "$$" 2>/dev/null; do sudo -n true 2>/dev/null || true; sleep 50; done ) &
if ! grep -rqs 'noble-backports' /etc/apt/sources.list /etc/apt/sources.list.d/; then
  die "noble-backports isn't enabled, and the ROS 2 packages need it. Add 'noble-backports' to the 'Suites:' line in /etc/apt/sources.list.d/ubuntu.sources, then re-run."
fi
if ! locale | grep -qi 'utf-8'; then
  apt_get install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi

# ---------------------------------------------------------------------------------------------
step 2 "System update + base packages"
apt_get update
apt_get install -y software-properties-common
sudo add-apt-repository -y universe
apt_get update
apt_get upgrade -y
apt_get install -y \
  curl wget gpg git build-essential cmake \
  python3-pip python3-venv python3-dev python3-serial python3-yaml python3-opencv python3-pil pipx \
  htop tmux vim jq rsync net-tools usbutils i2c-tools v4l-utils minicom screen gh \
  openssh-server avahi-daemon xrdp \
  gddrescue testdisk e2fsprogs dosfstools \
  libraspberrypi-bin   # vcgencmd: undervoltage / 5 V rail checks for run/power_check.sh

# ---------------------------------------------------------------------------------------------
step 3 "ROS 2 Jazzy apt source"
if have_pkg ros2-apt-source; then
  info "ros2-apt-source already installed"
else
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  [[ -n "$ROS_APT_SOURCE_VERSION" ]] || die "Couldn't read the ros-apt-source release from GitHub (rate limit?). Wait a few minutes and re-run."
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi
apt_get update

# ---------------------------------------------------------------------------------------------
step 4 "ROS 2 Jazzy desktop + robot packages (the long one: 20–40 min)"
apt_get install -y "ros-${ROS_DISTRO}-desktop" ros-dev-tools
ROS_PKGS=(
  slam-toolbox navigation2 nav2-bringup     # mapping (remap.sh), AMCL + Nav2 (--nav)
  teleop-twist-keyboard twist-mux           # driving while mapping
  robot-localization imu-tools              # gyro/IMU fusion & filtering
  tf2-tools xacro                           # TF checks (view_frames), robot description
  usb-cam v4l2-camera                       # Logitech Brio on the mast (run/camera.sh)
  laser-filters rqt-robot-steering
)
if ! apt_get install -y "${ROS_PKGS[@]/#/ros-${ROS_DISTRO}-}"; then
  warn "Bulk ROS install failed; retrying one package at a time."
  for p in "${ROS_PKGS[@]}"; do apt_get install -y "ros-${ROS_DISTRO}-$p" || warn "ros-${ROS_DISTRO}-$p did not install"; done
fi
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update --rosdistro "$ROS_DISTRO"

# ---------------------------------------------------------------------------------------------
step 5 "RPLIDAR C1 driver (Slamtec sllidar_ros2 → $ROS_WS)"
mkdir -p "$ROS_WS/src"
[[ -d "$ROS_WS/src/sllidar_ros2/.git" ]] || git clone https://github.com/Slamtec/sllidar_ros2.git "$ROS_WS/src/sllidar_ros2"
(
  ros_source "/opt/ros/$ROS_DISTRO/setup.bash"
  cd "$ROS_WS"
  rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO"
  colcon build --symlink-install
)
info "C1 launch: ros2 launch sllidar_ros2 sllidar_c1_launch.py   (defaults: /dev/ttyUSB0, 460800 baud, frame 'laser')"

# ---------------------------------------------------------------------------------------------
step 6 "micro-ROS agent (the ESP32 talks to ROS through it → $MICROROS_WS)"
mkdir -p "$MICROROS_WS/src"
[[ -d "$MICROROS_WS/src/micro_ros_setup/.git" ]] || \
  git clone -b "$ROS_DISTRO" https://github.com/micro-ROS/micro_ros_setup.git "$MICROROS_WS/src/micro_ros_setup"
(
  ros_source "/opt/ros/$ROS_DISTRO/setup.bash"
  cd "$MICROROS_WS"
  if [[ -f install/local_setup.bash ]]; then ros_source install/local_setup.bash; fi
  if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
    echo "    micro_ros_agent already built"
  else
    rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO"
    colcon build
    ros_source install/local_setup.bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
  fi
)
info "Run: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200   (verified 2026-10-01)"

# ---------------------------------------------------------------------------------------------
step 7 "Serial devices: udev names, permissions, brltty"
sudo tee /etc/udev/rules.d/99-campus-bot.rules >/dev/null <<'EOF'
# Campus Bot serial devices — stable names on top of /dev/ttyUSB*/ttyACM* (which still work).
# TO VERIFY on the robot:  lsusb   and   udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'
# If the ESP32 board's UART chip is ALSO a CP2102 (10c4:ea60), both would match the lidar line:
# pin each one with ATTRS{serial}=="…" instead.
# ID_MM_DEVICE_IGNORE stops ModemManager probing these ports (it can garble the first bytes).

# RPLIDAR C1 through the CP2102 USB-TTL stick (Silicon Labs) → /dev/lidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", ENV{ID_MM_DEVICE_IGNORE}="1"
# ESP32-S3 native USB port (Espressif) → /dev/esp32. micro-ROS runs on THIS port (verified 2026-10-01).
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", SYMLINK+="esp32", ENV{ID_MM_DEVICE_IGNORE}="1"
# ESP32-S3 board UART port (WCH CH34x bridge): only the firmware's text log → /dev/esp32_log.
# It must never be /dev/esp32, or the agent would talk micro-ROS to the log port.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="esp32_log", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
# brltty (braille display daemon) grabs CH34x/CP210x USB-serial adapters on Ubuntu — disable its udev hook.
if have_pkg brltty; then
  sudo ln -sf /dev/null /etc/udev/rules.d/85-brltty.rules
  sudo systemctl mask brltty-udev.service >/dev/null 2>&1 || true
  sudo systemctl stop brltty-udev.service brltty.service >/dev/null 2>&1 || true
  info "brltty's USB-serial grab disabled"
fi
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
for g in dialout video plugdev i2c gpio; do
  if getent group "$g" >/dev/null && ! id -nG "$USER" | tr ' ' '\n' | grep -qx "$g"; then
    sudo usermod -aG "$g" "$USER"; info "added $USER to group $g (takes effect after reboot)"
  fi
done

# ---------------------------------------------------------------------------------------------
step 8 "Headless access: SSH + xrdp remote desktop + hostname"
if ! systemctl is-active --quiet ssh.socket && ! systemctl is-active --quiet ssh; then
  sudo systemctl enable --now ssh
fi
sudo systemctl enable --now avahi-daemon
getent group ssl-cert >/dev/null && sudo usermod -aG ssl-cert xrdp
sudo systemctl enable --now xrdp
# Remote desktop sessions: XFCE via ~/.xsession (setup/05_remote_access.sh installs it). A second
# GNOME for a user logged in on the monitor crashes both.
cp "$PROJECT_DIR/setup/xsession" "$HOME/.xsession" 2>/dev/null && chmod +x "$HOME/.xsession" || true
sleep 2
if ! systemctl is-active --quiet xrdp; then
  warn "xrdp isn't running. If GNOME's own Settings → System → Remote Desktop → Remote Login is on, it holds port 3389 — turn it off and re-run."
fi
if sudo ufw status 2>/dev/null | grep -q 'Status: active'; then
  sudo ufw allow 22/tcp; sudo ufw allow 3389/tcp
fi
if [[ -n "$HOSTNAME_WANTED" && "$(hostname)" != "$HOSTNAME_WANTED" ]]; then
  OLD_HOST="$(hostname)"
  sudo hostnamectl set-hostname "$HOSTNAME_WANTED"
  if grep -q '^127\.0\.1\.1' /etc/hosts; then
    sudo sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$HOSTNAME_WANTED/" /etc/hosts
  else
    printf '127.0.1.1\t%s\n' "$HOSTNAME_WANTED" | sudo tee -a /etc/hosts >/dev/null
  fi
  info "hostname $OLD_HOST → $HOSTNAME_WANTED (reach it as $HOSTNAME_WANTED.local after reboot)"
fi
# Headless only works if WiFi connects at boot, before anyone logs in.
if command -v nmcli >/dev/null; then
  while IFS=: read -r name type; do
    [[ "$type" == 802-11-wireless ]] || continue
    perms="$(nmcli -g connection.permissions connection show "$name" 2>/dev/null || true)"
    auto="$(nmcli -g connection.autoconnect connection show "$name" 2>/dev/null || true)"
    flags="$(nmcli -g 802-11-wireless-security.psk-flags connection show "$name" 2>/dev/null || true)"
    if [[ -z "$perms" && "$auto" == yes && "${flags:-0}" == 0* ]]; then
      info "WiFi '$name' connects at boot ✓"
    else
      warn "WiFi '$name' only connects after you log in. For headless use: Settings → Wi-Fi → ⚙ on '$name' → tick 'Connect automatically' AND 'Available to all users', re-enter the password."
    fi
  done < <(nmcli -t -f NAME,TYPE connection show 2>/dev/null)
fi

# ---------------------------------------------------------------------------------------------
step 9 "Robot-friendly system settings (no sleep, no automount, USB current)"
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target >/dev/null
if [[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ]] && command -v gsettings >/dev/null; then
  # No automount: the corrupted old SD card must never be mounted read-write when you plug it in.
  gset() { gsettings set "$@" || warn "gsettings set $* failed"; }
  gset org.gnome.desktop.media-handling automount false
  gset org.gnome.desktop.media-handling automount-open false
  gset org.gnome.desktop.session idle-delay 0
  gset org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
  info "GNOME: automount off, no idle blanking, no sleep"
else
  warn "No desktop session found (ran over SSH?). Re-run from the Pi's own terminal to turn off automount BEFORE plugging in the old SD card."
fi
BOOTCFG=/boot/firmware/config.txt
if (( USB_MAX_CURRENT )); then
  if grep -qs '^usb_max_current_enable=1' "$BOOTCFG"; then
    info "usb_max_current_enable=1 already set"
  else
    sudo cp "$BOOTCFG" "$BOOTCFG.bak.$(date +%Y%m%d-%H%M%S)"
    printf '\n[all]\n# Campus Bot: lidar + webcam + ESP32 exceed the 600 mA default without a PD supply\nusb_max_current_enable=1\n' | sudo tee -a "$BOOTCFG" >/dev/null
    info "usb_max_current_enable=1 added to $BOOTCFG (active after reboot)"
  fi
else
  info "USB current left at the default. If the lidar/webcam drop out, re-run with --usb-max-current (needs a buck that can supply ~5 A)."
fi

# ---------------------------------------------------------------------------------------------
step 10 "Face verification venv ($FACE_VENV)"
if (( SKIP_FACE )); then
  info "skipped (--skip-face)"
elif [[ -x "$FACE_VENV/bin/python" ]] && "$FACE_VENV/bin/python" -c 'import insightface, onnxruntime, cv2, fastapi, cryptography' 2>/dev/null; then
  info "already set up"
else
  # Separate venv: keeps pip's numpy/opencv away from the system copies ROS uses.
  # insightface builds a C extension from source on arm64; build it against the venv's numpy.
  if python3 -m venv "$FACE_VENV" \
     && "$FACE_VENV/bin/pip" install --upgrade pip wheel setuptools \
     && "$FACE_VENV/bin/pip" install numpy cython \
     && "$FACE_VENV/bin/pip" install --no-build-isolation insightface \
     && "$FACE_VENV/bin/pip" install onnxruntime opencv-python-headless fastapi "uvicorn[standard]" \
          python-multipart cryptography pyserial pandas matplotlib; then
    info "face stack installed (InsightFace buffalo_s models download on first use)"
  else
    warn "Face stack install failed — see $LOG. Robot/mapping setup is unaffected."
  fi
fi
if ! command -v ngrok >/dev/null; then
  if sudo snap install ngrok >/dev/null 2>&1; then
    info "ngrok installed (enrolment HTTPS tunnel). Add your token yourself: ngrok config add-authtoken <token>"
  else
    warn "ngrok not installed (only needed for the phone enrolment tunnel)."
  fi
fi

# ---------------------------------------------------------------------------------------------
step 11 "ESP32 tools"
if (( SKIP_FW )); then
  info "skipped (--skip-firmware-tools)"
else
  apt_get install -y esptool
  if ! command -v pio >/dev/null && [[ ! -x "$HOME/.local/bin/pio" ]]; then
    pipx install platformio || warn "PlatformIO install failed (only needed to rebuild ESP32 firmware)."
  fi
  pipx ensurepath >/dev/null 2>&1 || true
fi

# ---------------------------------------------------------------------------------------------
step 12 "Project folder + shell environment"
mkdir -p "$PROJECT_DIR"/{run,config,maps,setup}
for f in CLAUDE.md RECOVERY.md; do
  if [[ -f "$SCRIPT_DIR/$f" && ! -f "$PROJECT_DIR/$f" ]]; then cp "$SCRIPT_DIR/$f" "$PROJECT_DIR/$f"; fi
done
if [[ ! "$SCRIPT_DIR" -ef "$PROJECT_DIR/setup" ]]; then
  for f in "$SCRIPT_DIR"/0*.sh "$SCRIPT_DIR/README.md"; do
    if [[ -f "$f" ]]; then cp -f "$f" "$PROJECT_DIR/setup/"; fi
  done
fi
if [[ ! -f "$PROJECT_DIR/.gitignore" ]]; then
  cat > "$PROJECT_DIR/.gitignore" <<'EOF'
# colcon / python
build/
install/
log/
__pycache__/
*.pyc
.venv/
# never commit: face data, keys, tokens, flash dumps (may contain WiFi credentials), recovery dumps
*.enc
*.key
.env
faces/
recovered/
*.bin
EOF
fi
[[ -d "$PROJECT_DIR/.git" ]] || git -C "$PROJECT_DIR" init -q -b main
if ! grep -qs '>>> campus_bot >>>' "$HOME/.bashrc"; then
  cat >> "$HOME/.bashrc" <<'EOF'

# >>> campus_bot >>>
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/microros_ws/install/local_setup.bash" ] && source "$HOME/microros_ws/install/local_setup.bash"
export PATH="$HOME/.local/bin:$PATH"
alias cb='cd ~/Desktop/campus_bot_fw'
# <<< campus_bot <<<
EOF
fi

# ---------------------------------------------------------------------------------------------
printf '\n\033[1;34m==> Verification\033[0m\n'
ros_source "/opt/ros/$ROS_DISTRO/setup.bash"
[[ -f "$ROS_WS/install/setup.bash" ]] && ros_source "$ROS_WS/install/setup.bash"
[[ -f "$MICROROS_WS/install/local_setup.bash" ]] && ros_source "$MICROROS_WS/install/local_setup.bash"
check() {  # check "label" command...
  local label="$1"; shift
  if "$@" >/dev/null 2>&1; then printf '  \033[32m✓\033[0m %s\n' "$label"
  else printf '  \033[31m✗\033[0m %s\n' "$label"; WARNINGS+=("check failed: $label"); fi
}
check "ros2 CLI (Jazzy)"                 bash -c 'command -v ros2'
check "slam_toolbox"                     ros2 pkg prefix slam_toolbox
check "nav2_bringup"                     ros2 pkg prefix nav2_bringup
check "teleop_twist_keyboard"            ros2 pkg prefix teleop_twist_keyboard
check "sllidar_ros2 (RPLIDAR C1)"        ros2 pkg prefix sllidar_ros2
check "micro_ros_agent"                  ros2 pkg prefix micro_ros_agent
check "rviz2"                            ros2 pkg prefix rviz2
check "xrdp running"                     systemctl is-active --quiet xrdp
check "ssh reachable"                    bash -c 'systemctl is-active --quiet ssh.socket || systemctl is-active --quiet ssh'
check "vcgencmd (power_check)"           sudo vcgencmd get_throttled
(( SKIP_FACE )) || check "face venv (insightface)" "$FACE_VENV/bin/python" -c 'import insightface, onnxruntime, cv2'
(( SKIP_FW ))   || check "esptool"       bash -c 'command -v esptool || command -v esptool.py'
info "Power now: $(sudo vcgencmd get_throttled 2>/dev/null || echo 'n/a')   (throttled=0x0 means no undervoltage since boot)"
info "Serial devices: $(ls /dev/ttyUSB* /dev/ttyACM* /dev/lidar /dev/esp32 2>/dev/null | tr '\n' ' ' || true)"

if ((${#WARNINGS[@]})); then
  printf '\n\033[1;33m%s warning(s):\033[0m\n' "${#WARNINGS[@]}"
  printf '  - %s\n' "${WARNINGS[@]}"
fi
cat <<EOF

Setup finished. Next:
  1. Reboot (groups, hostname, udev and kernel updates apply):   sudo reboot
  2. After reboot, with the OLD SD card in a USB reader:          bash $PROJECT_DIR/setup/03_recover_old_sd.sh
  3. ESP32 plugged in by its native USB-C port:                   bash $PROJECT_DIR/setup/04_backup_esp32.sh
  4. cd $PROJECT_DIR && claude   → "Read RECOVERY.md and continue the rebuild."
Remote desktop from the Mac: Windows App → Add PC → $(hostname).local  (log out of the Pi's monitor session first).
EOF
