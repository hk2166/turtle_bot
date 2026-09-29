#!/usr/bin/env bash
# 03_recover_old_sd.sh — copy the robot code, Claude Code history and system config off the
# corrupted SD card. READ-ONLY: nothing is ever written to the old card.
#
#   1. Boot the Pi from its NEW card. Only then plug the OLD card in through a USB card reader.
#      (Both cards carry the same partition labels; if the old one is in at boot, the Pi may boot from it.)
#   2. bash 03_recover_old_sd.sh             # finds the old card by itself
#      bash 03_recover_old_sd.sh /dev/sda    # or name it (see `lsblk`)
#
# Safety: the card is set read-only at the block-device level (blockdev --setro), and the ext4
# partition is mounted ro,noload (the damaged journal is not replayed).
# Output: ~/recovered/<timestamp>/  (never commit this folder: it can hold face data and credentials)
set -Eeuo pipefail

log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31m[fail] %s\033[0m\n' "$*"; exit 1; }
trap 'die "line $LINENO: \"$BASH_COMMAND\" failed."' ERR

[[ $EUID -ne 0 ]] || die "Run as your normal user; the script calls sudo itself."
sudo -v

ROOT_SRC="$(findmnt -no SOURCE /)"
ROOT_PK="$(lsblk -no PKNAME "$ROOT_SRC" 2>/dev/null | head -1 || true)"
ROOT_DISK="${ROOT_PK:+/dev/$ROOT_PK}"; ROOT_DISK="${ROOT_DISK:-$ROOT_SRC}"

# --- pick the old card --------------------------------------------------------------------------
if [[ $# -ge 1 ]]; then
  DISK="$1"
else
  mapfile -t CANDS < <(lsblk -dpno NAME,TYPE | awk '$2=="disk"{print $1}' | grep -vxF "$ROOT_DISK" | grep -vE '^/dev/(zram|ram|loop)' || true)
  case ${#CANDS[@]} in
    0) die "No second disk found. Plug the old SD card in through a USB card reader, wait 5 s, re-run." ;;
    1) DISK="${CANDS[0]}" ;;
    *) lsblk -o NAME,SIZE,TRAN,MODEL,FSTYPE,LABEL; die "Several disks found (${CANDS[*]}). Re-run naming the old card, e.g.: bash $0 /dev/sda" ;;
  esac
fi
[[ -b "$DISK" ]] || die "$DISK is not a block device."
[[ "$DISK" != "$ROOT_DISK" ]] || die "$DISK is the card the Pi is running from — refusing."
log "Old card: $DISK   (running system is on $ROOT_DISK)"
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS "$DISK" || true

mapfile -t PARTS < <(lsblk -lnpo NAME,TYPE "$DISK" | awk '$2=="part"{print $1}')
((${#PARTS[@]})) || PARTS=("$DISK")   # filesystem directly on the device (e.g. a loop image)

# Unmount anything the desktop auto-mounted, then make the whole device read-only in the kernel.
for p in "${PARTS[@]}"; do
  while read -r mp; do [[ -n "$mp" ]] && { warn "unmounting auto-mounted $p ($mp)"; sudo umount "$mp"; }; done < <(lsblk -no MOUNTPOINTS "$p" 2>/dev/null)
done
sudo blockdev --setro "$DISK"
for p in "${PARTS[@]}"; do sudo blockdev --setro "$p"; done
info "set read-only at block level: $DISK ${PARTS[*]}"

DEST="$HOME/recovered/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$DEST"
chmod 700 "$HOME/recovered" "$DEST"
MNT_ROOT=/mnt/oldsd_root
MNT_BOOT=/mnt/oldsd_boot
cleanup() { sudo umount "$MNT_ROOT" 2>/dev/null || true; sudo umount "$MNT_BOOT" 2>/dev/null || true; }
trap cleanup EXIT

# rsync that tolerates unreadable files (exit 23/24 = partial copy) instead of aborting.
grab() {  # grab SRC DEST_SUBDIR [extra rsync args...]
  local src="$1" sub="$2"; shift 2
  sudo test -e "$src" || return 0   # sudo: the old home may belong to a different uid
  mkdir -p "$DEST/$sub"
  local rc=0
  sudo rsync -a --no-owner --no-group "$@" "$src" "$DEST/$sub/" || rc=$?
  case $rc in
    0) info "✓ $src" ;;
    23|24) warn "partial copy (some files unreadable): $src" ;;
    *) warn "copy failed (rsync $rc): $src" ;;
  esac
}

ROOT_FOUND=0
for p in "${PARTS[@]}"; do
  # blkid reads the superblock directly (works without udev, and on a half-broken card).
  fstype="$(sudo blkid -p -o value -s TYPE "$p" 2>/dev/null || true)"
  label="$(sudo blkid -p -o value -s LABEL "$p" 2>/dev/null || true)"
  if [[ "$fstype" == vfat ]]; then
    log "Boot partition $p ($label)"
    sudo mkdir -p "$MNT_BOOT"
    if sudo mount -o ro "$p" "$MNT_BOOT"; then
      for f in config.txt cmdline.txt; do grab "$MNT_BOOT/$f" boot; done
      sudo umount "$MNT_BOOT"
    else
      warn "couldn't mount boot partition $p (not needed for the code)"
    fi
    continue
  fi

  log "Data partition $p (${fstype:-unknown type}, ${label:-no label})"
  sudo mkdir -p "$MNT_ROOT"
  if sudo mount -t ext4 -o ro,noload "$p" "$MNT_ROOT" 2>/dev/null; then
    info "mounted read-only"
  elif sudo mount -t ext4 -o ro,noload,sb=131072 "$p" "$MNT_ROOT" 2>/dev/null; then
    warn "primary superblock damaged — mounted from the backup superblock"
  else
    warn "couldn't mount $p read-only"
    continue
  fi
  ROOT_FOUND=1

  # Everything under each home directory that matters for the rebuild.
  for home in "$MNT_ROOT"/home/*; do
    sudo test -d "$home" || continue
    u="$(basename "$home")"
    info "home: $u"
    mkdir -p "$DEST/home_$u"
    grab "$home/Desktop/campus_bot_fw" "home_$u/Desktop"
    # Claude Code: full session transcripts (projects/**/*.jsonl) + pre-edit file snapshots (file-history/).
    grab "$home/.claude" "home_$u" --exclude '.credentials.json'
    grab "$home/.campus_bot" "home_$u" --chmod=D700,F600    # enrolled faces (encrypted) — private
    grab "$home/.bash_history" "home_$u"
    grab "$home/ros2_ws/src" "home_$u/ros2_ws"
    grab "$home/microros_ws/firmware" "home_$u/microros_ws"
    grab "$home/Arduino" "home_$u"
    grab "$home/Documents/PlatformIO" "home_$u/Documents"
    grab "$home/.config/Code/User" "home_$u/.config/Code" --exclude 'workspaceStorage/' --exclude 'globalStorage/'
    grab "$home/.xsessionrc" "home_$u"
    # Sweep: any other source/config file in the home dir, e.g. the face-enrolment service or
    # calibrate.py living outside the project folder.
    log "Sweeping $u's home for other source/config files"
    sudo find "$home" \
        \( -path "$home/Desktop/campus_bot_fw" -o -path "$home/.cache" -o -path "$home/.local" -o -path "$home/.venvs" -o -path "$home/snap" \
           -o -path "$home/.claude" -o -path "$home/.vscode" -o -path "$home/.vscode-server" \
           -o -path "$home/.platformio" -o -path "$home/.mozilla" -o -path "$home/.config/Code" \
           -o -name node_modules -o -name build -o -name install -o -name log -o -name __pycache__ \
           -o -name site-packages \) -prune -o \
        -type f -size -2M \( -name '*.py' -o -name '*.sh' -o -name '*.yaml' -o -name '*.yml' \
           -o -name '*.json' -o -name '*.launch' -o -name '*.xml' -o -name '*.rviz' -o -name '*.urdf' \
           -o -name '*.xacro' -o -name '*.ino' -o -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \
           -o -name '*.ini' -o -name '*.md' -o -name '*.txt' -o -name '*.csv' -o -name '*.pgm' -o -name '*.service' \
           -o -name 'CMakeLists.txt' -o -name 'Makefile' \) -printf '%P\0' 2>/dev/null \
      | sudo rsync -a --no-owner --no-group --from0 --files-from=- "$home/" "$DEST/home_$u/sweep/" 2>/dev/null \
      || warn "sweep partially failed"
    # Python venvs: record what was installed (name==version) so requirements can be rebuilt.
    while IFS= read -r -d '' cfg; do
      venv="$(dirname "$cfg")"
      out="$DEST/home_$u/venv_$(echo "${venv#"$home"/}" | tr '/' '_').txt"
      # glob inside sudo: the old home may not be readable by the current user
      sudo bash -c 'shopt -s nullglob; for d in "$1"/lib/python3*/site-packages/*.dist-info; do basename "$d"; done' _ "$venv" \
        | sed -E 's/\.dist-info$//; s/-([^-]+)$/==\1/' | sort > "$out" || true
      info "venv packages: ${venv#"$home"/} → $(basename "$out")"
    done < <(sudo find "$home" -maxdepth 4 -name pyvenv.cfg -print0 2>/dev/null)
  done

  # System config that the old setup changed.
  log "System configuration"
  mkdir -p "$DEST/system"
  grab "$MNT_ROOT/etc/udev/rules.d" system
  grab "$MNT_ROOT/etc/xrdp" system
  grab "$MNT_ROOT/etc/apt/sources.list.d" system
  sudo find "$MNT_ROOT/etc/systemd/system" -maxdepth 1 -name '*.service' -type f -exec cp {} "$DEST/system/" \; 2>/dev/null || true
  grab "$MNT_ROOT/var/log/apt/history.log" system
  if [[ -r "$MNT_ROOT/var/lib/dpkg/status" ]]; then
    awk '/^Package: /{p=$2} /^Status: install ok installed/{print p}' "$MNT_ROOT/var/lib/dpkg/status" | sort > "$DEST/system/installed-packages.txt"
    grep -E '^ros-' "$DEST/system/installed-packages.txt" > "$DEST/system/ros-packages.txt" || true
    info "✓ package list: $(wc -l < "$DEST/system/installed-packages.txt") installed, $(wc -l < "$DEST/system/ros-packages.txt") ROS"
  fi
  sudo umount "$MNT_ROOT"
done

sudo chown -R "$USER:$USER" "$DEST"

if (( ! ROOT_FOUND )); then
  cat <<EOF

Couldn't mount the data partition. Don't run repairs on the card itself. Instead:
  1. Image it (needs free space ≥ the card's size; use a USB drive, or do this on the Mac with 'brew install ddrescue'):
       sudo ddrescue -d -r3 $DISK /path/on/big/drive/oldsd.img /path/on/big/drive/oldsd.map
  2. Repair the COPY and re-run this script on it:
       LOOP=\$(sudo losetup -Pf --show /path/on/big/drive/oldsd.img)
       sudo e2fsck -fy \${LOOP}p2
       bash $0 \$LOOP
  3. If e2fsck can't save it, carve files out of the image (loses filenames, keeps content):
       photorec /path/on/big/drive/oldsd.img
EOF
  exit 2
fi

log "Recovered into $DEST"
( cd "$DEST" && du -sh -- * 2>/dev/null | sed 's/^/    /' )
n_jsonl="$(find "$DEST" -path '*/.claude/projects/*' -name '*.jsonl' 2>/dev/null | wc -l)"
n_hist="$(find "$DEST" -path '*/.claude/file-history/*' -type f 2>/dev/null | wc -l)"
proj="$(find "$DEST" -path '*/Desktop/campus_bot_fw' -type d 2>/dev/null | head -1)"
info "campus_bot_fw folder: ${proj:-NOT FOUND}"
info "Claude transcripts (.jsonl): $n_jsonl    file-history snapshots: $n_hist"
cat <<EOF

Next: cd ~/Desktop/campus_bot_fw && claude
      → "Recovered files are in $DEST. Read RECOVERY.md and rebuild from them."
Keep $DEST out of git: it can contain face data (.campus_bot) and credentials.
EOF
