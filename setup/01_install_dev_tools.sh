#!/usr/bin/env bash
# 01_install_dev_tools.sh — VS Code + Claude Code (CLI and VS Code extension) on the Raspberry Pi 5.
#
# Run from a terminal on the Pi's desktop, as your normal user (not with sudo):
#     bash 01_install_dev_tools.sh
# Afterwards: run `claude` once to log in, then open ~/Desktop/campus_bot_fw in VS Code.
# Safe to re-run.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$HOME/Desktop/campus_bot_fw"

log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31m[fail] %s\033[0m\n' "$*" >&2; exit 1; }
trap 'die "line $LINENO: \"$BASH_COMMAND\" failed. Fix and re-run; finished steps are skipped."' ERR

apt_get() { sudo env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l apt-get -o DPkg::Lock::Timeout=900 "$@"; }

[[ $EUID -ne 0 ]] || die "Run as your normal user (e.g. resp), not root/sudo. The script calls sudo itself."
[[ "$(uname -m)" == aarch64 ]] || warn "Expected aarch64 (Raspberry Pi 5); this is $(uname -m). Continuing."
sudo -v

log "Base tools"
apt_get update
apt_get install -y wget gpg curl git apt-transport-https

log "VS Code (Microsoft apt repo; it carries arm64 builds)"
if command -v code >/dev/null; then
  echo "Already installed: VS Code $(code --version | head -1)"
else
  key="$(mktemp)"
  wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > "$key"
  sudo install -D -o root -g root -m 644 "$key" /usr/share/keyrings/microsoft.gpg
  rm -f "$key"
  sudo tee /etc/apt/sources.list.d/vscode.sources >/dev/null <<'EOF'
Types: deb
URIs: https://packages.microsoft.com/repos/code
Suites: stable
Components: main
Architectures: amd64,arm64,armhf
Signed-By: /usr/share/keyrings/microsoft.gpg
EOF
  # We already added the repo; stop the package's installer from adding a second copy.
  echo "code code/add-microsoft-repo boolean false" | sudo debconf-set-selections
  apt_get update
  apt_get install -y code
fi
# Two entries for the same repo with different keys make every `apt update` fail.
if [[ -f /etc/apt/sources.list.d/vscode.list && -f /etc/apt/sources.list.d/vscode.sources ]]; then
  warn "Removing duplicate /etc/apt/sources.list.d/vscode.list"
  sudo rm -f /etc/apt/sources.list.d/vscode.list
fi

log "Claude Code CLI (native installer; picks the arm64 build)"
export PATH="$HOME/.local/bin:$PATH"
if command -v claude >/dev/null; then
  echo "Already installed: $(claude --version 2>/dev/null || echo claude)"
else
  curl -fsSL https://claude.ai/install.sh | bash
fi
grep -qs 'HOME/.local/bin' "$HOME/.bashrc" || echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
command -v claude >/dev/null || die "claude not on PATH after install — open a new terminal and run: claude --version"

log "VS Code extensions"
code --install-extension anthropic.claude-code --force || warn "Claude Code extension failed — install it from the Extensions panel (search 'Claude Code')."
code --install-extension ms-python.python || warn "Python extension failed (optional)."

log "Project folder: $PROJECT_DIR"
mkdir -p "$PROJECT_DIR/setup"
for f in CLAUDE.md RECOVERY.md; do
  if [[ -f "$SCRIPT_DIR/$f" && ! -f "$PROJECT_DIR/$f" ]]; then cp "$SCRIPT_DIR/$f" "$PROJECT_DIR/$f"; echo "  + $f"; fi
done
if [[ ! "$SCRIPT_DIR" -ef "$PROJECT_DIR/setup" ]]; then
  for f in "$SCRIPT_DIR"/0*.sh "$SCRIPT_DIR/README.md"; do
    if [[ -f "$f" ]]; then cp -f "$f" "$PROJECT_DIR/setup/"; echo "  + setup/$(basename "$f")"; fi
  done
fi

cat <<EOF

Done.
  1. Log in:        claude            (opens a browser; sign in to your Claude account, then /exit)
  2. Full setup:    bash "$PROJECT_DIR/setup/02_setup_pi.sh"     (asks for your sudo password; 45–90 min)
  3. Then work:     code "$PROJECT_DIR"   → Claude Code panel, or: cd "$PROJECT_DIR" && claude
     Drive it from your Mac/phone:  cd "$PROJECT_DIR" && claude remote-control
EOF
