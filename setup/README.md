# Campus Bot — fresh Raspberry Pi 5 setup

Do these steps in order. Every script is safe to re-run.

## 0. Flash the new card
- On the Mac, use Raspberry Pi Imager to flash **Ubuntu Desktop 24.04 LTS (64-bit)** for Raspberry Pi
  onto a **new** SD card. Keep the corrupted card untouched: don't format it, and don't plug it in yet.
- Boot the Pi with a monitor and keyboard. In the first-boot setup, use **username `resp`** and
  **computer name `resp-desktop`**, so the old paths and `resp-desktop.local` still work.
- Connect to the office WiFi. Then open Settings → Wi-Fi → ⚙ on the network, tick **Connect
  automatically** and **Available to all users**. Without that the Pi won't come back online headless
  after a reboot.

## 1. Get this folder onto the Pi
- On the Pi, open Firefox → claude.ai → this chat → download `campus_bot_bootstrap.tar.gz`.
  Or copy it over from the Mac with a USB stick.
- Then:
```bash
cd ~/Downloads && tar xzf campus_bot_bootstrap.tar.gz && cd campus_bot_bootstrap
```

## 2. VS Code + Claude Code (~5 min)
```bash
bash 01_install_dev_tools.sh
claude          # log in to your Claude account in the browser that opens, then /exit
```

## 3. Everything else (45–90 min; asks for your password once)
```bash
bash ~/Desktop/campus_bot_fw/setup/02_setup_pi.sh --hostname resp-desktop
sudo reboot
```
- Add `--usb-max-current` if the lidar or webcam drop out. This raises the Pi 5's USB power limit from
  600 mA to 1.6 A, and needs a 5 V buck that can deliver ~5 A.
- Run it from the Pi's own desktop terminal, not over SSH, so it can turn off USB automount before
  step 4.

## 4. Pull what's left off the old card (read-only)
Plug the old card in with a USB card reader **after** the Pi has booted, then:
```bash
bash ~/Desktop/campus_bot_fw/setup/03_recover_old_sd.sh
```

## 5. Back up the ESP32's firmware before anything reflashes it
Plug in the ESP32's UART USB-C port, then:
```bash
bash ~/Desktop/campus_bot_fw/setup/04_backup_esp32.sh
```

## 6. Hand over to Claude on the Pi
```bash
cd ~/Desktop/campus_bot_fw && claude
```
Say: *"Read RECOVERY.md and continue the rebuild."* `CLAUDE.md` briefs it on the robot automatically.
To drive that session from the Mac or phone, run `claude remote-control` instead.

## Remote desktop from the Mac
Windows App → Add PC → `resp-desktop.local` → log in as `resp`.
Log out of the desktop on the Pi's monitor first. If the same user is logged in on both, xrdp shows a
black screen.
