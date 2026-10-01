# CLAUDE.md — Campus Bot (office delivery robot)

Hemant's team is building an autonomous indoor delivery robot for their Gurugram office floor. It carries
snacks and parcels of up to 5 kg, navigates with lidar SLAM plus Nav2, and confirms the recipient by face
verification at handover (OTP as fallback). The budget is ₹40–80k. The mid-September demo date has
passed, so the schedule is "finish ASAP".

**This repo was lost on 2026-09-29.** The Pi's SD card corrupted, and there was no git remote or backup.
It is being rebuilt now. **Read `RECOVERY.md` before writing any code**: a recovered original always
beats a reconstruction.

## Hard rules
- **Commit and push to GitHub at the end of every working session.** Skipping this is how everything was lost.
- Recovered originals beat reconstruction. Anything rewritten from transcripts or memory gets a
  `# RECONSTRUCTED from <source>` header until it has run on the robot.
- Never commit face data (`~/.campus_bot/`, `faces.enc`, the Fernet key), ngrok tokens, WiFi or
  remote-desktop passwords, `~/.claude/.credentials.json`, ESP32 flash dumps (`*.bin`, which can contain
  WiFi credentials), or anything under `~/recovered/`.
- Don't reflash the ESP32 until `setup/04_backup_esp32.sh` has saved its current firmware.
- Motion limits: acceleration ≤ 0.5 m/s², a hard requirement for tower stability. Top speed is about
  0.8 m/s. Every drive path needs the 300 ms command watchdog and the bumper stop.
- Power: never put 3 packs in series. See the Power section before changing anything electrical.

## Platform
- Raspberry Pi 5 8 GB running Ubuntu 24.04 Desktop (arm64) and ROS 2 Jazzy. User `resp`, host
  `resp-desktop` (`resp-desktop.local`), office WiFi `Enrich-Labs`.
- Setup is scripted in `setup/02_setup_pi.sh`. Workspaces: `~/ros2_ws` (sllidar_ros2) and
  `~/microros_ws` (micro-ROS agent).
- Headless access: xrdp on port 3389, plus SSH. `setup/05_remote_access.sh` (run once with sudo) makes it
  WiFi-robust (power save off, endless reconnects, SSH keepalives). xrdp gives a black screen if `resp` is logged in on the
  physical monitor at the same time. Log out there first, or reboot and connect without touching the
  monitor.
- For a new session on the Pi, run `claude` in this folder, or `claude remote-control` to drive it from
  the Mac or phone app. The Pi session can't type your sudo password, so run sudo-heavy scripts yourself
  in a terminal.

## Hardware as built
- **Drive:** 2 of 3 NEMA23 steppers (19 kg·cm, Robokits RMCS-1002, 8 mm shaft, 2.8 A). Each goes through
  a DM542 driver (DIP set to 1600 pulses/rev, 2.84 A peak, SW4 OFF = half-current idle), an EasyMech 8 mm
  hub (47.5 mm PCD) and a 100 mm disc wheel. Motors are slung under the plate with wheels outboard and the
  axle at plate centre (X = 200). Diamond layout with centre pivot. Casters (2") front and rear are shimmed
  0.5 mm short of the drive wheels.
  - Wheel separation is ~340–360 mm nominal. Calibrate it: drive 5 m straight to set the wheel-diameter
    scale, then spin 10 turns to set the separation.
  - Motor mirror direction is unknown, handled by the `INVERT_LEFT`/`INVERT_RIGHT` flags. ENA is unwired (v1).
- **ESP32-S3 (Edgebax S3-PRO):**
  - Motor A: IO4 → PUL−, IO7 → DIR−, with ESP32 5V → PUL+ and DIR+ (common anode). Bench-proven.
    IO5/IO6 do not work on this board.
  - Motor B: IO8/IO9 (reserved).
  - Also free: IO10–18, 35–42, 47, 48, and the SDA/SCL header. Avoid IO0, IO3, IO45 and IO46.
  - Flash through the UART USB-C port.
- **Lidar:** Slamtec RPLIDAR C1 at **460800 baud (not 115200)**, via a CP2102 USB-TTL stick. The
  original adapter burned out when it was fed 25.6 V. Wiring: Red → 5V, Black → GND, Yellow (lidar TX) →
  RXD, Green (lidar RX) → TXD. The scan plane is at ~215 mm, and 4 column slivers are masked.
- **Sensors:**
  - MPU6050 IMU on the ESP32, used as the heading source.
  - Logitech Brio 100 on a mast at ~1.25 m, tilted up ~10°, with a 12 V warm-white LED strip as fill light.
  - 5 kg bar load cell + HX711 under a floating crate floor: a weight drop means the parcel was delivered.
  - 6× Omron V-156 microswitches on the front-only bumper, wired NC in series (open circuit = stop).
- **Structure:** 400×300×12 mm ply base; all electricals on deck 1 (≤ ~120 mm tall) under an acrylic
  cover at ~175–180 mm, with the lidar on the cover; 4× 500 mm 2020 columns up to a platform at ~50 cm
  holding an open crate (no lock: face verification is the delivery confirmation). There is no e-stop yet;
  a top master switch (20 A+) is still to buy.

## Power
- 2× Pro-Range 12.8 V 12 Ah LiFePO₄ packs in series: 25.6 V nominal, 29.2 V full. The series link joins
  Batt1 black to Batt2 red. A 10 A mini-blade fuse protects the main line. Two XL4015 bucks give 5 V
  (Pi) and 12 V (LED strip).
- The system window is 22–30 V. The hard ceiling is ~32 V, set by the XL4015s and the fuse holder,
  which is why 3 packs in series is never allowed.
- Charge one pack at a time with the 14.6 V charger, disconnected from the robot.
- The last session ran with "no BMS: stop at ~19.8 V". **⚠ Unconfirmed:** 19.8 V is below 8 × 2.5 V =
  20 V, the usual LiFePO₄ absolute minimum. Treat 22 V as the stop point until Hemant confirms.
- The Pi browns out if the pack sags, which kills a mapping run. `run/power_check.sh` watches the 5 V rail.

## Software (see RECOVERY.md for the per-file detail)
- ESP32 (micro-ROS over serial) ↔ `micro_ros_agent` on the Pi. The ESP32 publishes `/odom` and IMU data
  and takes velocity commands.
- `sllidar_ros2` publishes `/scan`, and `scan_filter.py` masks the columns. slam_toolbox then maps using
  `config/slam_toolbox.yaml` (tuned office-scale).
- Static TFs: `base_link → laser` (z 0.215 m), plus `imu` and `camera`.
- `run/remap.sh <area> [--nav]` does the whole mapping flow. Press `q` to save `maps/<area>` and
  `maps/<area>_clean`, then grade the map.
- Face stack: FastAPI enrolment service, InsightFace buffalo_s (ArcFace), 1:1 verification with
  best-of-N frames, threshold 0.42 (a placeholder pending calibration), and `calibrate.py`.
