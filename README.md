# campus_bot_fw

Mapping and navigation for the Campus Bot office delivery robot (Raspberry Pi 5, ROS 2 Jazzy).
Robot details are in `CLAUDE.md`. The rebuild after the SD-card loss is tracked in `RECOVERY.md`.

The mapping stack below was **rebuilt on 2026-09-30 from the specs in RECOVERY.md**. It has been tested
end to end on the Pi with a simulated robot (`--sim`), but **not yet on the real hardware**. If the old
SD card turns up the original files, those replace these.

## Day-one hardware checklist

Do this in order the first time everything is reconnected. Hardware check, Map, Drive, Lidar view,
STOP and Power watch are also desktop icons (the "Campus Bot …" launchers; reinstall them with
`bash setup/install_launchers.sh`).

1. **Power.** Pack at 22–30 V, never 3 packs in series. Pi on its 5 V buck. Then run
   `run/power_check.sh`: you want ≥ 4.85 V and `throttled=0x0`.
2. **Plug in** the lidar's CP2102 stick and the ESP32's **native USB** port (it shows up as
   `/dev/esp32` → ttyACM0, Espressif 303a:1001; micro-ROS runs on this port). The UART USB-C port only
   carries the firmware's text log. The firmware was backed up on 2026-10-01
   (`~/recovered/esp32-20261001-113417/`). Run `bash setup/04_backup_esp32.sh` again before any reflash.
3. **Hardware check:** `run/hw_check.sh` (icon 1). It identifies both USB devices, tests the lidar on
   its own, finds the ESP32's micro-ROS **port and baud** by trying each (resetting the board between
   tries), lists the ESP32's topics and their types, and prints the `config/robot.env` values that
   need changing. `run/hw_check.sh --apply` writes them. Each run's report is saved in
   `~/.campus_bot/logs/hw_check-*.txt`.
4. **Try the whole flow** with the wheels off the ground or the motors unpowered:
   `run/remap.sh test`. Everything in the pre-flight should be ✓ except the column mask.
5. **Calibrate the column mask.** Park the robot ≥ 1 m from everything, with the sensors running
   (step 4 leaves them up):
   ```bash
   python3 tools/calibrate_mask.py      # expect 4 sectors, ~52 beams
   ```
6. **Check which way the lidar faces.** Put a box ~0.7 m straight ahead:
   ```bash
   python3 tools/lidar_front.py         # must read ~0°; a box on the left must read ~+90°
   ```
   If ahead doesn't read 0°, put the printed `LASER_YAW` into `config/robot.env` and re-run
   `run/stop.sh` and then `run/remap.sh`.
7. **Wheel calibration** (on the office floor, clear space):
   ```bash
   python3 tools/calibrate_odom.py straight --distance 5    # tape-measure the real distance
   python3 tools/calibrate_odom.py spin --turns 10          # count the real turns by a tape mark
   ```
   These print the correction factors for `WHEEL_DIA` and `WHEEL_SEP` and log them to
   `~/.campus_bot/logs/calibrate_odom.csv`. Applying them means editing `WHEEL_DIA` / `WHEEL_SEP` /
   `GYRO_SCALE` in `firmware/campus_bot_base/include/config.h` and reflashing. That source is a
   reconstruction (see `firmware/README.md`), so just record the values until it has been bench-tested.
   Mapping still works without them: heading comes from the gyro, and SLAM absorbs a few percent of
   distance error.
8. **Map the office:** `run/remap.sh office` (icon 2). Drive with the keys shown on screen. Press `q`
   when you're back at the start.

## Commands

| Command | What it does |
|---|---|
| `run/remap.sh <area> [--nav] [--no-rviz] [--yes]` | Power check → sensors → odom TF → pre-flight → SLAM + RViz → keyboard driving. `q` saves `maps/<area>` and `maps/<area>_clean`, a pose graph and a stats report. `--nav` then starts AMCL + Nav2 on the new map. |
| `run/remap.sh <area> --sim` | The same flow with a fake robot (no hardware), for testing changes. |
| `run/drive.sh [--speed 0.25] [--terminal]` | **Hold-to-drive** window (the terminal driver over plain SSH): the robot moves only while ↑↓←→ / WASD is held; let go (or click away) and it stops at ≤ 0.5 m/s². Starts the ESP32 agent itself. In remote desktop it stops if the link goes silent for 1 s; over plain SSH it uses the terminal driver. |
| `run/lidar_view.sh [--no-rviz]` | Live lidar: RViz (fixed frame `laser`, no TF needed) plus numbers in the terminal (rate, nearest obstacle, 8 sectors, ASCII top-down map). |
| `sudo bash setup/05_remote_access.sh` | One-time setup for running without a monitor (see Remote access). |
| `run/stop.sh` | Sends zero velocity, then stops everything, strays included. |
| `run/power_check.sh [--brief\|--watch]` | 5 V rail and undervoltage flags. `--watch` during a run shows dips. |
| `run/camera.sh [on\|off\|status]` | Mast camera. It's off while mapping. |
| `python3 tools/preflight.py` | The readiness sweep on its own. |
| `python3 tools/calibrate_mask.py` | Measures the column mask and writes `config/scan_mask.yaml`. |
| `python3 tools/lidar_front.py` | Checks `LASER_YAW`. |
| `python3 tools/clean_map.py maps/<area>.yaml` | Re-cleans a saved map. |
| `run/hw_check.sh [--apply]` | First contact: identifies the ports, tests the lidar, checks the ESP32's micro-ROS link and topics, and suggests `robot.env` values. |
| `python3 tools/esp32_reset.py` | Safe ESP32 reset that reports whether the firmware or the ROM loader booted. |
| `python3 tools/lidar_check.py [port]` | Lidar alone, no ROS: info, health, scan rate (works on the Mac too). |
| `python3 tools/calibrate_odom.py straight\|spin` | Wheel diameter and separation calibration. |
| `bash setup/install_launchers.sh` | Desktop and app-grid launchers: hardware check, map, STOP, power watch. |

Logs go to `~/.campus_bot/logs/<name>.log` (bringup, slam, nav, rviz, odom_tf, map_saver).

## Remote access (robot on WiFi, no monitor)

**Once, in a terminal on the Pi:** `sudo bash setup/05_remote_access.sh`. It's safe to re-run. It:
- turns WiFi power saving off (it causes lag spikes and dropouts on the Pi);
- makes the WiFi connect at boot and retry forever;
- adds SSH keepalives;
- stops the colour-profile password prompts in remote desktop;
- opens the firewall, if one is active.

**From the Mac, on the office WiFi:**
- **Terminal:** `ssh resp@resp-desktop.local`. If `.local` doesn't resolve, use the IP from `hostname -I`;
  it can change, so a DHCP reservation for the MAC in `/sys/class/net/wlan0/address` helps. Run long jobs
  inside `tmux new -A -s robot`, so a WiFi drop doesn't end a mapping run.
- **Screen:** Windows App → Add PC → `resp-desktop.local`, user `resp`. The remote desktop is its own
  **XFCE** desktop, separate from the GNOME one on the robot's monitor. A second GNOME for the same user
  crashes both ("Oh no! Something has gone wrong").
- **The robot's own GNOME screen (mirrored):** GNOME Desktop Sharing on **port 3390**. In Windows App,
  use PC name `resp-desktop.local:3390`. Set its login once, in a terminal on the Pi:
  `grdctl rdp set-credentials resp '<a password>'`. It only works while someone is logged in to GNOME on
  the monitor. With no monitor attached, GNOME needs an HDMI dummy plug to have a screen to share. To
  turn it off: `grdctl rdp disable`.
- **Code:** VS Code on the Mac → Remote-SSH → `resp@resp-desktop.local` → open `~/Desktop/campus_bot_fw`.
- **Driving:**
  - In remote desktop, `run/drive.sh` opens the hold-to-drive window. It **stops by itself if the link
    goes silent for 1 s**: otherwise a key held during a WiFi drop would never be released, and
    the window, which runs on the robot, would keep driving.
  - Over SSH, `run/drive.sh` uses the terminal driver. It stops 0.6 s after the last key, so a drop
    stops it too.
- **WiFi quality:** `iw dev wlan0 link` shows the signal. Before mapping far corners, check it stays
  above about −70 dBm along the route.

**Security:** SSH and remote desktop accept `resp`'s password from anyone on the office WiFi.
- Use a strong one: `passwd`.
- Prefer SSH keys: on the Mac, run `ssh-copy-id resp@resp-desktop.local`.

## When the ESP32 doesn't connect

`run/drive.sh`, `run/remap.sh` and `run/hw_check.sh` reset the ESP32 by themselves when it doesn't answer.
By hand: `run/stop.sh && python3 tools/esp32_reset.py`. It prints the boot mode:
- `SPI_FAST_FLASH_BOOT` = the firmware is running.
- `DOWNLOAD` = it's stuck in the ROM loader. Unplug its USB for 3 s, then plug it back in.

Only one program can use a serial port at a time: two of them split its data, and the ESP32 then
churns its session. The run scripts refuse to start if a port is already open, and name the program
holding it. Close that program, or run `run/stop.sh`.

Never reset it with a pyserial RTS pulse. On the S3's native USB that can arm download mode (see
RECOVERY.md, 2026-10-01).

## Driving keys

**Hold-to-drive window (`run/drive.sh`):** hold ↑ / W forward, ↓ / S back, ← / A and → / D to turn,
two keys to arc. The robot moves only while a key is held down. Letting go, clicking outside the window,
SPACE, Esc or Ctrl-C all stop it (ramped at ≤ 0.5 m/s², so ~0.5 s from mapping speed). `+` / `-`
change the speed.

**In the mapping terminal (`run/remap.sh`):**

Hold `i` / `,` for forward / back, `j` / `l` to turn, and `u o m .` for arcs. Letting go ramps the
robot to a stop at ≤ 0.5 m/s². `k` or space stops. `+`/`-` change drive speed and `]`/`[` change turn
speed. `q` finishes and saves; Ctrl-C quits and asks whether to save.

## Layout

```
config/robot.env          every tunable: ports, bauds, topics, TF offsets, speed limits
config/slam_toolbox.yaml  office-scale SLAM (12 m range, 8 m loop search, 12 m scan buffer)
config/scan_mask.yaml     column mask (calibrate on the robot)
config/nav2_params.yaml   AMCL + Nav2 for this robot (base_link, 0.5 m/s², Regulated Pure Pursuit)
config/campus_bot.rviz    RViz layout
launch/                   bringup (lidar, agent, filter, static TFs) and slam
nodes/                    scan_filter, odom_to_tf, teleop_keys, sim_robot
tools/                    preflight, calibrate_mask, calibrate_odom, lidar_front, lidar_check, clean_map, pose_tool
firmware/campus_bot_base/ ESP32-S3 micro-ROS base (PlatformIO), RECONSTRUCTED, not flashed yet
face/                     face enrolment service, 1:1 verification, threshold calibration
run/                      the scripts above
maps/                     saved maps
```

## Still unknown (guesses in `config/robot.env`, marked VERIFY)

- ~~Agent baud, IMU topic, Twist vs TwistStamped, odom TF~~: confirmed on 2026-10-01 against the real
  firmware: 115200 on ttyACM0, `/imu/data_raw`, `Twist`, and no TF from the firmware (`odom_to_tf.py`
  provides it).
- The firmware's bumper, IMU and HX711 pin numbers and its calibration constants (`firmware/`, VERIFY).
- `LASER_YAW` (step 6) and the exact mask sectors (step 5).
- The original map-grading metric and cleaning steps. `clean_map.py` is a conservative stand-in that
  reports stats, not the original grade.
- The rest of the original slam_toolbox tuning. Only the three office-scale values are known.
