# campus_bot_fw

Mapping and navigation for the Campus Bot office delivery robot (Raspberry Pi 5, ROS 2 Jazzy).
Robot details are in `CLAUDE.md`. The rebuild after the SD-card loss is tracked in `RECOVERY.md`.

The mapping stack below was **rebuilt on 2026-09-30 from the specs in RECOVERY.md**. It has been tested
end to end on the Pi with a simulated robot (`--sim`), but **not yet on the real hardware**. If the old
SD card turns up the original files, those replace these.

## Day-one hardware checklist

Do this in order the first time everything is reconnected. Steps 3, 5 and STOP are also desktop icons
(the "Campus Bot …" launchers; reinstall them with `bash setup/install_launchers.sh`).

1. **Power.** Pack at 22–30 V, never 3 packs in series. Pi on its 5 V buck. Then run
   `run/power_check.sh`: you want ≥ 4.85 V and `throttled=0x0`.
2. **Plug in** the lidar's CP2102 stick and the ESP32's **UART** USB-C port. **Back up the ESP32
   firmware before anything else touches it:** `bash setup/04_backup_esp32.sh`. It's read-only, and
   the strings it extracts show topic names and whether micro-ROS uses serial or WiFi.
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
   `~/.campus_bot/logs/calibrate_odom.csv`. Applying them means changing the firmware, and its source
   is still lost, so for now just record them. Mapping still works without them: heading comes from
   the gyro, and SLAM absorbs a few percent of distance error.
8. **Map the office:** `run/remap.sh office` (icon 2). Drive with the keys shown on screen. Press `q`
   when you're back at the start.

## Commands

| Command | What it does |
|---|---|
| `run/remap.sh <area> [--nav] [--no-rviz] [--yes]` | Power check → sensors → odom TF → pre-flight → SLAM + RViz → keyboard driving. `q` saves `maps/<area>` and `maps/<area>_clean`, a pose graph and a stats report. `--nav` then starts AMCL + Nav2 on the new map. |
| `run/remap.sh <area> --sim` | The same flow with a fake robot (no hardware), for testing changes. |
| `run/stop.sh` | Sends zero velocity, then stops everything, strays included. |
| `run/power_check.sh [--brief\|--watch]` | 5 V rail and undervoltage flags. `--watch` during a run shows dips. |
| `run/camera.sh [on\|off\|status]` | Mast camera. It's off while mapping. |
| `python3 tools/preflight.py` | The readiness sweep on its own. |
| `python3 tools/calibrate_mask.py` | Measures the column mask and writes `config/scan_mask.yaml`. |
| `python3 tools/lidar_front.py` | Checks `LASER_YAW`. |
| `python3 tools/clean_map.py maps/<area>.yaml` | Re-cleans a saved map. |
| `run/hw_check.sh [--apply]` | First contact: identifies the ports, tests the lidar, auto-detects the ESP32's port, baud and topics, and suggests `robot.env` values. |
| `python3 tools/calibrate_odom.py straight\|spin` | Wheel diameter and separation calibration. |
| `bash setup/install_launchers.sh` | Desktop and app-grid launchers: hardware check, map, STOP, power watch. |

Logs go to `~/.campus_bot/logs/<name>.log` (bringup, slam, nav, rviz, odom_tf, map_saver).

## Driving keys

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
tools/                    preflight, calibrate_mask, calibrate_odom, lidar_front, clean_map, pose_tool
run/                      the scripts above
maps/                     saved maps
```

## Still unknown (guesses in `config/robot.env`, marked VERIFY)

- The micro-ROS agent baud (115200 assumed), the IMU topic name, and whether the firmware takes
  `Twist` or `TwistStamped`. `run/hw_check.sh` detects all three.
- Whether the firmware broadcasts `odom → base_link` itself. `remap.sh` checks and starts
  `odom_to_tf.py` only if nothing else does.
- `LASER_YAW` (step 6) and the exact mask sectors (step 5).
- The original map-grading metric and cleaning steps. `clean_map.py` is a conservative stand-in that
  reports stats, not the original grade.
- The rest of the original slam_toolbox tuning. Only the three office-scale values are known.
