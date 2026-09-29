# campus_bot_fw

Mapping and navigation for the Campus Bot office delivery robot (Raspberry Pi 5, ROS 2 Jazzy).
Robot details are in `CLAUDE.md`. The rebuild after the SD-card loss is tracked in `RECOVERY.md`.

The mapping stack below was **rebuilt on 2026-09-30 from the specs in RECOVERY.md**. It has been tested
end to end on the Pi with a simulated robot (`--sim`), but **not yet on the real hardware**. If the old
SD card turns up the original files, those replace these.

## Day-one hardware checklist

Do this in order the first time everything is reconnected.

1. **Power.** Pack at 22–30 V, never 3 packs in series. Pi on its 5 V buck. Then run
   `run/power_check.sh`: you want ≥ 4.85 V and `throttled=0x0`.
2. **Plug in** the lidar's CP2102 stick and the ESP32's **UART** USB-C port, then check the names:
   ```bash
   ls -l /dev/lidar /dev/esp32 /dev/ttyUSB* /dev/ttyACM*
   ```
   If `/dev/lidar` or `/dev/esp32` is missing, or both point at the same port (both bridges are
   CP2102), either set `LIDAR_PORT` / `ESP32_PORT` in `config/robot.env` or pin them by serial number
   in `/etc/udev/rules.d/99-campus-bot.rules`.
3. **Back up the ESP32 firmware before anything reflashes it:** `bash setup/04_backup_esp32.sh`.
4. **Try the whole flow** with the wheels off the ground or the motors unpowered:
   ```bash
   run/remap.sh test
   ```
   If `/odom` doesn't appear, press the ESP32's RST button and re-run. If it still doesn't appear,
   `AGENT_BAUD` in `config/robot.env` doesn't match the firmware. Check the strings from the step 3
   backup.
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
7. **Confirm the ESP32's topics.** Run `ros2 topic list -t`. Compare `CMD_VEL_TOPIC`, `ODOM_TOPIC` and
   `IMU_TOPIC` in `config/robot.env` (the pre-flight also auto-finds odom and IMU topics by type).
8. **Map the office:** `run/remap.sh office`. Drive with the keys shown on screen. Press `q` when you're
   back at the start.

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
tools/                    preflight, calibrate_mask, lidar_front, clean_map, pose_tool
run/                      the scripts above
maps/                     saved maps
```

## Still unknown (guesses in `config/robot.env`, marked VERIFY)

- The micro-ROS agent baud (115200 assumed), the IMU topic name, and whether the firmware takes
  `Twist` or `TwistStamped` (the pre-flight prints the type).
- Whether the firmware broadcasts `odom → base_link` itself. `remap.sh` checks and starts
  `odom_to_tf.py` only if nothing else does.
- `LASER_YAW` (step 6) and the exact mask sectors (step 5).
- The original map-grading metric and cleaning steps. `clean_map.py` is a conservative stand-in that
  reports stats, not the original grade.
- The rest of the original slam_toolbox tuning. Only the three office-scale values are known.
