# RECOVERY.md — rebuilding campus_bot_fw after the SD-card loss (2026-09-29)

## What was lost
Everything on the Pi's SD card:
- `~/Desktop/campus_bot_fw`: scripts, config, maps, Python nodes, possibly the firmware source
- `~/.campus_bot/`: the enrolled faces
- both ROS workspaces
- the system configuration
- `~/.claude/`: the Claude Code transcripts

Nothing had been pushed to GitHub.

## Where the originals may still exist: work down this list before reconstructing anything
1. **The old SD card.** `setup/03_recover_old_sd.sh` copies it read-only into `~/recovered/<timestamp>/`.
   Check these, in order:
   - `home_resp/Desktop/campus_bot_fw/`: the project itself, if the filesystem mounts.
   - `home_resp/.claude/projects/**/*.jsonl`: the complete transcripts of the last sessions. Every
     `Write` tool call there contains a full file, and every `Edit` contains the before/after text.
     Replay them in order per file to rebuild the final contents. Write a small extractor after
     inspecting the real JSONL format; don't guess it.
   - `home_resp/.claude/file-history/`: Claude Code's snapshots of files before each edit.
   - `home_resp/sweep/`: every other source or config file found in the home directory.
   - `system/ros-packages.txt` and `system/installed-packages.txt`: exactly what was installed.
     Diff these against `setup/02_setup_pi.sh`.
   - `boot/config.txt`: shows whether `usb_max_current_enable` was set.
   - `home_resp/venv_*.txt`: the pip packages in each old venv.
2. **The last Claude Code session**, "Wheel 360 rotation incomplete", Sept 1–29, run in VS Code on the
   Pi via Remote Control:
   https://claude.ai/code/session_01HWhm2Dxuba1ztrmYwm83Q1
   Hemant can open it in a browser. The expandable tool-call blocks hold full file contents. Ask him
   to paste the blocks for specific files.
3. **The ESP32-S3's flash.** The firmware is still on the chip. `setup/04_backup_esp32.sh` dumps it and
   extracts its strings (topic names and log text help rewrite the source).
4. **claude.ai chats.** These hold the ESP32 differential-drive firmware (written "in another chat"),
   the stepper test sketches, `lidar_check.py`, `calibrate.py`, and blueprint Sheets 1–10 plus the
   deck-assembly card.
5. **Hemant's Mac.** Check the Arduino and PlatformIO folders, and wherever the face-enrolment service ran.

## What the transcripts prove about each file (the spec for reconstruction)
Numbers are exact where the last session reported them. Everything else is marked as inference.

### `run/remap.sh <area> [--nav]`
- **Bring-up:** sensors (lidar driver, `scan_filter.py`, micro-ROS agent, static TFs), then the same
  pre-flight as the readiness sweep below. Then it opens RViz, runs slam_toolbox with
  `config/slam_toolbox.yaml`, and gives the terminal to keyboard teleop.
- **On `q`:** saves `maps/<area>` plus a cleaned `maps/<area>_clean`, then **grades** the map. The
  grading metric is unknown: recover it, don't invent it.
- **`--nav`:** continues straight into autonomy (AMCL + Nav2) on the new map.
- It is idempotent: after a reboot it re-checks everything, so there is nothing to redo by hand.

### Readiness sweep / pre-flight: the all-green state from the last session
| Check | Expected |
|---|---|
| Lidar `/scan` | live on `/dev/ttyUSB0` |
| Column mask | **52.0 beams masked, 0 leaks** (columns fully covered) |
| IMU | a real gyro: live noise, not the synthetic placeholder |
| Heading source | gyro (survives carpet turn-slip) |
| `/odom` + micro-ROS agent | up |
| TF tree | `odom → base_link → laser/imu/camera` all resolving; `base_link → laser` at **0.215 m**; **6 static publishers** |
| Camera | off while mapping (frees CPU and WiFi) |
| Power | 5 V clean, no undervoltage |

### `run/stop.sh`
Sends the motors a zero-velocity command, then tears down the TF tree, odom, RViz, the agent and the
lidar. **Known bug in the original:** it missed `scan_filter.py` and three static TF publishers. The
rebuild must stop those too.

### `run/camera.sh`
Turns the camera back on after mapping.

### `run/power_check.sh`
Reports 5 V rail dips and undervoltage. Inference: it most likely used `vcgencmd get_throttled` and
`vcgencmd pmic_read_adc` (EXT5V_V) on the Pi 5.

### `config/slam_toolbox.yaml`
The office-scale retune changed three values (diff +4/−3):
- laser range 8 → **12 m**
- loop-closure search 5 → **8 m**
- scan buffer 10 → **12 m**

Inference: these are the parameters `max_laser_range`, `loop_search_maximum_distance` and
`scan_buffer_maximum_scan_distance`. The rest was cabin-scale tuning of unknown values; recover it if
possible.

### `scan_filter.py`
Masks the 4 column slivers (the 2020 columns cross the scan plane at ~215 mm): **52 beams in total**.
It publishes the masked scan that slam_toolbox consumes.

### ESP32 firmware (micro-ROS)
- **Inputs:** velocity commands from the Pi.
- **Outputs:** `/odom`, plus IMU data from the MPU6050. It has a synthetic-IMU placeholder mode used
  when the real gyro isn't delivering.
- **Safety:** 300 ms command watchdog; stop when the bumper's NC loop opens.
- **Config:** `WHEEL_DIA`, `WHEEL_SEP`, `INVERT_LEFT`/`INVERT_RIGHT`.
- **Pins:** IO4/IO7 (motor A), IO8/IO9 (motor B).
- **Motion:** acceleration capped at ≤ 0.5 m/s².
- **Heading:** gyro-based. Carpet turns are only ~60% efficient.
- The old session's title, "Wheel 360 rotation incomplete", probably refers to this turn-slip and
  wheel-separation calibration work.

### Face system
- **Enrolment service:** FastAPI, exposed over HTTPS with ngrok; multi-angle capture on a phone;
  InsightFace buffalo_s (ArcFace). It stores embeddings only, never images, Fernet-encrypted at
  `~/.campus_bot/faces.enc`. Enrolment is opt-in and there is a delete endpoint.
- **Enrolled:** 3 colleagues. They must re-enrol unless the card returns both `faces.enc` and its key.
- **`calibrate.py`:** collects genuine and impostor frames in desk and mast sessions, writes a CSV,
  then runs a TAR/FAR sweep, timing stats and an angle-effect check. It has a `--det-size` option.

**Calibration findings to carry forward:**
- The mast angle doesn't hurt: median scores were 0.804 (mast) vs 0.797 (desk). Phone enrolment is
  therefore fine, and enrolment can scale to 15+ people.
- **Unresolved:** frames labelled genuine for `lakshaysharma` scored ~0.18, against ~0.80 for Hemant's
  genuine frames. Either the label is wrong (someone else stood in, which makes it useful impostor data)
  or Lakshay's enrolment is broken. Ask Hemant which.
- One genuine frame scored 0.296, below the 0.42 placeholder threshold. Use best-of-N and never trust a
  single frame.
- Timing: the median is fine, but p95 is ~900 ms and the worst single frame took over 2 s, which blows
  the < 2 s verify budget. Test `--det-size 320`.
- The sweep couldn't run: there were 120 genuine frames and 0 impostor frames, and it needs both.

## Mapping plan for the office (from Hemant's photo)
- The glass meeting room ("Inboxkit") is the #1 risk. The lidar sees through glass. Drive slowly
  0.5–1 m off it, put matte tape on the glass at scan height (~215 mm), and check afterwards that it
  reads as a wall.
- Do the perimeter first, then the desk aisles and the café corner. Finish where you started so the loop
  closes.
- The planters (solid black cabinets) give clean returns. Avoid overhanging leaves that droop below 21.5 cm.
- People and rolling chairs leave ghosts. Map when the floor is quiet.
- Turn slowly and gently.
- Keep the camera off while mapping.
- Watch the pack voltage.

## Where things stood at the loss
- All sensors were green (see the table above).
- SLAM had been retuned office-scale.
- The motors were unpowered while the battery charged.
- xrdp was set up for a headless mapping run.
- The next step was the whole-office map.

## Rebuild status
- **2026-09-30:** The mapping stack was reconstructed from the spec above, before any recovery (the
  old card and the hardware weren't available). This covers `run/remap.sh`, `stop.sh`, `camera.sh`,
  `power_check.sh`, `scan_filter.py`, `config/slam_toolbox.yaml`, a Nav2 config, and the preflight,
  mask-calibration and map-cleaning tools. It was tested end to end on the Pi with `nodes/sim_robot.py`
  (map saved, and a Nav2 goal reached), but has **not** run on the robot. Hardware bring-up steps are
  in `README.md`. Recovered originals still replace these files.
- **Not reconstructed:** ESP32 firmware, face stack, `calibrate.py`, `lidar_check.py`.
- **2026-10-01, first day on the hardware:**
  - **Lidar** (CP2102 stick → `/dev/lidar` = ttyUSB0): it survived the 25.6 V incident. Model 0x41, fw 1.02,
    hw 18, health good, sllidar_ros2 scans at 9.97 Hz. `tools/lidar_check.py` was rebuilt.
  - **ESP32 flash backed up** to `~/recovered/esp32-20261001-113417/` (16 MB plus sha256, never commit).
    The apt esptool 4.7 can't read an S3 (its stub file is missing), so `04_backup_esp32.sh` now uses
    `~/.venvs/esptool` (pip esptool 5.4).
  - **The original firmware is alive.** It talks micro-ROS on the **native USB port** (`/dev/esp32` =
    ttyACM0, 303a:1001) at 115200, and the agent session comes up first try. Node `campus_bot_base`:
    `/odom` (Odometry, `odom`→`base_link`, 20 Hz), `/imu/data_raw` (Imu, `imu_link`, 20 Hz, real gyro:
    σ 0.0007 rad/s, gravity 9.84), `/bumper_tripped` (Bool, False: loop closed), `/parcel/weight_g`
    (Float32, no messages, so the HX711 is absent or unwired), and it subscribes `/cmd_vel` (Twist).
    It does **not** broadcast the odom TF (`odom_to_tf.py` covers that). `config/robot.env` needed no
    changes.
  - **What the dump proves about the firmware source:** PlatformIO + arduino-esp32 2.x (IDF v4.4.7),
    board `adafruit_feather_esp32s3`, FastAccelStepper (`stepperConnectToPin`), raw-register MPU6050
    and a bit-banged HX711 (no library strings), log on UART0 with these messages: `[campus_bot] boot / waiting
    for agent / agent connected / agent lost - stopping / publishes failing - reconnecting / alive heap=…`,
    `[drive] ready: %.1f steps/m, acc %u steps/s^2`, `[imu] ok, gyro bias …`, `[hx711] ok, tare offset …`,
    `[bumper] %s`. Partitions: default 8 MB (app0/app1 3.2 MB, spiffs, coredump).
  - **Reconstructed** `firmware/campus_bot_base/` from that spec. It **compiles**, with no warnings in its own
    code. The image is 446 KB against the original's 416 KB, both with 5 segments, and the flash
    layout matches the original's (DIO, 80 MHz, 8 MB, `default_8MB.csv`). Build it with
    `firmware/build.sh`: micro-ROS has to be built outside the ROS environment, and with PlatformIO
    installed by pipx it needs the `~/.platformio/penv` link. It has **not been flashed**:
    the chip still runs the original. Pin numbers for the bumper, IMU and HX711 and the numeric constants
    are guesses marked VERIFY. Read the real ones from the old firmware's UART0 boot log before flashing.
  - **Reconstructed** `face/` (enrolment service, store, verify, calibrate.py). The 3 old enrolments are gone
    with the card, so people must re-enrol.
  - **Camera:** the Brio 100 enumerates (UVC, MJPG). All four Pi USB ports were in use, so it was
    swapped out for the ESP32 during the check.
  - **Incident, same day:** the first `run/drive.sh` "reset" left the ESP32 in its **ROM download loader**
    ("waiting for download"), so it went silent: no session, no `/odom`. On the S3's native USB, DTR/RTS
    are commands. DTR=1 + RTS=0 arms download mode, DTR=0 + RTS=1 resets, and 0/0 disarms. This port is
    `-hupcl`, so the armed flag outlived the script, and the next reset booted the loader. Fixed with
    `tools/esp32_reset.py`, which sets both lines in one ioctl and reads the ROM banner to confirm
    `SPI_FAST_FLASH_BOOT`. **Never pulse RTS with pyserial on this port.** `drive.sh`, `remap.sh` and
    `hw_check.sh` now use it.
  - **Measured link behaviour (original firmware):**
    - An idle firmware connects to a new agent in < 1 s, and reconnects by itself when the agent restarts.
    - After a reset it boots in ~3 s.
    - The session churn seen at 12:19 (a new session every second, no topics for ~10 s) was almost
      certainly **two agents on one port**: a `drive.sh` run overlapped `hw_check.sh`'s agent, and the
      two split the incoming bytes. With a single agent it connected within 1 s every time. Every run
      script now refuses to open a port that another program holds, and names that program.
    - The baud rate is ignored on native USB.
    - QoS: every publisher is RELIABLE, the `cmd_vel` subscription is BEST_EFFORT.
  - **Verified end to end on the hardware:** `run/hw_check.sh`, `run/drive.sh` (including recovery from the
    loader), `run/lidar_view.sh`, `run/camera.sh` (30 Hz `/image_raw`), `run/stop.sh` and `run/remap.sh`.
    The full mapping flow ran with the robot parked: pre-flight READY, map + pose graph saved. The face
    engine ran on live Brio frames (detector 75 ms at det-size 640, 15 ms at 320).
  - **Column mask** (dry run, robot not in open space yet): 4 columns at about −120°, −57°, +59° and
    +128° in the laser frame. Columns at a 350×250 deck's corners would sit near ±35° and ±145° if the
    lidar's 0° pointed forward, so **the lidar may be mounted rotated ~90°**. Check `LASER_YAW` with
    `tools/lidar_front.py` before mapping.

## Plan
1. Recover: steps 1–5 above.
2. Rebuild `campus_bot_fw` from recovered originals, filling gaps from the spec above. Mark every
   reconstructed file.
3. Create a **private** GitHub repo (Hemant to confirm the name; suggested `hk2166/campus_bot_fw`),
   then push, and push after every session from then on.
4. Re-run the readiness sweep, then map the office.
5. AMCL and Nav2.
6. Face calibration: settle the Lakshay question, collect impostor data, test `--det-size 320`.
7. Fleet server and the open-box state machine (verify → announce → load-cell weight drop = delivered),
   E2E tests, enrolment to 15+ people, then the demo.
