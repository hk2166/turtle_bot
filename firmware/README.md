# firmware/campus_bot_base: ESP32-S3 drive base

**RECONSTRUCTED on 2026-10-01 from the strings in the flash dump. It compiles but has not run.** The
chip still runs the original firmware, which works: micro-ROS session, `/odom`, `/imu/data_raw` and
`/bumper_tripped` were all verified live that day. Only flash this source when you need a change,
such as new calibration constants. Before that:

1. **Back up the chip again:** `bash setup/04_backup_esp32.sh`. That gives a byte-exact restore point.
2. **Read the old firmware's real pins and constants.** Plug in the board's UART USB-C port (it appears
   as a second ttyUSB/ttyACM), run `pio device monitor -p <that port> -b 115200`, then press RESET.
   Copy the numbers from `[imu] … SDA=… SCL=…`, `[hx711] … DT=… SCK=…` and
   `[drive] ready: … steps/m, acc …` into `include/config.h`. Everything marked VERIFY there is a guess.
3. Flash with the **wheels off the ground**: `pio run -t upload --upload-port /dev/esp32`. Run
   `run/hw_check.sh` again: the topics and rates must match what the original gave.
4. Test the safety paths before the robot touches the floor:
   - Stop sending `cmd_vel`: the wheels must ramp to a stop within 300 ms plus the ramp.
   - Open the bumper loop: it must stop, reject forward motion, and still allow reversing.
   - Kill the agent: you should see `agent lost - stopping`.

To roll back: `~/.venvs/esptool/bin/esptool --chip esp32s3 -p /dev/esp32 write-flash 0 ~/recovered/esp32-20261001-113417/esp32s3_full_flash_16MB.bin`

## Interface (matches the original)

| Topic | Type | Notes |
|---|---|---|
| `cmd_vel` (sub) | `geometry_msgs/Twist` | Clamped to 0.8 m/s and 1.5 rad/s, ramped at ≤ 0.5 m/s². A 300 ms watchdog stops the robot when commands stop. |
| `odom` | `nav_msgs/Odometry` | `odom` → `base_link`, 20 Hz. Distance comes from step counts and heading from the gyro. No TF is broadcast. |
| `imu/data_raw` | `sensor_msgs/Imu` | `imu_link`, 20 Hz, from the MPU6050. Without an IMU it publishes synthetic data from the wheels with covariance 1e3. |
| `bumper_tripped` | `std_msgs/Bool` | Publishes on every change and every 500 ms. True means the NC loop is open. |
| `parcel/weight_g` | `std_msgs/Float32` | 5 Hz, only when the HX711 answered at boot. |

QoS (measured on the original, 2026-10-01): all four publishers RELIABLE + VOLATILE, the `cmd_vel`
subscription BEST_EFFORT.

Transport: micro-ROS serial over the native USB CDC (`/dev/esp32`), agent at 115200 (the baud is
ignored on native USB). The log goes to UART0. Reset it only with `tools/esp32_reset.py`: on the S3's
native USB, a plain RTS pulse while DTR is high arms download mode, and the next reset then boots the
ROM loader instead of this firmware.
