// RECONSTRUCTED from the ESP32 flash dump strings + CLAUDE.md (2026-10-01) — not yet run on the robot.
// The firmware on the chip (backed up 2026-10-01 to ~/recovered/esp32-*/) is still the working one.
// Do NOT flash this until it has been bench-tested with the wheels off the floor.
//
// Every value marked VERIFY is a guess: the dump gives names and log text, not numbers.
// The old firmware logs its pins/constants on UART0 (the board's UART USB-C port) at boot:
//   [imu] not found on SDA=%d SCL=%d ...     [drive] ready: %.1f steps/m, acc %u steps/s^2
// Plug that port in, `pio device monitor -p /dev/ttyUSB1`, press RESET and copy the numbers here.
#pragma once

// ---------- Geometry (calibrate with tools/calibrate_odom.py, then edit here) ----------
// The ORIGINAL firmware's boot log (2026-10-10, spare board on UART0) says "[drive] ready: 4676.7
// steps/m": with 1600 steps/rev that is a 108.9 mm wheel in its maths, NOT the 100 mm assumed on
// 2026-10-01. The Pi's calibrated correction (ODOM_LIN_SCALE 0.8994, tools/calibrate_odom.py, carpet)
// therefore puts the real effective diameter at 0.8994 x 108.9 = 97.9 mm. (The old 89.9 mm here came
// from applying 0.8994 to 100 mm and would have made every distance ~9% short.) If THIS firmware is
// flashed with the value below, set ODOM_LIN_SCALE / ODOM_ANG_SCALE in config/robot.env back to 1.0.
#define WHEEL_DIA        0.0979f  // m, effective on carpet (derived above; re-run calibrate_odom.py after flashing)
#define WHEEL_SEP        0.350f   // m, nominal 340–360; spin calibration sets the EFFECTIVE value
#define STEPS_PER_REV    1600     // DM542 DIP setting (no gearbox)
#define INVERT_LEFT      false    // VERIFY: motors are mirrored, one side normally needs inverting
#define INVERT_RIGHT     true     // VERIFY
#define GYRO_SCALE       1.000f   // heading correction from the spin test (gyro turns / real turns)

// ---------- Motion limits (CLAUDE.md hard rules) ----------
#define MAX_LIN_SPEED    0.80f    // m/s
#define MAX_ANG_SPEED    1.50f    // rad/s
#define MAX_LIN_ACCEL    0.50f    // m/s^2 — tower stability, NEVER raise. (The original firmware ramps at
                                  // 7015 steps/s^2 = 1.5 m/s^2 in its maths, ~1.35 m/s^2 real: its own
                                  // watchdog/bumper stops are harder than this. The Pi's smoother keeps
                                  // normal driving at 0.5.)
#define MAX_ANG_ACCEL    1.00f    // rad/s^2
#define CMD_TIMEOUT_MS   300      // command watchdog: no cmd_vel for this long → ramp to 0
// Bumper hit: stop harder than the normal ramp. Still a ramp, not a step-stall, so the tower
// isn't thrown. VERIFY on the floor with a load on the platform.
#define BUMPER_DECEL     1.00f    // m/s^2

// ---------- Pins (Edgebax S3-PRO). Avoid IO0, IO3, IO45, IO46; IO5/IO6 don't work on this board. ----------
#define PIN_LEFT_PUL     4        // motor A, bench-proven (PUL−, common-anode +5 V)
#define PIN_LEFT_DIR     7        // motor A, bench-proven (DIR−)
#define PIN_RIGHT_PUL    -1       // UNKNOWN: NOT 8/9 - the IMU is there (original boot log, below). The
#define PIN_RIGHT_DIR    -1       // CLAUDE.md "Motor B: IO8/IO9" note was wrong; find the real pins.
#define PIN_BUMPER       10       // VERIFY: 6× V-156 NC in series to GND, INPUT_PULLUP → HIGH = open = tripped
#define PIN_IMU_SDA      8        // REAL: original boot log "[imu] not found on SDA=8 SCL=9" (2026-10-10,
#define PIN_IMU_SCL      9        // original board, IMU disconnected after the swap)
#define PIN_HX711_DT     11       // REAL: original boot log "[hx711] no response on DT=11 SCK=12" (2026-10-10)
#define PIN_HX711_SCK    12       // REAL

// ---------- Sensors ----------
#define MPU_ADDR         0x68
#define GYRO_BIAS_SAMPLES 400     // ~2 s at boot; robot must be still
#define HX711_SCALE      420.0f   // VERIFY: raw counts per gram (put a known weight on the crate floor)

// ---------- Rates ----------
#define CONTROL_HZ       50
#define ODOM_HZ          20       // the dump ran /odom and /imu/data_raw at 20 Hz (measured 2026-10-01)
#define WEIGHT_HZ        5
#define BUMPER_PUB_MS    500      // also published immediately on every change
#define PUB_FAILS_RECONNECT 20    // "publishes failing - reconnecting"
