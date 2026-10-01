// RECONSTRUCTED from the ESP32 flash dump strings + CLAUDE.md (2026-10-01) — not yet run on the robot.
// campus_bot_base: differential-drive base for the Campus Bot. micro-ROS client over the native USB
// port (Serial = HW CDC → /dev/ttyACM0 on the Pi; agent at 115200), human-readable log on UART0.
//
//   sub  cmd_vel          geometry_msgs/Twist     300 ms watchdog, ramped at ≤ 0.5 m/s²
//   pub  odom             nav_msgs/Odometry       odom → base_link, distance from steps, heading from gyro
//   pub  imu/data_raw     sensor_msgs/Imu         imu_link, MPU6050 (synthetic from wheels if absent)
//   pub  bumper_tripped   std_msgs/Bool           NC loop open = true; blocks forward motion
//   pub  parcel/weight_g  std_msgs/Float32        HX711 under the crate floor (only if present)
//
// The odom → base_link TF is NOT broadcast here; nodes/odom_to_tf.py does it on the Pi.
#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <micro_ros_utilities/string_utilities.h>
#include "config.h"

#define LOG(...) Serial0.printf(__VA_ARGS__)
#define RCCHECK(fn) { rcl_ret_t rc_ = (fn); if (rc_ != RCL_RET_OK) return false; }
#define RCIGNORE(fn) { rcl_ret_t rc_ = (fn); (void)rc_; }   // teardown with the agent gone: failures expected

static const float STEPS_PER_M = STEPS_PER_REV / (PI * WHEEL_DIA);

// ======================================================================= drive
static FastAccelStepperEngine engine;
static FastAccelStepper *left = nullptr, *right = nullptr;

static float target_v = 0, target_w = 0;     // last command (clamped)
static float cur_v = 0, cur_w = 0;           // ramped command actually applied
static uint32_t last_cmd_ms = 0;
static bool bumper = false;

static void set_wheel(FastAccelStepper *s, float mps) {
  int32_t hz = (int32_t)lroundf(fabsf(mps) * STEPS_PER_M);
  if (hz < 20) { s->stopMove(); return; }    // below ~1 cm/s: stop (FastAccelStepper min speed)
  s->setSpeedInHz(hz);
  if (mps > 0) s->runForward(); else s->runBackward();
}

static bool drive_init() {
  engine.init();
  left = engine.stepperConnectToPin(PIN_LEFT_PUL);
  right = engine.stepperConnectToPin(PIN_RIGHT_PUL);
  if (!left || !right) { LOG("[drive] FATAL: stepperConnectToPin failed (no free RMT/PCNT unit?)\n"); return false; }
  // The robot-level ramp below enforces the limits; the stepper ramp only has to keep up with it.
  uint32_t acc = (uint32_t)((MAX_LIN_ACCEL + MAX_ANG_ACCEL * WHEEL_SEP / 2) * STEPS_PER_M * 1.2f);
  left->setDirectionPin(PIN_LEFT_DIR, !INVERT_LEFT);
  right->setDirectionPin(PIN_RIGHT_DIR, !INVERT_RIGHT);
  left->setAcceleration(acc);
  right->setAcceleration(acc);
  LOG("[drive] ready: %.1f steps/m, acc %u steps/s^2\n", STEPS_PER_M, acc);
  return true;
}

static float ramp(float cur, float tgt, float step) {
  return cur + constrain(tgt - cur, -step, step);
}

// 50 Hz: watchdog, bumper interlock, (v, w) ramp, wheel speeds.
static void drive_update(float dt) {
  float tv = target_v, tw = target_w;
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) tv = tw = 0;      // watchdog
  float lin_acc = MAX_LIN_ACCEL;
  if (bumper && tv > 0) { tv = 0; tw = 0; }                      // may back off / turn, never push on
  if (bumper && cur_v > 0) lin_acc = BUMPER_DECEL;
  cur_v = ramp(cur_v, tv, lin_acc * dt);
  cur_w = ramp(cur_w, tw, MAX_ANG_ACCEL * dt);
  set_wheel(left, cur_v - cur_w * WHEEL_SEP / 2);
  set_wheel(right, cur_v + cur_w * WHEEL_SEP / 2);
}

static void drive_stop_now() {      // agent lost: ramp down, the loop keeps running with zero target
  target_v = target_w = 0;
}

// ======================================================================= bumper
static void bumper_update() {
  static uint8_t hist = 0;
  hist = (hist << 1) | (digitalRead(PIN_BUMPER) == HIGH);   // open loop = tripped
  bool now = bumper ? (hist != 0x00) : ((hist & 0x07) == 0x07);  // trip on 3 samples, clear on 8
  if (now != bumper) { bumper = now; LOG("[bumper] %s\n", bumper ? "TRIPPED" : "clear"); }
}

// ======================================================================= IMU (MPU6050, raw registers)
static bool imu_ok = false;
static float gbias[3] = {0, 0, 0};
static float gyro[3], accel[3];          // rad/s, m/s^2 (body frame, bias removed)

static bool mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpu_read_raw(int16_t a[3], int16_t g[3]) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++) {
    a[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]);
    g[i] = (int16_t)(b[8 + 2 * i] << 8 | b[9 + 2 * i]);
  }
  return true;
}

static void imu_init() {
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL, 400000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    LOG("[imu] not found on SDA=%d SCL=%d - check the header wiring\n", PIN_IMU_SDA, PIN_IMU_SCL);
    return;
  }
  mpu_write(0x6B, 0x01);   // wake, PLL on gyro X
  mpu_write(0x1A, 0x03);   // DLPF 44 Hz
  mpu_write(0x1B, 0x00);   // ±250 °/s → 131 LSB/(°/s)
  mpu_write(0x1C, 0x00);   // ±2 g → 16384 LSB/g
  delay(100);
  int16_t a[3], g[3];
  double sum[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    if (mpu_read_raw(a, g)) { for (int k = 0; k < 3; k++) sum[k] += g[k]; n++; }
    delay(5);
  }
  if (n < GYRO_BIAS_SAMPLES / 2) { LOG("[imu] present but unreadable\n"); return; }
  for (int k = 0; k < 3; k++) gbias[k] = sum[k] / n;
  imu_ok = true;
  LOG("[imu] ok, gyro bias %.1f %.1f %.1f LSB (%d samples)\n", gbias[0], gbias[1], gbias[2], n);
}

static bool imu_update() {
  int16_t a[3], g[3];
  if (!imu_ok || !mpu_read_raw(a, g)) return false;
  const float G2RAD = (PI / 180.0f) / 131.0f, A2MS = 9.80665f / 16384.0f;
  for (int k = 0; k < 3; k++) { gyro[k] = (g[k] - gbias[k]) * G2RAD; accel[k] = a[k] * A2MS; }
  gyro[2] /= GYRO_SCALE;
  return true;
}

// ======================================================================= HX711 (bit-banged, channel A gain 128)
static bool hx_ok = false;
static long hx_tare = 0;

static bool hx_ready(uint32_t timeout_ms) {
  uint32_t t = millis();
  while (digitalRead(PIN_HX711_DT) == HIGH) { if (millis() - t > timeout_ms) return false; delay(1); }
  return true;
}

static long hx_read() {
  long v = 0;
  noInterrupts();
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_HX711_SCK, HIGH); delayMicroseconds(1);
    v = (v << 1) | digitalRead(PIN_HX711_DT);
    digitalWrite(PIN_HX711_SCK, LOW); delayMicroseconds(1);
  }
  digitalWrite(PIN_HX711_SCK, HIGH); delayMicroseconds(1);   // 25th pulse: gain 128 next
  digitalWrite(PIN_HX711_SCK, LOW);
  interrupts();
  if (v & 0x800000) v |= ~0xFFFFFFL;   // sign-extend
  return v;
}

static void hx_init() {
  pinMode(PIN_HX711_DT, INPUT);
  pinMode(PIN_HX711_SCK, OUTPUT);
  digitalWrite(PIN_HX711_SCK, LOW);
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    if (!hx_ready(500)) { LOG("[hx711] no response on DT=%d SCK=%d\n", PIN_HX711_DT, PIN_HX711_SCK); return; }
    sum += hx_read();
  }
  hx_tare = sum / 10;
  hx_ok = true;
  LOG("[hx711] ok, tare offset %ld\n", hx_tare);
}

// ======================================================================= odometry
static float ox = 0, oy = 0, oth = 0, meas_v = 0, meas_w = 0;
static int32_t last_l = 0, last_r = 0;

static void odom_update(float dt, bool have_gyro) {
  int32_t l = left->getCurrentPosition(), r = right->getCurrentPosition();
  float dl = (l - last_l) / STEPS_PER_M, dr = (r - last_r) / STEPS_PER_M;
  last_l = l; last_r = r;
  float ds = (dl + dr) / 2;
  // Heading from the gyro (survives carpet turn-slip, ~60% turn efficiency); wheels only as fallback.
  float dth = have_gyro ? gyro[2] * dt : (dr - dl) / WHEEL_SEP;
  float mid = oth + dth / 2;
  ox += ds * cosf(mid);
  oy += ds * sinf(mid);
  oth = atan2f(sinf(oth + dth), cosf(oth + dth));
  meas_v = ds / dt;
  meas_w = dth / dt;
}

// ======================================================================= micro-ROS
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState state = WAITING_AGENT;

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t cmd_sub;
static rcl_publisher_t odom_pub, imu_pub, bump_pub, weight_pub;
static geometry_msgs__msg__Twist cmd_msg;
static nav_msgs__msg__Odometry odom_msg;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__Bool bump_msg;
static std_msgs__msg__Float32 weight_msg;
static uint32_t pub_fails = 0;

static void cmd_cb(const void *msg) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msg;
  float v = m->linear.x, w = m->angular.z;
  target_v = isfinite(v) ? constrain(v, -MAX_LIN_SPEED, MAX_LIN_SPEED) : 0;
  target_w = isfinite(w) ? constrain(w, -MAX_ANG_SPEED, MAX_ANG_SPEED) : 0;
  last_cmd_ms = millis();
}

static void stamp(builtin_interfaces__msg__Time *t) {
  int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : (int64_t)millis() * 1000000LL;
  t->sec = ns / 1000000000LL;
  t->nanosec = ns % 1000000000LL;
}

static void publish(rcl_publisher_t *p, const void *m) {
  static bool failing = false;
  rcl_ret_t rc = rcl_publish(p, m, NULL);
  if (rc != RCL_RET_OK) {
    if (pub_fails++ == 0 || pub_fails % 100 == 0) LOG("[odom] rcl_publish failed rc=%d: %s\n", (int)rc, rcl_get_error_string().str);
    rcl_reset_error();
    failing = true;
  } else if (failing) {
    LOG("[odom] publishing again after %u failures\n", pub_fails);
    failing = false;
    pub_fails = 0;
  }
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "campus_bot_base", "", &support));
  // QoS as measured on the original firmware (2026-10-01): every publisher RELIABLE, cmd_vel BEST_EFFORT.
  RCCHECK(rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  RCCHECK(rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
  RCCHECK(rclc_publisher_init_default(&bump_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "bumper_tripped"));
  RCCHECK(rclc_publisher_init_default(&weight_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "parcel/weight_g"));
  RCCHECK(rclc_subscription_init_best_effort(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_cb, ON_NEW_DATA));
  rmw_uros_sync_session(100);
  pub_fails = 0;
  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
  RCIGNORE(rcl_subscription_fini(&cmd_sub, &node));
  RCIGNORE(rcl_publisher_fini(&odom_pub, &node));
  RCIGNORE(rcl_publisher_fini(&imu_pub, &node));
  RCIGNORE(rcl_publisher_fini(&bump_pub, &node));
  RCIGNORE(rcl_publisher_fini(&weight_pub, &node));
  RCIGNORE(rclc_executor_fini(&executor));
  RCIGNORE(rcl_node_fini(&node));
  RCIGNORE(rclc_support_fini(&support));
}

static void init_messages() {
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");
  imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
  // Planar robot: x, y, yaw are estimated; z, roll, pitch are pinned (large covariance).
  const double P[6] = {0.01, 0.01, 1e6, 1e6, 1e6, 0.02}, T[6] = {0.01, 1e6, 1e6, 1e6, 1e6, 0.02};
  for (int i = 0; i < 6; i++) { odom_msg.pose.covariance[i * 7] = P[i]; odom_msg.twist.covariance[i * 7] = T[i]; }
  imu_msg.orientation_covariance[0] = -1;   // no orientation estimate (data_raw)
  for (int i = 0; i < 3; i++) {
    imu_msg.angular_velocity_covariance[i * 4] = 0.0004;
    imu_msg.linear_acceleration_covariance[i * 4] = 0.04;
  }
}

static void publish_odom_imu(bool have_gyro) {
  stamp(&odom_msg.header.stamp);
  odom_msg.pose.pose.position.x = ox;
  odom_msg.pose.pose.position.y = oy;
  odom_msg.pose.pose.orientation.z = sinf(oth / 2);
  odom_msg.pose.pose.orientation.w = cosf(oth / 2);
  odom_msg.twist.twist.linear.x = meas_v;
  odom_msg.twist.twist.angular.z = meas_w;
  publish(&odom_pub, &odom_msg);

  imu_msg.header.stamp = odom_msg.header.stamp;
  if (have_gyro) {
    imu_msg.angular_velocity.x = gyro[0]; imu_msg.angular_velocity.y = gyro[1]; imu_msg.angular_velocity.z = gyro[2];
    imu_msg.linear_acceleration.x = accel[0]; imu_msg.linear_acceleration.y = accel[1]; imu_msg.linear_acceleration.z = accel[2];
    imu_msg.angular_velocity_covariance[8] = 0.0004;
  } else {
    // Synthetic placeholder: yaw rate from the wheels, flagged with a huge covariance so nothing trusts it.
    imu_msg.angular_velocity.x = imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = meas_w;
    imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 9.80665;
    imu_msg.angular_velocity_covariance[8] = 1e3;
  }
  publish(&imu_pub, &imu_msg);
}

// ======================================================================= main
void setup() {
  Serial0.begin(115200);                 // log (UART USB-C port)
  LOG("\n[campus_bot] boot\n");
  LOG("[campus_bot] reset_reason=%d heap=%u MTU=%d hist=%d\n", (int)esp_reset_reason(), ESP.getFreeHeap(),
      RMW_UXRCE_MAX_TRANSPORT_MTU, RMW_UXRCE_MAX_HISTORY);
  pinMode(PIN_BUMPER, INPUT_PULLUP);
  if (!drive_init()) { while (true) delay(1000); }
  imu_init();
  hx_init();
  init_messages();
  Serial.begin(115200);                  // micro-ROS transport (native USB → /dev/ttyACM0)
  set_microros_serial_transports(Serial);
  LOG("[campus_bot] waiting for agent\n");
}

void loop() {
  static uint32_t t_ctrl = 0, t_odom = 0, t_weight = 0, t_bump = 0, t_ping = 0, t_alive = 0;
  static bool last_bump_pub = false;
  uint32_t now = millis();

  // Control runs whatever the agent is doing, so the watchdog and bumper always apply.
  if (now - t_ctrl >= 1000 / CONTROL_HZ) {
    float dt = (now - t_ctrl) / 1000.0f;
    t_ctrl = now;
    bumper_update();
    drive_update(dt);
  }
  if (now - t_odom >= 1000 / ODOM_HZ) {
    float dt = (now - t_odom) / 1000.0f;
    t_odom = now;
    bool g = imu_update();
    odom_update(dt, g);
    if (state == AGENT_CONNECTED) publish_odom_imu(g);
  }

  switch (state) {
    case WAITING_AGENT:
      if (now - t_ping > 500) { t_ping = now; if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) state = AGENT_AVAILABLE; }
      break;
    case AGENT_AVAILABLE:
      if (create_entities()) { state = AGENT_CONNECTED; LOG("[campus_bot] agent connected\n"); }
      else { destroy_entities(); state = WAITING_AGENT; }
      break;
    case AGENT_CONNECTED:
      if (now - t_ping > 1000) {
        t_ping = now;
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) { state = AGENT_DISCONNECTED; break; }
      }
      if (pub_fails > PUB_FAILS_RECONNECT) {
        LOG("[campus_bot] publishes failing - reconnecting\n");
        state = AGENT_DISCONNECTED;
        break;
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      if (bumper != last_bump_pub || now - t_bump >= BUMPER_PUB_MS) {
        t_bump = now; last_bump_pub = bumper;
        bump_msg.data = bumper;
        publish(&bump_pub, &bump_msg);
      }
      if (hx_ok && now - t_weight >= 1000 / WEIGHT_HZ && digitalRead(PIN_HX711_DT) == LOW) {
        t_weight = now;
        weight_msg.data = (hx_read() - hx_tare) / HX711_SCALE;
        publish(&weight_pub, &weight_msg);
      }
      break;
    case AGENT_DISCONNECTED:
      LOG("[campus_bot] agent lost - stopping\n");
      drive_stop_now();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  if (now - t_alive > 10000) {
    t_alive = now;
    LOG("[campus_bot] alive heap=%u fails=%u odom=(%.2f %.2f %.2f)\n", ESP.getFreeHeap(), pub_fails, ox, oy, oth);
  }
}
