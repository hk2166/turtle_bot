#!/usr/bin/env python3
# RECONSTRUCTED from the readiness-sweep table in RECOVERY.md (2026-09-30) — not yet run on the robot.
"""Pre-flight / readiness sweep. Run by run/remap.sh before mapping; also usable alone:

    python3 tools/preflight.py            (with the bring-up running)

Last session's all-green state:
  Lidar /scan live on /dev/ttyUSB0 · column mask 52.0 beams, 0 leaks · IMU a real gyro (live noise) ·
  heading from gyro · /odom + micro-ROS agent up · TF odom→base_link→laser/imu/camera, laser z 0.215 m ·
  camera off while mapping · 5 V clean.
Exit status 0 = every critical check passed (warnings allowed), 1 = something critical failed.
"""
import argparse
import math
import os
import statistics
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
from campus_common import PROJECT_DIR, load_env, load_mask, mask_indices  # noqa: E402

G, R, Y, N = '\033[32m', '\033[31m', '\033[33m', '\033[0m'
results = []


def report(ok, label, detail, critical=True):
    """ok: True = pass, False = fail, None = warning/unknown."""
    results.append((ok, label, detail, critical))


def find_topic(node, preferred, type_name):
    topics = dict(node.get_topic_names_and_types())
    if preferred in topics and type_name in topics[preferred]:
        return preferred
    for name, types in topics.items():
        if type_name in types:
            return name
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=5.0, help='how long to listen')
    ap.add_argument('--sim', action='store_true')
    ap.add_argument('--lidar-port', default='')
    args = ap.parse_args()
    env = load_env()
    self_radius = float(env.get('SELF_RADIUS', 0.40))
    laser_z = float(env.get('LASER_Z', 0.215))

    rclpy.init()
    node = rclpy.create_node('preflight')
    tf_buf = Buffer(cache_time=Duration(seconds=10))
    TransformListener(tf_buf, node)

    # Give discovery a moment so topic lookups see the ESP32's topics (at least 2 s, up to 6 s
    # while the odom topic is still missing).
    t0 = time.monotonic()
    while time.monotonic() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > 2.0 and find_topic(node, env.get('ODOM_TOPIC', '/odom'), 'nav_msgs/msg/Odometry'):
            break

    scan_t = env.get('SCAN_TOPIC', '/scan')
    filt_t = env.get('SCAN_FILTERED_TOPIC', '/scan_filtered')
    odom_t = find_topic(node, env.get('ODOM_TOPIC', '/odom'), 'nav_msgs/msg/Odometry')
    imu_t = find_topic(node, env.get('IMU_TOPIC', '/imu/data_raw'), 'sensor_msgs/msg/Imu')

    data = {'scan': [], 'filt': [], 'odom': [], 'imu': []}
    node.create_subscription(LaserScan, scan_t, lambda m: data['scan'].append(m), qos_profile_sensor_data)
    node.create_subscription(LaserScan, filt_t, lambda m: data['filt'].append(m), qos_profile_sensor_data)
    if odom_t:
        node.create_subscription(Odometry, odom_t, lambda m: data['odom'].append(m), qos_profile_sensor_data)
    if imu_t:
        node.create_subscription(Imu, imu_t, lambda m: data['imu'].append(m), qos_profile_sensor_data)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    dur = time.monotonic() - t0

    # --- Lidar ---
    n = len(data['scan'])
    src = 'simulated' if args.sim else (args.lidar_port or 'serial')
    if n:
        report(n / dur > 5, 'Lidar /scan', f'{n / dur:.1f} Hz, {len(data["scan"][-1].ranges)} beams ({src})')
    else:
        report(False, 'Lidar /scan', f'no messages on {scan_t} ({src}). Check the CP2102 stick, 460800 baud, 5 V to the lidar.')

    # --- Column mask ---
    sectors, calibrated = load_mask()
    if data['filt']:
        masked, leaks = [], []
        for m in data['filt']:
            idx = set(mask_indices(m.angle_min, m.angle_increment, len(m.ranges), sectors))
            masked.append(len(idx))
            leaks.append(sum(1 for i, r in enumerate(m.ranges)
                             if i not in idx and math.isfinite(r) and m.range_min < r < self_radius))
        avg_m, avg_l = statistics.mean(masked), statistics.mean(leaks)
        detail = f'{avg_m:.1f} beams masked, {avg_l:.0f} leaks (last good: 52.0 masked, 0 leaks)'
        if not calibrated or not sectors:
            report(None, 'Column mask', detail + ' — NOT CALIBRATED: run tools/calibrate_mask.py', critical=False)
        else:
            report(avg_l < 1, 'Column mask', detail + ('' if avg_l < 1 else
                   ' — something within SELF_RADIUS outside the mask: recalibrate, or move away from obstacles'),
                   critical=False)
    else:
        report(False, 'Column mask', f'no messages on {filt_t}: is scan_filter.py running?')

    # --- IMU ---
    if not imu_t:
        report(None, 'IMU', 'no sensor_msgs/Imu topic found (firmware may not publish it)', critical=False)
    elif not data['imu']:
        report(None, 'IMU', f'{imu_t} exists but no messages', critical=False)
    else:
        gz = [m.angular_velocity.z for m in data['imu']]
        gx = [m.angular_velocity.x for m in data['imu']]
        sd = statistics.pstdev(gz) if len(gz) > 1 else 0.0
        real = sd > 1e-6 and statistics.pstdev(gx) > 1e-6
        report(real if real else None, 'IMU',
               f'{imu_t} {len(gz) / dur:.0f} Hz, gyro-z noise σ={sd:.5f} rad/s → '
               + ('real gyro' if real else 'NO NOISE: looks like the synthetic placeholder, not the MPU6050'),
               critical=False)
    report(None, 'Heading source', 'gyro (set in firmware; not checkable from the Pi)', critical=False)

    # --- Odom + agent ---
    if not odom_t or not data['odom']:
        report(False, '/odom', 'no odometry: is the ESP32 powered and the micro-ROS agent connected?')
    else:
        report(len(data['odom']) / dur > 5, '/odom', f'{odom_t} {len(data["odom"]) / dur:.0f} Hz')
    if not args.sim:
        up = subprocess.run(['pgrep', '-f', 'micro_ros_agent'], capture_output=True).returncode == 0
        report(up, 'micro-ROS agent', 'running' if up else 'not running')
    cmd_t = env.get('CMD_VEL_TOPIC', '/cmd_vel')
    subs = node.get_subscriptions_info_by_topic(cmd_t)
    types = sorted({s.topic_type for s in subs})
    report(bool(subs), f'{cmd_t} listener', f'{len(subs)} subscriber(s) {types}' if subs else
           'nobody subscribes: the ESP32 is not taking velocity commands')

    # --- TF ---
    def lookup(parent, child):
        try:
            return tf_buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=1.0))
        except Exception:
            return None
    chain = []
    ok_tf = True
    for child in ('laser', 'imu', 'camera'):
        t = lookup('odom', child)
        chain.append(f'{child} {"✓" if t else "✗"}')
        ok_tf &= t is not None if child == 'laser' else True
    report(ok_tf, 'TF tree', 'odom → base_link → ' + ', '.join(chain))
    t = lookup('base_link', 'laser')
    if t:
        z = t.transform.translation.z
        report(abs(z - laser_z) < 0.005, 'base_link → laser', f'z = {z:.3f} m (expected {laser_z:.3f})')
    statics = node.get_publishers_info_by_topic('/tf_static')
    report(len(statics) >= 3 or None, 'Static TF publishers', f'{len(statics)} (≥ 3 expected; the original had 6)',
           critical=False)

    # --- Camera off ---
    img = [nm for nm, ty in node.get_topic_names_and_types() if 'sensor_msgs/msg/Image' in ty
           and node.count_publishers(nm)]
    report(not img or None, 'Camera', 'off (good for mapping)' if not img else
           f'ON ({", ".join(img)}): run/camera.sh off to free CPU and WiFi', critical=False)

    # --- Power ---
    pc = subprocess.run(['bash', os.path.join(PROJECT_DIR, 'run', 'power_check.sh'), '--brief'],
                        capture_output=True, text=True)
    report(True if pc.returncode == 0 else None, 'Power', pc.stdout.strip() or pc.stderr.strip(), critical=False)

    node.destroy_node()
    rclpy.try_shutdown()

    print('\n  Pre-flight')
    fail = False
    for ok, label, detail, critical in results:
        mark = f'{G}✓{N}' if ok else (f'{R}✗{N}' if ok is False and critical else f'{Y}!{N}')
        fail |= ok is False and critical
        print(f'   {mark} {label:<22} {detail}')
    print(f'\n  {R}NOT READY{N}: fix the ✗ lines.' if fail else f'\n  {G}READY{N}' +
          ('' if all(r[0] for r in results) else ' (see the ! lines)'))
    sys.exit(1 if fail else 0)


if __name__ == '__main__':
    main()
