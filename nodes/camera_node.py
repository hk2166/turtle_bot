#!/usr/bin/env python3
# Written 2026-10-01: the mast camera's ROS driver + AprilTag detector. Not yet run on the robot end to end.
"""Mast camera (Logitech Brio 100) → AprilTag detections, plus a light preview.

    python3 nodes/camera_node.py --ros-args -p tag_size:=0.16     (run/camera.sh on starts it)

Owns /dev/video0 (1280x720 MJPG; neither v4l2_camera nor usb_cam decode the Brio's MJPG on this Pi).
Publishes
    /tags                        vision_msgs/Detection3DArray  every tag36h11 tag seen, stamped at capture
                                 time. results[0]: class_id = tag id, pose = tag in frame camera_optical,
                                 score = tag edge in pixels (negative: orientation ambiguous, a face-on
                                 or small tag; position still good). results[1..4]: class_id 'corner', position =
                                 the observed corner's ray (x/z, y/z, 1), undistorted, in aruco order.
    /camera/image/compressed     sensor_msgs/CompressedImage   small JPEG preview (8 Hz, 480 px) with tags drawn
    /camera/camera_info          sensor_msgs/CameraInfo        for the preview size
Full frames never go on the network: detection runs here, so mapping over WiFi stays light.
Intrinsics: config/camera_intrinsics.yaml (tools/calibrate_camera.py); spec-sheet FOV until then.
"""
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

import tags_common as tc


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.device = self.declare_parameter('device', '/dev/video0').value
        self.width = int(self.declare_parameter('width', 1280).value)
        self.height = int(self.declare_parameter('height', 720).value)
        self.fps = int(self.declare_parameter('fps', 30).value)
        self.frame_id = self.declare_parameter('frame_id', 'camera_optical').value
        self.tag_size = float(self.declare_parameter('tag_size', 0.16).value)
        self.min_side_px = float(self.declare_parameter('min_side_px', 20.0).value)
        detect_hz = float(self.declare_parameter('detect_hz', 10.0).value)
        # 8 Hz x 480 px (was 2 Hz x 640): the dashboard feed lagged; detection already decodes 10 Hz,
        # so the preview only adds a resize + small JPEG per frame (2026-10-10)
        preview_hz = float(self.declare_parameter('preview_hz', 8.0).value)
        self.preview_width = int(self.declare_parameter('preview_width', 480).value)
        # Capture → read latency of the MJPG pipeline; the stamp is moved back by this much.
        self.latency = float(self.declare_parameter('latency', 0.05).value)

        self.K, self.D, calibrated = tc.load_intrinsics(self.width, self.height)
        if calibrated:
            self.get_logger().info(f'intrinsics: {tc.INTRINSICS_FILE}')
        else:
            self.get_logger().warning('camera NOT calibrated: using the Brio spec FOV (tag distances can be '
                                      'off by ~5%). Run tools/calibrate_camera.py')
        self.detector = tc.TagDetector()

        self.pub_tags = self.create_publisher(Detection3DArray, '/tags', 10)
        self.pub_img = self.create_publisher(CompressedImage, '/camera/image/compressed', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)

        self.lock = threading.Lock()
        self.frame = None          # (seq, stamp_msg, bgr)
        self.seq = 0
        self.last_detected_seq = -1
        self.last_dets = []        # for the preview overlay
        self.running = True
        self.want = threading.Event()   # set by detect()/preview(): decode the next frame
        self.want.set()
        self.stats = {'frames': 0, 'detect_runs': 0, 'tags': 0}
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.create_timer(1.0 / detect_hz, self.detect)
        self.create_timer(1.0 / preview_hz, self.preview)
        self.create_timer(30.0, self.report)

    # ---------------------------------------------------------------- capture thread
    def capture_loop(self):
        cap, warned = None, False
        while self.running:
            if cap is None:
                busy = tc.camera_busy(self.device)
                cap = tc.open_camera(self.device, self.width, self.height, self.fps)
                if cap is None:
                    if not warned:
                        self.get_logger().error(f'cannot open {self.device}' +
                                                (f' (held by PID {busy})' if busy else ' (unplugged?)') +
                                                ': retrying every 2 s')
                        warned = True
                    time.sleep(2.0)
                    continue
                w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                if (w, h) != (self.width, self.height):
                    self.get_logger().error(f'camera gives {w}x{h}, not {self.width}x{self.height}')
                self.get_logger().info(f'{self.device} open: {w}x{h} MJPG')
                warned = False
            # grab() every frame so the newest is never stale, but decode only when asked: MJPG
            # decoding all 30 fps took half a core on the Pi 5.
            ok = cap.grab()
            img = None
            if ok and self.want.is_set():
                ok, img = cap.retrieve()
            if not ok:
                self.get_logger().warning(f'{self.device}: read failed, reopening')
                cap.release()
                cap = None
                time.sleep(0.5)
                continue
            if img is None:
                continue
            self.want.clear()
            stamp = self.get_clock().now() - Duration(seconds=self.latency)
            with self.lock:
                self.seq += 1
                self.frame = (self.seq, stamp.to_msg(), img)
                self.stats['frames'] += 1
        if cap is not None:
            cap.release()

    # ---------------------------------------------------------------- detection
    def detect(self):
        self.want.set()
        with self.lock:
            if self.frame is None or self.frame[0] == self.last_detected_seq:
                return
            seq, stamp, img = self.frame
        self.last_detected_seq = seq
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        out = Detection3DArray()
        out.header.stamp = stamp
        out.header.frame_id = self.frame_id
        drawn = []
        for tag_id, corners in self.detector.detect(gray):
            side = tc.side_px(corners)
            T, why = (None, 'too small') if side < self.min_side_px else \
                tc.tag_pose(corners, self.tag_size, self.K, self.D, keep_ambiguous=True)
            drawn.append((tag_id, corners, T is not None))
            if T is None:
                continue
            d = Detection3D()
            d.header = out.header
            d.id = str(tag_id)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = str(tag_id)
            # pixels per edge, negative when the orientation is ambiguous (position still good)
            h.hypothesis.score = float(-side if why == 'ambiguous' else side)
            p = h.pose.pose
            p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
            qx, qy, qz, qw = tc.mat_to_quat(T[:3, :3])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            d.results.append(h)
            # The corners as seen (undistorted, normalized: x/z, y/z) for the planar fix in tag_map.py
            for (u, v) in cv2.undistortPoints(corners.reshape(-1, 1, 2), self.K, self.D).reshape(4, 2):
                c = ObjectHypothesisWithPose()
                c.hypothesis.class_id = 'corner'
                c.pose.pose.position.x, c.pose.pose.position.y, c.pose.pose.position.z = float(u), float(v), 1.0
                d.results.append(c)
            d.bbox.center = p
            d.bbox.size.x = d.bbox.size.y = self.tag_size
            out.detections.append(d)
        self.pub_tags.publish(out)
        self.last_dets = drawn
        self.stats['detect_runs'] += 1
        self.stats['tags'] += len(out.detections)

    # ---------------------------------------------------------------- preview
    def preview(self):
        if self.pub_img.get_subscription_count() == 0 and self.pub_info.get_subscription_count() == 0:
            return
        with self.lock:
            if self.frame is None:
                return
            _, stamp, img = self.frame
        s = self.preview_width / float(img.shape[1])
        small = cv2.resize(img, (self.preview_width, int(round(img.shape[0] * s))), interpolation=cv2.INTER_AREA)
        for tag_id, corners, good in self.last_dets:
            pts = (corners * s).astype(np.int32)
            cv2.polylines(small, [pts], True, (0, 255, 0) if good else (0, 165, 255), 2)
            cv2.putText(small, str(tag_id), tuple(int(v) for v in pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if good else (0, 165, 255), 2)
        ok, jpg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpg.tobytes()
        self.pub_img.publish(msg)
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = small.shape[1], small.shape[0]
        K = self.K.copy()
        K[:2] *= s
        info.k = [float(v) for v in K.reshape(-1)]
        info.d = [float(v) for v in self.D.reshape(-1)]
        info.distortion_model = 'plumb_bob'
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def report(self):
        st = self.stats
        self.get_logger().info(f'last 30 s: {st["frames"] / 30.0:.1f} frames/s decoded, {st["detect_runs"]} detection runs, '
                               f'{st["tags"]} tag sightings')
        self.stats = {k: 0 for k in st}

    def close(self):
        self.running = False
        self.thread.join(timeout=2.0)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
