# Written 2026-10-01: camera + AprilTag helpers shared by nodes/camera_node.py, nodes/tag_map.py and
# tools/{calibrate_camera,camera_pose,make_tags}.py. Not yet run on the robot end to end.
"""AprilTag (tag36h11) detection and pose maths for the mast camera (Logitech Brio 100).

Frames: OpenCV/ROS optical frame for the camera (x right, y down, z forward) and the OpenCV marker frame
for a tag (origin at its centre, x to the printed right, y to the printed top, z out of the paper
toward the viewer). All poses are 4x4 numpy matrices T_a_b (maps points in b to a).
"""
import math
import os

import cv2
import numpy as np
import yaml

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INTRINSICS_FILE = os.path.join(PROJECT_DIR, 'config', 'camera_intrinsics.yaml')
DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
FAMILY = 'tag36h11'
BRIO100_DFOV_DEG = 58.0      # Logitech spec (diagonal); only used until tools/calibrate_camera.py has run


# --------------------------------------------------------------------------------------------- camera
def brio_device():
    """The mast camera's device. /dev/videoN numbers follow plug order: since the face camera was added,
    a re-plug made the HIKVISION /dev/video0 and camera_node opened it instead of the Brio (2026-10-30).
    So: CAMERA_DEV from config/robot.env (a stable /dev/v4l/by-id name), else any Logitech by-id capture
    node, else /dev/video0 as before."""
    import glob
    import re
    try:
        m = re.search(r'^CAMERA_DEV=(\S+)', open(os.path.join(PROJECT_DIR, 'config', 'robot.env')).read(), re.M)
        if m and os.path.exists(m.group(1)):
            return m.group(1)
    except OSError:
        pass
    found = sorted(glob.glob('/dev/v4l/by-id/usb-046d_*-video-index0'))
    return found[0] if found else '/dev/video0'


def open_camera(device=None, width=1280, height=720, fps=30):
    """The Brio gives 1280x720 at 30 fps only as MJPG (raw YUYV is 5 fps at that size)."""
    cap = cv2.VideoCapture(device or brio_device(), cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def camera_busy(device=None):
    """PIDs (as text) of other processes holding the camera, '' if it's free."""
    real = os.path.realpath(device or brio_device())
    pids = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        try:
            for fd in os.listdir(f'/proc/{pid}/fd'):
                if os.path.realpath(f'/proc/{pid}/fd/{fd}') == real:
                    pids.append(pid)
                    break
        except OSError:
            continue
    return ' '.join(pids)


def load_intrinsics(width, height, path=INTRINSICS_FILE):
    """(K, D, calibrated). Falls back to the Brio's spec FOV if there is no calibration for this size."""
    try:
        with open(path) as f:
            c = yaml.safe_load(f) or {}
        if int(c['image_width']) == width and int(c['image_height']) == height:
            K = np.array(c['camera_matrix']['data'], float).reshape(3, 3)
            D = np.array(c['distortion_coefficients']['data'], float).reshape(-1)
            return K, D, True
    except (FileNotFoundError, KeyError, TypeError, ValueError):
        pass
    diag = math.hypot(width, height)
    f = (diag / 2.0) / math.tan(math.radians(BRIO100_DFOV_DEG) / 2.0)
    K = np.array([[f, 0, (width - 1) / 2.0], [0, f, (height - 1) / 2.0], [0, 0, 1]], float)
    return K, np.zeros(5), False


def save_intrinsics(path, width, height, K, D, rms, n_views):
    """ROS camera_info YAML (what camera_calibration writes), plus how good the fit was."""
    P = np.hstack([K, np.zeros((3, 1))])
    data = {
        'image_width': int(width), 'image_height': int(height), 'camera_name': 'brio100',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in K.reshape(-1)]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': len(D), 'data': [float(v) for v in D.reshape(-1)]},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': [float(v) for v in P.reshape(-1)]},
        'calibration': {'rms_px': round(float(rms), 3), 'views': int(n_views)},
    }
    with open(path, 'w') as f:
        f.write('# Written by tools/calibrate_camera.py: Brio 100 intrinsics (MJPG stream).\n')
        yaml.safe_dump(data, f, sort_keys=False)


# --------------------------------------------------------------------------------------------- tags
class TagDetector:
    def __init__(self):
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
        p = cv2.aruco.DetectorParameters_create()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        p.cornerRefinementWinSize = 3
        self.params = p

    def detect(self, gray):
        """[(id, corners 4x2 float)] in aruco order: top-left, top-right, bottom-right, bottom-left."""
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None:
            return []
        return [(int(i), c.reshape(4, 2).astype(np.float64)) for i, c in zip(ids.reshape(-1), corners)]


def tag_pose(corners, size, K, D, max_ambiguity=0.6, keep_ambiguous=False):
    """(T_cam_tag 4x4, reason) for one tag; T is None when there is no usable pose.

    Planar PnP has two candidate poses; when their reprojection errors are close (a small or face-on
    tag) the orientation can flip. Those come back as (None, 'ambiguous'), or with keep_ambiguous as
    (best T, 'ambiguous'): its position is still good, its orientation isn't.
    """
    h = size / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, corners, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error as e:
        return None, f'pnp failed ({e.msg if hasattr(e, "msg") else e})'
    if n < 1:
        return None, 'pnp failed'
    errs = np.asarray(errs).reshape(-1)
    order = np.argsort(errs)
    best = order[0]
    T = rt_to_mat(rvecs[best], tvecs[best])
    if T[2, 3] <= 0:
        return None, 'behind camera'
    if n > 1 and errs[order[1]] > 1e-9 and errs[best] / errs[order[1]] > max_ambiguity:
        return (T if keep_ambiguous else None), 'ambiguous'
    return T, f'err {errs[best]:.2f}px'


def side_px(corners):
    """Mean edge length of the tag in the image (pixels): small tags give poor poses."""
    return float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]))


# --------------------------------------------------------------------------------------------- maths
def rt_to_mat(rvec, tvec):
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))
    T[:3, 3] = np.asarray(tvec, float).reshape(3)
    return T


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    """(x, y, z, w) from a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def pose_to_mat(p):
    """geometry_msgs Pose → 4x4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    T[:3, 3] = (p.position.x, p.position.y, p.position.z)
    return T


def tf_to_mat(t):
    """geometry_msgs Transform → 4x4."""
    T = np.eye(4)
    r = t.rotation
    T[:3, :3] = quat_to_mat(r.x, r.y, r.z, r.w)
    T[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
    return T


def rpy_to_mat(roll, pitch, yaw):
    """ROS fixed-axis roll-pitch-yaw (R = Rz(yaw) Ry(pitch) Rx(roll)), as tf2's static publisher uses."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def mat_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


def planar(T):
    """(x, y, yaw, z, tilt) of a robot-base pose: yaw from its x axis, tilt = how far its z axis leans."""
    yaw = math.atan2(T[1, 0], T[0, 0])
    tilt = math.acos(max(-1.0, min(1.0, T[2, 2])))
    return T[0, 3], T[1, 3], yaw, T[2, 3], tilt


def planar_mat(x, y, yaw):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = x, y
    return T


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def average_rotation(Rs, weights):
    """Weighted mean rotation (quaternion eigenvector method)."""
    M = np.zeros((4, 4))
    for R, w in zip(Rs, weights):
        q = np.array(mat_to_quat(R))
        M += w * np.outer(q, q)
    vals, vecs = np.linalg.eigh(M)
    q = vecs[:, int(np.argmax(vals))]
    return quat_to_mat(*q)


# Optical frame (x right, y down, z forward) relative to the camera body frame (x forward, z up).
R_BODY_OPTICAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


# --------------------------------------------------------------------------------------------- planar fix
def tag_corners(T_tag, size):
    """The 4 corners (4x3) of a tag at pose T_tag, in that pose's parent frame."""
    h = size / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
    return (T_tag[:3, :3] @ obj.T).T + T_tag[:3, 3]


def normalized(points_cam):
    """Pinhole projection without intrinsics: (x/z, y/z) per point."""
    return points_cam[:, :2] / points_cam[:, 2:3]


def planar_fix(obs, T_base_cam, x0, iters=15):
    """Robot pose (x, y, yaw) in the map that best explains the tag corners seen, keeping the camera's
    height, pitch and roll fixed at their mounted values (the robot is on the floor).

    obs: [(corners_map 4x3, corners_seen normalized 4x2)]. A single tag's full 6-DOF pose mixes up
    tilt and position when seen face-on (20 cm / 4° errors at 3 m in simulation); with only x, y, yaw
    free the same corners pin the pose down. Returns (x, y, yaw, rms in normalized units) or None.
    """
    P = np.vstack([c for c, _ in obs])
    N = np.vstack([n for _, n in obs]).reshape(-1)
    T_cam_base = inv(T_base_cam)

    def residual(p):
        T_cam_map = T_cam_base @ inv(planar_mat(*p))
        pc = (T_cam_map[:3, :3] @ P.T).T + T_cam_map[:3, 3]
        if np.any(pc[:, 2] <= 0.05):
            return None
        return normalized(pc).reshape(-1) - N

    p = np.array(x0, float)
    r = residual(p)
    if r is None:
        return None
    for _ in range(iters):
        J = np.empty((len(r), 3))
        for k, eps in enumerate((1e-4, 1e-4, 1e-5)):
            dp = p.copy()
            dp[k] += eps
            rk = residual(dp)
            if rk is None:
                return None
            J[:, k] = (rk - r) / eps
        try:
            step = np.linalg.lstsq(J, -r, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None
        p = p + step
        p[2] = wrap(p[2])
        r = residual(p)
        if r is None:
            return None
        if np.linalg.norm(step[:2]) < 1e-5 and abs(step[2]) < 1e-6:
            break
    return p[0], p[1], p[2], float(np.sqrt(np.mean(r ** 2)))
