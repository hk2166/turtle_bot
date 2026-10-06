#!/usr/bin/env python3
# Written 2026-10-06: one frame through the robot's eyes, for terminal-driven missions.
"""Grab one camera frame and save it as a JPEG.

    python3 tools/look.py [out.jpg]       default: ~/.campus_bot/look/look-<time>.jpg (path printed)

Works alongside navigation as long as nodes/camera_node.py is NOT holding the camera
(run/camera.sh off). Waits briefly for auto-exposure so the frame isn't dark.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
import cv2  # noqa: E402
import tags_common as tc  # noqa: E402

out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    time.strftime('~/.campus_bot/look/look-%H%M%S.jpg'))
busy = tc.camera_busy()
if busy:
    sys.exit(f'camera in use by PID {busy} (run/camera.sh off frees it)')
cap = tc.open_camera()
if cap is None:
    sys.exit('cannot open the camera (unplugged, or grabbed by a desktop app)')
for _ in range(10):
    cap.read()
ok, img = cap.read()
cap.release()
if not ok:
    sys.exit('camera read failed')
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
cv2.imwrite(out, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
print(out)
