-- campus_bot (2026-10-30): WirePlumber 0.4 (Ubuntu 24.04 ships 0.4.17) reads Lua files from
-- ~/.config/wireplumber/main.lua.d/, NOT the 0.5-style wireplumber.conf.d/*.conf next to it, so
-- wireplumber-no-camera.conf never took effect here and PipeWire kept the cameras open (seen holding
-- the Brio, 2026-10-30). No desktop app on this machine needs a camera: turn both monitors off, so
-- only the robot's software opens them (camera_node: the Brio; the face service: the HIKVISION).
-- Installed by setup/05_remote_access.sh as main.lua.d/51-campus-bot-no-camera.lua.
v4l2_monitor.enabled = false
libcamera_monitor.enabled = false
