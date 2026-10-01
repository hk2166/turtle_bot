#!/usr/bin/env python3
# RECONSTRUCTED from the handoff notes (2026-10-01); this version verified the C1 on the Pi that day.
"""RPLIDAR C1 health check without ROS: GET_INFO (A5 50), GET_HEALTH (A5 52), then a short scan (A5 20).

  python3 tools/lidar_check.py                 # /dev/lidar, else /dev/ttyUSB0, at 460800
  python3 tools/lidar_check.py /dev/cu.usbserial-0001   # on the Mac

Reading the result (the fork from the burnt-adapter incident):
  port missing            → the USB-TTL stick isn't enumerating (cable, stick)
  no reply at all         → head not spinning? check 5 V on red/black (~300 mA when alive);
                            spinning but silent → swap TXD↔RXD; confirm 460800, not 115200
  info + health OK, ~10 Hz → lidar and wiring are fine
"""
import os
import sys
import time

import serial

BAUD = 460800


def request(s, cmd, n):
    s.reset_input_buffer()
    s.write(bytes([0xA5, cmd]))
    return s.read(n)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else ('/dev/lidar' if os.path.exists('/dev/lidar') else '/dev/ttyUSB0')
    if not os.path.exists(port):
        sys.exit(f'✗ {port} does not exist: the USB-TTL stick is not enumerating.')
    s = serial.Serial(port, BAUD, timeout=1)
    s.write(b'\xA5\x25')            # STOP, in case it was left scanning
    time.sleep(0.1)

    r = request(s, 0x50, 27)
    if len(r) != 27 or r[:2] != b'\xa5\x5a':
        sys.exit(f'✗ no GET_INFO reply on {port} @ {BAUD} (got {r.hex() or "nothing"}).\n'
                 '  Head spinning? If yes, swap TXD↔RXD. If no, check 5 V / GND.')
    d = r[7:]
    print(f'✓ info: model 0x{d[0]:02x}  firmware {d[2]}.{d[1]:02d}  hardware {d[3]}  serial {d[4:].hex()}')

    r = request(s, 0x52, 10)
    status = r[7] if len(r) == 10 else None
    print({0: '✓ health: good', 1: '! health: WARNING', 2: '✗ health: ERROR'}.get(status, f'✗ health: no reply ({r.hex()})'),
          f'(code {r[8] | r[9] << 8})' if status else '')

    s.reset_input_buffer()
    s.write(b'\xA5\x20')
    s.timeout = 3
    hdr = s.read(7)
    buf = s.read(5 * 5000)        # ~1 s of samples at 5 kHz; also covers spin-up
    s.write(b'\xA5\x25')
    if hdr[:2] != b'\xa5\x5a' or not buf:
        sys.exit('✗ scan did not start (no samples). Motor stalled or underpowered?')
    nodes = [buf[i:i + 5] for i in range(0, len(buf) - 4, 5)]
    starts = valid = 0
    ranges = []
    for p in nodes:
        if (p[0] & 1) != ((p[0] >> 1) & 1) and p[1] & 1:
            valid += 1
            starts += p[0] & 1
            dist = (p[3] | p[4] << 8) / 4000.0
            if dist > 0:
                ranges.append(dist)
    secs = len(nodes) / 5000
    print(f'✓ scan: {len(nodes)} samples, {valid} well-formed, ~{starts / secs:.1f} Hz rotation, '
          f'{len(ranges)} with range ({min(ranges, default=0):.2f}–{max(ranges, default=0):.2f} m)')
    if valid < 0.9 * len(nodes):
        print('! many malformed samples: wrong baud, or a noisy/long TX line')


if __name__ == '__main__':
    main()
