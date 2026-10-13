#!/usr/bin/env python3
# Written 2026-10-13: clear the RPLIDAR C1's "internal error" without unplugging it.
"""Check the lidar's health and, if it reports an error, software-reset it.

    python3 tools/lidar_reset.py [--port /dev/lidar] [--force]
exit 0 = health GOOD (after a reset if one was needed), 1 = still bad / no answer.

sllidar_ros2 exits on "health status : 2 ... Please reboot the device" and never tries the protocol's
RESET (0xA5 0x40). That command alone brought a C1 back to GOOD on 2026-10-13, where before the fix
was unplugging its USB stick. run/remap.sh runs this before starting the sensors. The port must be
free (no sllidar_node running).
"""
import argparse
import sys
import time

import serial

STATUS = {0: 'GOOD', 1: 'WARNING', 2: 'ERROR'}


def health(s):
    s.reset_input_buffer()
    s.write(bytes([0xA5, 0x52]))                  # GET_HEALTH
    r = s.read(10)                                # 7-byte descriptor + status, error code (u16)
    if len(r) < 10 or r[:2] != b'\xa5\x5a':
        return None, None
    return r[7], r[8] | (r[9] << 8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/lidar')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--force', action='store_true', help='reset even if healthy')
    a = ap.parse_args()
    try:
        s = serial.Serial(a.port, a.baud, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f'lidar port: {e}')
    s.dtr = False
    s.write(bytes([0xA5, 0x25]))                  # STOP any scan left running
    time.sleep(0.05)
    st, code = health(s)
    print(f'lidar health: {STATUS.get(st, "no answer")}' + (f' (code {code})' if code else ''))
    if st != 0 or a.force:
        s.write(bytes([0xA5, 0x40]))              # RESET: the core reboots (~2 s)
        time.sleep(2.5)
        st, code = health(s)
        print(f'after reset: {STATUS.get(st, "no answer")}' + (f' (code {code})' if code else ''))
    s.close()
    sys.exit(0 if st == 0 else 1)


if __name__ == '__main__':
    main()
