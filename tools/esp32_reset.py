#!/usr/bin/env python3
# Written 2026-10-01 (new tool, not a reconstruction).
"""Reset the ESP32-S3 into its firmware through the native USB port (USB-Serial/JTAG), safely.

On the S3's native USB the two modem lines are commands to the chip (see esptool's reset code):

    DTR RTS
     1   1   nothing (Linux sets this whenever a program opens the port)
     1   0   ARM download mode for the next reset   <- pyserial's `rts = False` while DTR is up
     0   1   hold the chip in reset
     0   0   disarm download mode / release the reset

This port is -hupcl (closing it doesn't drop the lines), so an armed flag survives until the next
reset, which then boots the ROM loader ("waiting for download") instead of the firmware. On
2026-10-01 that left the robot silent: no micro-ROS session, no /odom. This script only ever sets
both lines in one ioctl, so it never passes through "1 0".

  python3 tools/esp32_reset.py [/dev/esp32]      # reset; if no other program has the port open,
                                                 # also read the ROM banner to confirm the boot mode
Exit status: 0 = firmware booting (or reset sent), 1 = stuck in download mode, 2 = port problem.
"""
import fcntl
import os
import re
import struct
import sys
import termios
import time

DTR, RTS = termios.TIOCM_DTR, termios.TIOCM_RTS


def set_lines(fd, dtr, rts):
    fcntl.ioctl(fd, termios.TIOCMSET, struct.pack('I', (DTR if dtr else 0) | (RTS if rts else 0)))


def port_in_use(dev):
    """True if another process has the port open (reading the banner would steal its data)."""
    real = os.path.realpath(dev)
    me = str(os.getpid())
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if pid == me:
            continue
        try:
            for fd in os.listdir(f'/proc/{pid}/fd'):
                if os.path.realpath(f'/proc/{pid}/fd/{fd}') == real:
                    return True
        except OSError:
            pass
    return False


def reset(fd):
    set_lines(fd, 0, 0)          # disarm a stale download flag first
    time.sleep(0.05)
    set_lines(fd, 0, 1)          # reset
    time.sleep(0.1)
    set_lines(fd, 0, 0)          # release: normal boot


def read_banner(fd, secs=1.0):
    buf, end = b'', time.time() + secs
    while time.time() < end:
        try:
            buf += os.read(fd, 4096)
        except BlockingIOError:
            time.sleep(0.02)
        if b'boot:0x' in buf and b'\n' in buf[buf.index(b'boot:0x'):]:
            break
    return buf.decode(errors='replace')


def main():
    dev = next((a for a in sys.argv[1:] if not a.startswith('-')), '/dev/esp32')
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        print(f'✗ cannot open {dev}: {e}', file=sys.stderr)
        return 2
    try:
        check = not port_in_use(dev)
        if check:
            termios.tcflush(fd, termios.TCIFLUSH)
        for attempt in (1, 2):
            reset(fd)
            if not check:
                print(f'reset sent to {dev} (port in use by another program, boot mode not checked)')
                return 0
            text = read_banner(fd)
            m = re.search(r'boot:(0x[0-9a-f]+) \(([^)]*)\)', text)
            if not m:
                print(f'reset sent to {dev}; no ROM banner seen (boot mode unknown)')
                return 0
            if 'DOWNLOAD' not in m.group(2):
                print(f'✓ ESP32 reset into its firmware (boot:{m.group(1)} {m.group(2)})')
                return 0
            print(f'! attempt {attempt}: the ESP32 booted the ROM loader (boot:{m.group(1)} {m.group(2)})')
        print('✗ still in download mode. Is BOOT/IO0 held low? Unplug the ESP32 USB for 3 s and plug it back in.',
              file=sys.stderr)
        return 1
    except OSError as e:
        print(f'✗ {dev}: {e}', file=sys.stderr)
        return 2
    finally:
        try:
            set_lines(fd, 1, 1)  # idle; "1 1" is a no-op for the chip
        except OSError:
            pass
        os.close(fd)


if __name__ == '__main__':
    sys.exit(main())
