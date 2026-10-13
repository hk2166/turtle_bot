#!/usr/bin/env python3
# Written 2026-10-13: the dashboard's admin password.
"""Set (or change) the admin password of the robot's dashboard.

    python3 tools/set_admin_password.py          asks twice, hidden
Stores a salted PBKDF2 hash in ~/.campus_bot/admin.yaml (mode 600, never committed). Changing it
logs everyone out at the next dashboard restart. The call page (/call) needs no password.
"""
import getpass
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
from fleet import ADMIN_FILE, hash_password   # noqa: E402

p1 = getpass.getpass('new admin password: ')
if len(p1) < 6:
    sys.exit('use at least 6 characters')
if getpass.getpass('again: ') != p1:
    sys.exit('the two entries differ: nothing changed')
os.makedirs(os.path.dirname(ADMIN_FILE), exist_ok=True)
fd = os.open(ADMIN_FILE + '.tmp', os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
with os.fdopen(fd, 'w') as f:
    f.write('# dashboard admin password (salted PBKDF2-SHA256) - tools/set_admin_password.py\n')
    yaml.safe_dump(hash_password(p1), f)
os.replace(ADMIN_FILE + '.tmp', ADMIN_FILE)
print(f'saved to {ADMIN_FILE}. Log in at http://resp-desktop.local:8088/login')
