#!/usr/bin/env python3
"""Recover only registered domain183 children after a live-check owner exits."""
import json
import os
from pathlib import Path
import signal
import sys
import time

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('/tmp/livox-live-check')
domains = {Path('/tmp/livox-live-check'): 183, Path('/tmp/livox-native-check'): 183,
           Path('/tmp/livox-shutdown-debug'): 185, Path('/tmp/px4-ev-ground-check'): 187,
           Path('/tmp/new-camera-ground-check'): 189, Path('/tmp/platform-soak-check'): 83}
assert root in domains
registry = root/'owned-processes.json'
owned = json.loads(registry.read_text()) if registry.exists() else []
groups = {int(entry['pid']) for entry in owned}


def members():
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            pid = int(proc.name)
            if os.getpgid(pid) not in groups:
                continue
            stat = (proc/'stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] == 'Z':
                continue
            environment = (proc/'environ').read_bytes().split(b'\0')
            domain = ('ROS_DOMAIN_ID='+str(domains[root])).encode()
            if domain not in environment or (proc/'cwd').resolve() != root:
                raise RuntimeError('refusing unowned group member '+str(pid))
            # A group leader must still be the exact registered process.
            if pid in groups:
                expected = next(e['start_ticks'] for e in owned if e['pid'] == pid)
                if stat[19] != expected:
                    raise RuntimeError('refusing reused group leader '+str(pid))
            found.append({'pid': pid, 'pgid': os.getpgid(pid)})
        except (FileNotFoundError, ProcessLookupError):
            continue
    return found


before = members()
for sig, seconds in [(signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)]:
    current = members()
    if not current:
        break
    for pgid in {p['pgid'] for p in current}:
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline and members():
        time.sleep(.1)
remaining = members()
report = {'before': before, 'remaining': remaining, 'passed': not remaining}
(root/'owned-cleanup.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report))
raise SystemExit(0 if report['passed'] else 1)
