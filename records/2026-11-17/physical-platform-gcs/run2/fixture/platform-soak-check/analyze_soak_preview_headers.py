#!/usr/bin/env python3
"""Require every native-observed preview stamp in independent camera captures."""
import csv,json
from pathlib import Path

def analyze(out):
 out=Path(out)
 with (out/'gimbal-headers.csv').open() as f:rows=list(csv.DictReader(f))
 native={int(r['stamp_ns']) for r in rows if r['qos']=='reliable' and r['key']=='stereo'}
 preview={int(r['stamp_ns']) for r in rows if r['qos']=='reliable' and r['key']=='person'}
 python={int(json.loads(s)[1]) for s in (out/'color-headers.jsonl').read_text().splitlines()}
 missing_native=sorted(preview-native);missing_both=sorted(preview-(native|python))
 return dict(passed=bool(preview) and not missing_both,native_camera_count=len(native),python_camera_count=len(python),preview_count=len(preview),missing_from_native_camera=missing_native,missing_from_both_camera_observers=missing_both,corroborated_by_python=sorted((preview-native)&python),scope='Every native-observed preview stamp must occur in independent source-camera observation; no startup/end exemptions')

if __name__=='__main__':
 import sys
 result=analyze(sys.argv[1]);print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
