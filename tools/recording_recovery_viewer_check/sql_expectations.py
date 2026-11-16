#!/usr/bin/env python3
"""Independent minimal Odometry CDR reader for the crash fixture; no Qt code reused."""
import json,sqlite3,struct,sys
from pathlib import Path
folder=Path(sys.argv[1]).resolve();poses=[];stamps=[]
for f in folder.glob('*.db3'):
 with sqlite3.connect(f.as_uri()+'?mode=ro',uri=True) as db:
  for topic,payload in db.execute('SELECT topics.name,messages.data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name IN (?,?)',('/factory_fastlio/odometry','/factory_fastlio/cloud_world')):
   assert payload[:4]==b'\x00\x01\x00\x00';sec,nsec=struct.unpack_from('<iI',payload,4);ns=sec*10**9+nsec;stamps.append(ns)
   if topic.endswith('odometry'):
    offset=12
    for i in range(2):
     offset=4+((offset-4+3)//4)*4;size=struct.unpack_from('<I',payload,offset)[0];offset+=4+size
    offset=4+((offset-4+7)//8)*8
    poses.append((ns,struct.unpack_from('<ddd',payload,offset)))
poses.sort();print(json.dumps(dict(files=[str(p) for p in sorted(folder.glob('*.db3'))],pose_count=len(poses),first_xyz=poses[0][1],last_xyz=poses[-1][1],duration_ms=(max(stamps)-min(stamps))//1000000),indent=2))
