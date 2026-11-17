#!/usr/bin/env python3
"""以官方Humble序列化器和现场CDR为独立参照，检查飞前元数据解析。"""
import ast
import base64
import json
from pathlib import Path
import shlex
import subprocess
import sys


CHECK = r'''
import json,statistics,time
from rclpy.serialization import serialize_message,deserialize_message
from livox_ros_driver2.msg import CustomMsg,CustomPoint
checks={}
def check(name,condition):checks[name]=bool(condition)
def reference(msg):
    return {'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec,
            'frame':msg.header.frame_id,'point_count':msg.point_num,'timebase':msg.timebase}
begin=time.perf_counter_ns();decoded=deserialize_message(live,CustomMsg)
official_ms=(time.perf_counter_ns()-begin)/1e6
check('live metadata equals official decoder',livox_metadata(live)==reference(decoded))
check('live bytes roundtrip through official serializer',serialize_message(decoded)==live)
durations=[]
for _ in range(2000):
    begin=time.perf_counter_ns();livox_metadata(live);durations.append((time.perf_counter_ns()-begin)/1e6)
median_ms=statistics.median(durations)
check('metadata work below one millisecond and tenfold faster',median_ms<1. and median_ms*10<official_ms)
fixtures=0
for frame in ('','a','abc','1234567','12345678','123456789','雷达坐标系'):
    for count in (0,1,3):
        msg=CustomMsg();msg.header.stamp.sec=-1;msg.header.stamp.nanosec=999999999
        msg.header.frame_id=frame;msg.timebase=1234567890123456789;msg.point_num=count
        msg.points=[CustomPoint(offset_time=i,x=1.25,y=-2.5,z=3.75,reflectivity=20,tag=3,line=1) for i in range(count)]
        raw=serialize_message(msg)
        check('official serializer alignment frame '+repr(frame)+' points '+str(count),livox_metadata(raw)==reference(msg))
        fixtures+=1
# 大端CDR1经官方反序列化器独立核对；空点数组避免自写点编码器。
big=b'\0\0\0\0'+struct.pack('>iII',-7,42,4)+b'map\0'+struct.pack('>QIB3sI',12345,0,2,b'\0\0\0',0)
check('big endian official decoder agrees',livox_metadata(big)==reference(deserialize_message(big,CustomMsg)))
frame_length=struct.unpack_from('<I',live,12)[0];frame_end=16+frame_length
offset=4+((frame_end-4+7)//8)*8
cases={}
for length in (0,3,16,frame_end-1,offset+19,len(live)-1):cases['truncated '+str(length)]=live[:length]
cases['extra tail']=live+b'\0'
def replaced(name,offset,contents):
    value=bytearray(live);value[offset:offset+len(contents)]=contents;cases[name]=value
replaced('unknown encapsulation',0,b'\0\3\0\0')
replaced('invalid nanoseconds',8,struct.pack('<I',1000000000))
replaced('empty CDR string',12,struct.pack('<I',0))
replaced('oversized CDR string',12,struct.pack('<I',0xffffffff))
replaced('missing string terminator',frame_end-1,b'x')
replaced('embedded string null',16,b'\0')
replaced('invalid UTF8',16,b'\xff')
replaced('point count mismatch',offset+8,struct.pack('<I',1))
replaced('oversized sequence',offset+16,struct.pack('<I',0xffffffff))
for name,data in cases.items():
    try:livox_metadata(data)
    except ValueError:check('reject '+name,True)
    else:check('reject '+name,False)
print(json.dumps({'passed':all(checks.values()),'checks':checks,'official_fixtures':fixtures,
    'live_bytes':len(live),'live_point_count':decoded.point_num,'official_deserialize_ms':official_ms,
    'metadata_median_ms':median_ms,'metadata_max_ms':max(durations),
    'speedup_median':official_ms/median_ms,'scope':'offline official serialization oracle, real captured CDR; no publishers or FCU commands'},ensure_ascii=False))
raise SystemExit(0 if all(checks.values()) else 1)
'''


def main():
    root = Path(__file__).resolve().parents[1]
    record = root/'records/2026-11-17/base-autostart/gcs-sampler-check'
    reader = root/'groundstation/tools/preflight/telemetry_reader.py'
    source = reader.read_text()
    definition = next(node for node in ast.parse(source).body if isinstance(node, ast.FunctionDef) and node.name == 'livox_metadata')
    parser_source = ast.get_source_segment(source, definition)
    live = base64.b64encode((record/'live-livox.cdr').read_bytes()).decode()
    program = 'import struct,base64\n'+parser_source+'\nlive=base64.b64decode('+repr(live)+')\n'+CHECK
    remote = 'bash -c '+shlex.quote('source /home/orangepi/drone_platform/env.sh && exec /usr/bin/python3 -u -')
    command = ['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=8',
               '-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
               '-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224',remote]
    result = subprocess.run(command,input=program,text=True,capture_output=True,timeout=35)
    (record/'metadata-oracle.stderr.log').write_text(result.stderr)
    if result.stdout:
        value=json.loads(result.stdout)
        (record/'metadata-oracle.json').write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(value,ensure_ascii=False))
    result.check_returncode()


if __name__ == '__main__':
    main()
