#!/usr/bin/env python3
"""只读获取真实ROS2 Livox CDR及官方解码耗时；不在机载保存文件。"""
import base64
import json
from pathlib import Path
import shlex
import subprocess
import sys


REMOTE = r'''
import base64,gc,json,statistics,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message,serialize_message
from livox_ros_driver2.msg import CustomMsg
rclpy.init(args=[])
node=rclpy.create_node('livox_metadata_readonly_probe',enable_rosout=False)
samples=[]
sub=node.create_subscription(CustomMsg,'/livox/lidar',lambda data:samples.append(data),qos_profile_sensor_data,raw=True)
deadline=time.monotonic()+15
try:
    while len(samples)<1 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
    if not samples:raise RuntimeError('no live lidar message')
finally:
    node.destroy_node();rclpy.shutdown()
raw=samples[0];durations=[]
for index in range(5):
    start=time.perf_counter_ns();msg=deserialize_message(raw,CustomMsg)
    durations.append((time.perf_counter_ns()-start)/1e6)
    if index==0:
        header={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec,
                'frame_id':msg.header.frame_id,'timebase':msg.timebase,
                'point_num':msg.point_num,'points_length':len(msg.points),'lidar_id':msg.lidar_id}
        roundtrip=serialize_message(msg)==raw
    del msg;gc.collect()
print(json.dumps({'header':header,'bytes':len(raw),'official_deserialize_ms':durations,
                  'official_roundtrip_identical':roundtrip,'raw_base64':base64.b64encode(raw).decode()}))
'''


def main():
    target = Path(sys.argv[1])
    target.mkdir(parents=True, exist_ok=True)
    remote = 'bash -c ' + shlex.quote('source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=0 exec /usr/bin/python3 -u -')
    command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
               '-o', 'UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
               '-o', 'HostKeyAlias=opi-ros2-emmc', 'orangepi@192.168.1.224', remote]
    result = subprocess.run(command, input=REMOTE, text=True, capture_output=True, timeout=45)
    (target/'probe.stderr.log').write_text(result.stderr)
    result.check_returncode()
    value = json.loads(result.stdout)
    (target/'live-livox.cdr').write_bytes(base64.b64decode(value.pop('raw_base64')))
    (target/'official-probe.json').write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(value, ensure_ascii=False))


if __name__ == '__main__':
    main()
