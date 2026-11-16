#!/usr/bin/env python3
"""从机载真实归档提取轻量分包，ROS只用于生成测试基线，正式查看器不依赖ROS。"""
import json, sys
from pathlib import Path
import rosbag
from geometry_msgs.msg import PoseStamped
source=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
with rosbag.Bag(str(source)) as bag:
    midpoint=(bag.get_start_time()+bag.get_end_time())/2
    cloud_index=0;poses=[];points=0;times=[]
    with rosbag.Bag(str(out/'part-0.bag'),'w',compression='lz4') as a,rosbag.Bag(str(out/'part-1.bag'),'w',compression='bz2') as b:
        for topic,m,t in bag.read_messages(topics=['/factory_fastlio/cloud_world','/factory_fastlio/odometry']):
            target=a if t.to_sec()<midpoint else b
            if topic.endswith('cloud_world'):
                cloud_index+=1
                if cloud_index%10!=1:continue
                points+=m.width*m.height
            else:
                poses.append([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]);times.append(m.header.stamp.to_sec())
                bad=PoseStamped();bad.header=m.header;bad.pose.position.x=999;bad.pose.orientation.w=1
                target.write('/mavros/local_position/pose',bad,t)
            target.write(topic,m,t)
    (out/'expected.json').write_text(json.dumps({'pose_count':len(poses),'point_records':points,'duration_ms':round((times[-1]-times[0])*1000),'first':poses[0],'last':poses[-1]},indent=2))
