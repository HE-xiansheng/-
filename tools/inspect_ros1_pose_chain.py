#!/usr/bin/env python3
"""Observe source pose chain; subscribe only, stream result to the computer."""
import json, math, os, tempfile, threading, time
from collections import Counter
with tempfile.TemporaryDirectory(prefix='pose-observer-', dir='/dev/shm') as temporary:
    os.environ['ROS_LOG_DIR'] = temporary
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    rospy.init_node('ros2_migration_pose_observer', anonymous=True, disable_signals=True)
    samples={k:[] for k in ('odom','vision','local','state')}
    lock=threading.Lock()
    def observe(msg,key):
        entry={'receipt':time.monotonic(),'stamp_ns':msg.header.stamp.to_nsec(),'age':(rospy.Time.now()-msg.header.stamp).to_sec(),'frame':msg.header.frame_id}
        if key=='state':entry.update(connected=msg.connected,armed=msg.armed,mode=msg.mode)
        else:
            p=msg.pose.pose if key=='odom' else msg.pose
            entry.update(position=[p.position.x,p.position.y,p.position.z],quaternion=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
            if key=='odom':entry['child_frame']=msg.child_frame_id
        with lock:samples[key].append(entry)
    subscribers=[rospy.Subscriber(topic,typ,observe,callback_args=key,queue_size=100,tcp_nodelay=True) for key,topic,typ in [('odom','/factory_fastlio/odometry',Odometry),('vision','/mavros/vision_pose/pose',PoseStamped),('local','/mavros/local_position/pose',PoseStamped),('state','/mavros/state',State)]]
    time.sleep(10)
    for sub in subscribers:sub.unregister()
    rospy.signal_shutdown('observation complete')
    def summarize(rows):
        stamps=[r['stamp_ns'] for r in rows]
        return {'count':len(rows),'receipt_hz':(len(rows)-1)/(rows[-1]['receipt']-rows[0]['receipt']) if len(rows)>1 else None,'unique_stamps':len(set(stamps)),'stamp_regressions':sum(b<a for a,b in zip(stamps,stamps[1:])),'frames':dict(Counter(r['frame'] for r in rows)),'age_min_max_seconds':[min(r['age'] for r in rows),max(r['age'] for r in rows)] if rows else None,'first':rows[0] if rows else None,'last':rows[-1] if rows else None}
    result={'topics':{k:summarize(v) for k,v in samples.items()},'read_only_observation':True,'flight_commands_sent':False}
    odom=samples['odom'];vision=samples['vision']
    if odom and vision:
        def pose_error(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a['position'],b['position'])))+min(math.sqrt(sum((x-s*y)**2 for x,y in zip(a['quaternion'],b['quaternion']))) for s in (-1,1))
        # Compare values against every observed odometry frame; this does not infer a dynamic transform.
        errors=[min(pose_error(v,o) for o in odom) for v in vision]
        stamps={o['stamp_ns'] for o in odom}
        result['comparison']={'vision_poses_exactly_matching_observed_odom':sum(e<1e-8 for e in errors),'vision_stamps_matching_observed_odom':sum(v['stamp_ns'] in stamps for v in vision),'pose_error_min_max':[min(errors),max(errors)],'limitations':'Short observation can identify repeated/copied samples, but cannot prove ENU/NED axes or frame alignment.'}
    print(json.dumps(result,ensure_ascii=False,indent=2))
