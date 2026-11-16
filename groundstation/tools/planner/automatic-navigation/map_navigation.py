#!/usr/bin/env python3
"""直接更新原生规划器目标；不使用人工确认、令牌或候选复查。"""
import argparse,json,math,os,threading,time

def main():
    parser=argparse.ArgumentParser()
    sub=parser.add_subparsers(dest='action',required=True)
    sub.add_parser('preview').add_argument('xyz',nargs=3,type=float)
    sub.add_parser('status')
    args=parser.parse_args()
    if args.action=='preview' and not all(math.isfinite(v) for v in args.xyz):
        raise ValueError('目标坐标不是有限数值')
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('diff_planner_navigation_client',anonymous=True,disable_signals=True)
    updates=[];lock=threading.Lock()
    def receive(msg):
        try:
            value=json.loads(msg.data)
            with lock:updates.append(value)
        except (ValueError,TypeError):pass
    reader=rospy.Subscriber('/diff_planner/manual_preview/status',String,receive,queue_size=10)
    def wait(predicate,timeout=8):
        end=time.monotonic()+timeout
        while not rospy.is_shutdown() and time.monotonic()<end:
            with lock:rows=list(updates);updates.clear()
            for value in rows:
                if predicate(value):return value
            time.sleep(.02)
        raise RuntimeError('未收到目标更新回执，请查看机载规划器状态')
    if args.action=='status':
        print(json.dumps(wait(lambda s:True),ensure_ascii=False));return
    publisher=rospy.Publisher('/diff_planner/navigation/preview_goal',PoseStamped,queue_size=1)
    end=time.monotonic()+3
    while publisher.get_num_connections()==0 and time.monotonic()<end:time.sleep(.02)
    if publisher.get_num_connections()==0:raise RuntimeError('原生规划器目标入口未运行')
    msg=PoseStamped();msg.header.stamp=rospy.Time.now();msg.header.frame_id='world';msg.pose.orientation.w=1.
    msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=args.xyz
    publisher.publish(msg)
    print(json.dumps(wait(lambda s:s.get('has_goal') and s.get('goal')==args.xyz),ensure_ascii=False))

if __name__=='__main__':main()
