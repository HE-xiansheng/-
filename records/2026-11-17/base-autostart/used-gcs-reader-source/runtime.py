"""机载ROS2公共运行时。由自包含payload装载，不在地面站导入rclpy。"""
import base64
import json
import math
import os
import sys
import threading
import time


def configuration():
    return json.loads(base64.b64decode(os.environ.get('GCS_ROS2_CONFIG','e30=')).decode())


def stamp_seconds(stamp):
    return int(stamp.sec)+int(stamp.nanosec)*1e-9


class Runtime:
    def __init__(self, name):
        # ROS日志必须走stderr：stdout可能是VID1二进制或JSON控制回执。
        os.environ['RCUTILS_LOGGING_USE_STDOUT']='0'
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        self.rclpy=rclpy
        self.config=configuration()
        rclpy.init(args=[])
        self.node=Node(name+'_'+str(os.getpid()), enable_rosout=False,
                       automatically_declare_parameters_from_overrides=False)
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.subscriptions=[]
        self.stopped=threading.Event()
        self.thread=threading.Thread(target=self._spin,daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as exc:
            if not self.stopped.is_set():
                print('ROS2 executor: '+str(exc),file=sys.stderr,flush=True)
                self.stopped.set()

    def topic(self,key,default):
        return self.config.get('topics',{}).get(key,default)

    def qos(self,key,default='best_effort',depth=1):
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
        row=self.config.get('qos',{}).get(key,{})
        reliability=row.get('reliability',default)
        durability=row.get('durability','volatile')
        if reliability not in ('reliable','best_effort') or durability not in ('volatile','transient_local'):
            raise ValueError('无效QoS配置: '+key)
        count=int(row.get('depth',depth))
        if not 1<=count<=1000:raise ValueError('无效QoS深度: '+key)
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=count,
                          reliability=ReliabilityPolicy.RELIABLE if reliability=='reliable' else ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL if durability=='transient_local' else DurabilityPolicy.VOLATILE)

    def subscribe(self,typ,topic,callback,key='',default='best_effort',depth=1):
        sub=self.node.create_subscription(typ,topic,callback,self.qos(key,default,depth))
        self.subscriptions.append(sub)
        return sub

    def now(self):
        return self.node.get_clock().now().nanoseconds*1e-9

    def ok(self):
        return self.rclpy.ok() and not self.stopped.is_set()

    def publishers(self,topics):
        result={}
        for topic in topics:
            endpoints=self.node.get_publishers_info_by_topic(topic)
            result[topic]=[{'node':('/'+v.node_namespace.strip('/')+'/'+v.node_name).replace('//','/'),
                            'type':v.topic_type,
                            'reliability':v.qos_profile.reliability.name.lower(),
                            'durability':v.qos_profile.durability.name.lower()} for v in endpoints]
        return result

    def close(self):
        if not self.stopped.is_set():self.stopped.set()
        self.executor.shutdown(timeout_sec=2.)
        if self.thread is not threading.current_thread():self.thread.join(timeout=3.)
        self.node.destroy_node()
        if self.rclpy.ok():self.rclpy.shutdown()
