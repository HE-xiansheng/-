
import rclpy,time,json,statistics
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu
rclpy.init();node=rclpy.create_node('imu_qos_readonly_comparison',enable_rosout=False)
rows={'reliable':[],'best_effort':[]};subs=[]
def callback(key):
 def receive(m):
  now=node.get_clock().now().nanoseconds;stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  rows[key].append([time.monotonic(),stamp,(now-stamp)*1e-9])
 return receive
for key,rel in [('reliable',ReliabilityPolicy.RELIABLE),('best_effort',ReliabilityPolicy.BEST_EFFORT)]:
 subs.append(node.create_subscription(Imu,'/livox/imu',callback(key),QoSProfile(depth=10,reliability=rel)))
begin=time.monotonic();wall=time.time()
while time.monotonic()-begin<25:rclpy.spin_once(node,timeout_sec=.02)
result={}
for key,r in rows.items():
 ages=sorted(x[2] for x in r)
 result[key]=dict(count=len(r),hz=(len(r)-1)/(r[-1][0]-r[0][0]) if len(r)>1 else None,age_min_median_p95_max_s=[ages[0],statistics.median(ages),ages[round((len(ages)-1)*.95)],ages[-1]] if ages else [],unique=len({x[1] for x in r}))
print(json.dumps(dict(started_wall=wall,result=result,rows=rows)))
node.destroy_node();rclpy.try_shutdown()
