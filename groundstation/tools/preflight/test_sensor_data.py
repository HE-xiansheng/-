"""ROS2类型化传感器小字段及显示新鲜度回归，无ROS环境也可运行。"""
import unittest
from types import SimpleNamespace as S
from telemetry_reader import sensor_values
from preflight_status import sensor_snapshot

class Sensors(unittest.TestCase):
    def test_imu(self):
        msg=S(angular_velocity=S(x=1.,y=2.,z=3.),linear_acceleration=S(x=4.,y=5.,z=6.))
        self.assertEqual(sensor_values(msg,'imu'),{'angular_velocity':[1.,2.,3.],'acceleration':[4.,5.,6.]})
        msg.angular_velocity.x=float('nan');self.assertEqual(sensor_values(msg,'imu'),{})
    def test_pressure_and_cloud(self):
        self.assertEqual(sensor_values(S(fluid_pressure=101500.),'pressure'),{'pressure_pa':101500.})
        self.assertEqual(sensor_values(S(fluid_pressure=float('inf')),'pressure'),{})
        self.assertEqual(sensor_values(S(point_num=20300),'lidar'),{'point_count':20300})
        self.assertEqual(sensor_values(S(height=3,width=5),'cloud'),{'point_count':15})
    def test_sensor_expiry(self):
        row={'topic':'/livox/imu','source_age_s':.1,'receipt_age_s':.05,'limit_s':.4,'values':{'acceleration':[0,0,9.8]}}
        packet={'topics':{'imu':row}}
        self.assertTrue(sensor_snapshot(packet)['imu']['ok'])
        packet['_transport_delay_s']=.35;self.assertFalse(sensor_snapshot(packet)['imu']['ok'])
        packet['_transport_delay_s']=0;row['source_age_s']=2;self.assertFalse(sensor_snapshot(packet)['imu']['ok'])

if __name__=='__main__':unittest.main(verbosity=2)
