"""传感器字段偏移、无效输入和新鲜度回归。"""
import struct
import unittest
from telemetry_reader import sensor_values
from preflight_status import sensor_snapshot

class Sensors(unittest.TestCase):
    def header(self):return struct.pack('<IIII',1,10,20,3)+b'imu'
    def test_imu(self):
        raw=self.header()+struct.pack('<37d',*range(37))
        values=sensor_values(raw,'sensor_msgs/Imu')
        self.assertEqual(values['angular_velocity'],[13,14,15])
        self.assertEqual(values['acceleration'],[25,26,27])
        self.assertEqual(sensor_values(raw[:30],'sensor_msgs/Imu'),{})
    def test_pressure_and_cloud(self):
        self.assertEqual(sensor_values(self.header()+struct.pack('<dd',101500,0),'sensor_msgs/FluidPressure'),{'pressure_pa':101500})
        self.assertEqual(sensor_values(self.header()+struct.pack('<d',float('nan')),'sensor_msgs/FluidPressure'),{})
        self.assertEqual(sensor_values(self.header()+struct.pack('<QIB',0,20300,1),'livox_ros_driver2/CustomMsg'),{'point_count':20300})
        self.assertEqual(sensor_values(self.header()+struct.pack('<II',2,100),'sensor_msgs/PointCloud2'),{'point_count':200})
    def test_expiry(self):
        row={'receipt_age_s':.1,'source_age_s':.1,'limit_s':.5,'values':{'acceleration':[1,2,3]}}
        self.assertTrue(sensor_snapshot({'topics':{'imu':row}})['imu']['ok'])
        self.assertFalse(sensor_snapshot({'topics':{'imu':row},'_transport_delay_s':1})['imu']['ok'])
        self.assertFalse(sensor_snapshot({})['pressure']['ok'])

if __name__=='__main__':unittest.main()
