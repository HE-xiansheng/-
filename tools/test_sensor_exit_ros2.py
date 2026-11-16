#!/usr/bin/env python3
"""Exercise the candidate's actual exit handler with harmless processes."""
import importlib.util,json,os,subprocess,sys,time
from pathlib import Path
from launch import LaunchContext,LaunchDescription,LaunchService
from launch.actions import ExecuteProcess
assert os.environ['ROS_LOCALHOST_ONLY']=='1' and os.environ['ROS_DOMAIN_ID']=='81'
out=Path(os.environ['SENSOR_TEST_OUTPUT'])
spec=importlib.util.spec_from_file_location('candidate','/home/orangepi/drone_platform/02_drivers/sensors/launch/platform_sensors.launch.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
ctx=LaunchContext()
ctx.launch_configurations.update({'enable_'+k:str(k=='rtsp').lower() for k in ['mavros','camera','usb_camera','lidar','rtsp']})
ctx.launch_configurations.update(fcu_url='unused',video_device='unused',device_config='unused',rtsp_url='unused')
m.Node=lambda **kw:ExecuteProcess(cmd=[sys.executable,'-c','import time;time.sleep(2);raise SystemExit(7)'])
actions=m.start(ctx)
pidfile=out/'sentinel.pid'
sentinel=ExecuteProcess(cmd=[sys.executable,'-c','import os,time;from pathlib import Path;Path('+repr(str(pidfile))+').write_text(str(os.getpid()));time.sleep(90)'])
service=LaunchService();service.include_launch_description(LaunchDescription([actions[0],sentinel]+actions[1:]))
rc=service.run();pid=int(pidfile.read_text());clean=not Path('/proc/'+str(pid)).exists()
result={'returncode':rc,'remaining_sentinel':not clean,'passed':rc!=0 and clean}
(out/'unexpected-exit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));assert result['passed']
