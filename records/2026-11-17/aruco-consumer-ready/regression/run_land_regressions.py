import os,subprocess,signal,json
from pathlib import Path
root=Path('/tmp/land-consumer-regression')
env=dict(os.environ,ROS_DOMAIN_ID='87',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(root/'ros-log'))
with (root/'controller.log').open('wb') as log, (root/'airborne.log').open('wb') as result:
    p=subprocess.Popen([str(root/'controller'),'--ros-args','--params-file',str(root/'config.yaml'),'-p','command_gimbal_down:=false','-p','gimbal_down_on_startup:=false','-p','diagnostic_directory:='+str(root/'data')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:r=subprocess.run(['python3',str(root/'test_land_ros2.py')],env=env,stdout=result,stderr=subprocess.STDOUT,timeout=85)
    finally:
        os.killpg(p.pid,signal.SIGINT);p.wait(timeout=10)
env.update(ROS_DOMAIN_ID='90',LAND_EXECUTABLE=str(root/'controller'),LAND_CONFIG=str(root/'config.yaml'),LAND_MODES_OUTPUT=str(root/'modes'))
with (root/'modes.log').open('wb') as output:
    other=subprocess.run(['python3',str(root/'test_land_modes_ros2.py')],env=env,stdout=output,stderr=subprocess.STDOUT,timeout=95)
(root/'runner.json').write_text(json.dumps(dict(airborne=r.returncode,controller=p.returncode,modes=other.returncode)))
raise SystemExit(r.returncode or p.returncode or other.returncode)
