from pathlib import Path
import os,signal,subprocess,json,hashlib,time
root=Path('/tmp/person-consumer-regression');install=Path('/home/orangepi/drone_platform/01_apps/mission_demo/workspace/install')
exe=root/'Safina_track_test'
config=install/'track_test/share/track_test/config/track.yaml'
results=[]
for label,script in [('controller','test_track_ros2.py'),('npu-chain','test_person_chain_ros2.py')]:
 children=[];logs=[]
 def start(name,args):
  log=(root/(label+'-'+name+'.log')).open('wb');logs.append(log)
  p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
 try:
  controller=start('controller',[str(exe),'--ros-args','--params-file',str(config)])
  if label=='npu-chain':
   start('detector',[str(install/'yolo_ultra_test/lib/yolo_ultra_test/Safina_yoloUltra_test'),'--ros-args','--params-file',str(install/'yolo_ultra_test/share/yolo_ultra_test/config/yolo_track.yaml')])
  fixture=start('fixture',['python3',str(root/script)]+([str(root/'bus.jpg')] if label=='npu-chain' else []))
  rc=fixture.wait(timeout=100)
  results.append(dict(label=label,returncode=rc,controller_alive=controller.poll() is None,binary_sha256=hashlib.sha256(exe.read_bytes()).hexdigest()))
 finally:
  for p in reversed(children):
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for log in logs:log.close()
 time.sleep(1)
(root/'result.json').write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(0 if all(r['returncode']==0 and r['controller_alive'] for r in results) else 1)
