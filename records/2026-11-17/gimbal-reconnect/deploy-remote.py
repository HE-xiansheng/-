from pathlib import Path
import subprocess,hashlib,tarfile,io,sys,json,os
root=Path('/home/orangepi/drone_platform')
assert subprocess.check_output(['git','-C',str(root),'rev-parse','--short','HEAD']).decode().strip()=='1f81c3b'
assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain']).strip()
assert subprocess.run(['pgrep','-f','^.*lib/rtsp_test/rtsp_hw_pub_ultra( |$)'],capture_output=True).returncode==1
old={'01_apps/mission_demo/workspace/src/rtsp_test/CMakeLists.txt': '69b64b5aab6663695454beaecff7c3730c10599cfdfca2c78210d0550ae35249', '01_apps/mission_demo/workspace/src/rtsp_test/package.xml': '0b78772be191bb81494e65f32ada20d88c26b969569441b9937044e96a644e25', '01_apps/mission_demo/workspace/src/rtsp_test/src/rtsp_hw_pub_ultra.cpp': '88c492138012127ed8ea5c485ddbf08c680fa69a2abe304906bbd40928730bcf', '01_apps/mission_demo/workspace/src/rtsp_test/config/fastdds_large_images.xml': '6038209349e7f966f197c378b08b7c8a875c462f21ed9178042bc238b5998071', '01_apps/mission_demo/workspace/install/rtsp_test/lib/rtsp_test/rtsp_hw_pub_ultra': 'f391437954c37c5b2b253c637db141421fc5479b808d5d7d2915a33810d85cc1', '01_apps/mission_demo/workspace/install/rtsp_test/share/rtsp_test/package.xml': '0b78772be191bb81494e65f32ada20d88c26b969569441b9937044e96a644e25', '01_apps/mission_demo/workspace/install/rtsp_test/share/rtsp_test/config/fastdds_large_images.xml': '6038209349e7f966f197c378b08b7c8a875c462f21ed9178042bc238b5998071'}
new={'01_apps/mission_demo/workspace/src/rtsp_test/src/rtsp_hw_pub_ultra.cpp': 'e63eba8d537b4c9a533d6ad06bf61adce07492279cfd92f12ad5e0e3caeda8be', '01_apps/mission_demo/workspace/install/rtsp_test/lib/rtsp_test/rtsp_hw_pub_ultra': '9b208175f65baa3307c20220d58e42e883a4762a56d3dca7dd7070b9bfce0686'}
for n in new:assert hashlib.sha256((root/n).read_bytes()).hexdigest()==old[n],n
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r') as a:
 for n in new:
  data=a.extractfile(n).read();assert hashlib.sha256(data).hexdigest()==new[n]
  target=root/n;tmp=target.with_name(target.name+'.new');tmp.write_bytes(data);tmp.chmod(target.stat().st_mode);os.replace(tmp,target)
subprocess.run(['git','-C',str(root),'add','01_apps/mission_demo/workspace/src/rtsp_test/src/rtsp_hw_pub_ultra.cpp'],check=True)
subprocess.run(['git','-C',str(root),'-c','user.name=tyzk','-c','user.email=940833041@qq.com','commit','-m','Recover MPP RTSP streams after disconnect and bound fractional preview scaling'],check=True)
print(json.dumps(dict(head=subprocess.check_output(['git','-C',str(root),'rev-parse','--short','HEAD']).decode().strip(),sha256=new,status=subprocess.check_output(['git','-C',str(root),'status','--porcelain']).decode()),indent=2))
