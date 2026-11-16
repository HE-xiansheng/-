from pathlib import Path
import os,subprocess,shutil,sys,json
repo=Path('/home/tyzk/drone-migration-github');out=Path(sys.argv[1]);out.mkdir(exist_ok=False);(out/'shims').mkdir();shutil.copy2(repo/'tools/gcs_concurrent/gst-wrapper.py',out/'shims/gst-launch-1.0');(out/'shims/gst-launch-1.0').chmod(0o700)
qt='/home/tyzk/dianyunGCS-review-20261026/.deps/Qt';libs='/home/tyzk/drone-migration-work/portable relocation test3/drone-gcs-ros2-20261116-22c9b52-r2-linux-amd64/lib';env=dict(os.environ,PATH=str(out/'shims')+':'+os.environ['PATH'],LD_LIBRARY_PATH=libs+':'+qt+'/lib',QML_IMPORT_PATH=qt+'/qml',DISPLAY=':1',LIBGL_ALWAYS_SOFTWARE='1',GCS_STATE_DIR=str(out/'state'),XDG_CONFIG_HOME=str(out/'config'),XDG_DATA_HOME=str(out/'data'),XDG_CACHE_HOME=str(out/'cache'),GCS_VIDEO_PIPE_PROBE='1',GCS_CONCURRENT_RESULT=str(out/'result.json'),GCS_GST_DIAG=str(out/'sink.log'))
if len(sys.argv)>2:
 for option in sys.argv[2:]:
  k,v=option.split('=',1);env[k]=v
(out/'settings.json').write_text(json.dumps({k:v for k,v in env.items() if k.startswith(('QSG_','QT_QUICK','LIBGL','GCS_VIDEO'))},indent=2))
with (out/'qt.log').open('w') as f:r=subprocess.run(['/home/tyzk/drone-migration-work/build-gcs-concurrent/gcs/customgeometry','--no-splash','--host','192.168.1.224'],env=env,cwd=out,stdout=f,stderr=subprocess.STDOUT,timeout=60)
print('exit',r.returncode)
