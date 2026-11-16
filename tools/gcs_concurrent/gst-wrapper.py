#!/usr/bin/python3
import os,sys
args=sys.argv[1:]
old='location=rtsp://192.168.1.224:8554/gimbal';new='location=rtsp://192.168.1.224:18554/gimbal'
assert args.count(old)==1
args[args.index(old)]=new
if os.environ.get('GCS_GST_DIAG'):
 os.environ['GST_DEBUG']='basesink:6'
 os.environ['GST_DEBUG_FILE']=os.environ['GCS_GST_DIAG']
os.execv('/usr/bin/gst-launch-1.0',['gst-launch-1.0',*args])
