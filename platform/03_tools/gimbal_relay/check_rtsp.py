"""Read-only RTSP/TCP H264 packet check; no external Python dependencies."""
import socket, re, time, struct, json, sys
from urllib.parse import urlsplit
url=sys.argv[1] if len(sys.argv)>1 else 'rtsp://192.168.1.224:8554/gimbal'
u=urlsplit(url);s=socket.create_connection((u.hostname,u.port or 554),5);s.settimeout(8);f=s.makefile('rb');seq=0

def request(method,target,headers=''):
 global seq
 seq+=1;s.sendall((f'{method} {target} RTSP/1.0\r\nCSeq: {seq}\r\n'+headers+'\r\n').encode())
 status=f.readline().decode().strip();h={}
 while True:
  l=f.readline()
  if l==b'\r\n':break
  if not l:raise RuntimeError('connection closed')
  k,v=l.decode().split(':',1);h[k.lower()]=v.strip()
 body=f.read(int(h.get('content-length','0')))
 if ' 200 ' not in status:raise RuntimeError(status+' '+body.decode(errors='replace'))
 return h,body
h,b=request('DESCRIBE',url,'Accept: application/sdp\r\n');sdp=b.decode();assert 'H264/90000' in sdp,sdp
control=[x for x in re.findall(r'a=control:([^\r\n]+)',sdp) if x!='*'][0];base=h.get('content-base',url+'/');track=control if control.startswith('rtsp://') else base.rstrip('/')+'/'+control
h,_=request('SETUP',track,'Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n');session=h['session'].split(';')[0]
request('PLAY',url,'Session: '+session+'\r\n')
start=time.monotonic();packets=frames=total=0;first=None
while time.monotonic()-start<5:
 hdr=f.read(4)
 if len(hdr)!=4 or hdr[0]!=36:raise RuntimeError('invalid interleaved packet')
 payload=f.read(int.from_bytes(hdr[2:4],'big'))
 if hdr[1]!=0:continue
 if first is None:first=time.monotonic()-start
 packets+=1;total+=len(payload)
 if payload[1]&128:frames+=1
elapsed=time.monotonic()-start
print(json.dumps(dict(url=url,codec='H264',seconds=round(elapsed,3),rtp_packets=packets,frame_markers=frames,fps=round(frames/elapsed,2),mbps=round(total*8/elapsed/1e6,3),first_packet_ms=round(first*1000,2)),indent=2))
f.close();s.close()
