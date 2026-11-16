import base64,json,subprocess,threading,time,unittest,zlib
from stream_reader import decode_grid,packets_from_pipe,StatusAssembler
from telemetry_reader import status_datagrams,grid_display_payload,grid_transmissions,source_frequency
from collections import deque
import numpy as np,struct
from types import SimpleNamespace
from bridge import format_packet,merge_grid

class ReadOnlyTransport(unittest.TestCase):
 def test_low_rate_battery_window_and_unknown_frequency(self):
  samples=deque([0.,2.]);self.assertEqual(source_frequency(samples,4.9,12.),.5)
  self.assertEqual(list(samples),[0.,2.])
  self.assertIsNone(source_frequency(deque([2.]),2.1,12.))
  self.assertIsNone(source_frequency(deque([2.,2.]),2.1,12.))
  self.assertIsNone(source_frequency(samples,14.1,12.))
 def test_udp_reorder_missing_and_corruption(self):
  payload={'topics':{'lio':{'hz':10}},'noise':list(range(10000))};frames=list(status_datagrams(payload,123));self.assertGreater(len(frames),1)
  assembler=StatusAssembler();result=None
  for frame in reversed(frames):result=assembler.accept(frame) or result
  self.assertEqual(result,payload)
  incomplete=StatusAssembler()
  for frame in frames[:-1]:self.assertIsNone(incomplete.accept(frame))
  wrong=StatusAssembler()
  for frame in frames:self.assertIsNone(wrong.accept(frame[:16]+b'broken'))
 def test_source_sample_age_and_transport_age_stay_separate(self):
  packet={'topics':{'lidar':{'topic':'/livox/lidar','source_age_s':.1,'receipt_age_s':.05,'limit_s':.4,'hz':10.,'publishers':['/driver']}},'_transport_delay_s':.8}
  value=format_packet(packet,'192.168.1.235');lidar=next(r for r in value['rows'] if r['key']=='lidar')
  self.assertEqual(lidar['state'],'正常');self.assertEqual(lidar['age'],'0.10');self.assertEqual(value['transport_delay_s'],.8)
  packet['topics']['lidar']['source_age_s']=.9;self.assertEqual(format_packet(packet,'host')['rows'][0]['state'],'缺失/过期')
 def test_grid_fields_empty_padding_endianness_and_bounds(self):
  def frame(points,endian='<',width=None):
   width=width or len(points);rows=[]
   for row in range(len(points)//width):
    rows.append(b''.join(struct.pack(endian+'ffff',*v,1.) for v in points[row*width:(row+1)*width])+b'pad!')
   return SimpleNamespace(width=width,height=len(rows),point_step=16,row_step=16*width+4,data=b''.join(rows),is_bigendian=endian=='>',fields=[SimpleNamespace(name=n,datatype=7,count=1,offset=o) for n,o in [('x',0),('y',4),('z',8)]])
  positions=[(-2.95,1.05,.55),(.05,-.15,1.25),(1.05,2.05,3.05),(0.15,.25,.35)]
  for endian in ('<','>'):
   value=grid_display_payload(frame(positions,endian,2),.1);self.assertIn(value['codec'],('zlib-i16-half-grid','zlib-i16-delta-half-grid'))
   self.assertTrue(np.allclose(np.frombuffer(decode_grid(value),dtype='<f4').reshape(-1,3),positions,atol=1e-5))
  non_grid=grid_display_payload(frame([(.1234,.25,.35)]),.1);self.assertEqual(non_grid['codec'],'zlib-f32')
  empty=SimpleNamespace(width=0,height=1,data=b'',fields=[],point_step=16,row_step=0,is_bigendian=False)
  self.assertEqual(grid_display_payload(empty,.1)['count'],0);self.assertEqual(decode_grid(grid_display_payload(empty,.1)),b'')
  invalid=frame(positions);invalid.data=invalid.data[:-1]
  with self.assertRaises(ValueError):grid_display_payload(invalid,.1)
 def test_grid_udp_recovers_one_lost_fragment_without_accepting_corruption(self):
  import random
  rng=random.Random(41)
  points=[(rng.randrange(-100,100)*.1+.05,rng.randrange(-100,100)*.1+.05,rng.randrange(-10,10)*.1+.05) for _ in range(5000)]
  msg=SimpleNamespace(width=len(points),height=1,point_step=12,row_step=12*len(points),data=b''.join(struct.pack('<fff',*v) for v in points),is_bigendian=False,fields=[SimpleNamespace(name=n,datatype=7,count=1,offset=o) for n,o in [('x',0),('y',4),('z',8)]])
  grid=grid_display_payload(msg,.1);message={'type':'GRID1','stream_id':'test','ego_planner':{'grid':{'data':grid}}}
  chunks=list(status_datagrams(message,712,magic=b'TLG1'));self.assertGreater(len(chunks),1)
  a=StatusAssembler(b'TLG1');result=None
  for fragment in chunks[1:]+chunks:
   result=a.accept(fragment) or result
  self.assertEqual(result['type'],'GRID1')
  restored=np.frombuffer(decode_grid(result['ego_planner']['grid']['data']),dtype='<f4').reshape(-1,3)
  self.assertTrue(np.allclose(restored,points,atol=1e-5))
  self.assertIsNone(StatusAssembler().accept(chunks[0]))
 def test_grid_retransmission_keeps_original_age_and_session(self):
  def frame(session,sequence,sent):
   return {'type':'GRID1','server_monotonic':sent,'ego_planner':{'status':{'data':{'session_id':session}},'grid':{'receipt_age_s':.2,'data':{'sequence':sequence}}}}
  first=frame('A',1,100.);groups,previous=grid_transmissions(first,7,None)
  self.assertEqual(len(groups),2)
  groups,previous=grid_transmissions(frame('A',2,100.5),8,previous)
  self.assertEqual(len(groups),3)
  # 新帧两份都丢失后，补发仍是上一帧原始时间，不能刷新其采样龄期。
  assembler=StatusAssembler(b'TLG1');result=None
  for datagram in groups[2]:result=assembler.accept(datagram) or result
  self.assertEqual(result['server_monotonic'],100.);self.assertEqual(result['ego_planner']['grid']['receipt_age_s'],.2)
  groups,_=grid_transmissions(frame('B',3,101.),9,previous)
  self.assertEqual(len(groups),2)
 def test_grid_session_identity_and_age_are_preserved(self):
  planning={'status':{'data':{'session_id':'A'}}};cache={'session_id':'B','received':10.,'row':{'receipt_age_s':.2,'data':{'count':12}}}
  self.assertNotIn('grid',merge_grid(dict(planning),cache,11.))
  cache['session_id']='A';merged=merge_grid(dict(planning),cache,11.)
  self.assertAlmostEqual(merged['grid']['receipt_age_s'],1.2);self.assertEqual(merged['grid']['data']['count'],12)
  self.assertGreater(merge_grid(dict(planning),cache,12.)['grid']['receipt_age_s'],1.5)
 def test_compressed_grid_bounds(self):
  raw=b'\0'*12000;self.assertEqual(decode_grid({'codec':'zlib-f32','count':1000,'xyz':base64.b64encode(zlib.compress(raw)).decode()}),raw)
  with self.assertRaises(ValueError):decode_grid({'codec':'zlib-f32','count':18000,'xyz':base64.b64encode(zlib.compress(b'\0'*216001)).decode()})
  with self.assertRaises(ValueError):decode_grid({'count':0,'xyz':base64.b64encode(b'bad').decode()})
 def test_stalled_process_times_out(self):
  p=subprocess.Popen(['/usr/bin/python3','-u','-c','import time; print(\'{"server_monotonic":1}\',flush=True); time.sleep(10)'],stdout=subprocess.PIPE)
  try:
   stream=packets_from_pipe(p,threading.Event(),idle_timeout=.35,startup_timeout=1);next(stream)
   with self.assertRaises(TimeoutError):next(stream)
  finally:p.terminate();p.wait();p.stdout.close()
 def test_burst_keeps_latest_complete_packet(self):
  p=subprocess.Popen(['/usr/bin/python3','-u','-c','print(\'{"seq":1}\\n{"seq":2}\\n{"seq":3}\',flush=True)'],stdout=subprocess.PIPE)
  try:
   time.sleep(.1);rows=list(packets_from_pipe(p,threading.Event()));self.assertEqual(rows[-1]['seq'],3);self.assertEqual(len(rows),1)
  finally:p.wait();p.stdout.close()
if __name__=='__main__':unittest.main(verbosity=2)
