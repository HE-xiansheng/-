#!/usr/bin/env python3
"""离线核实编排守卫；不连接生产 ROS，不发飞控命令。"""
import unittest
import sys
from unittest.mock import patch
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from manage_stack import operate,ExistingStack

class Fake:
    def __init__(self,lio=False,relay=False,error=None):self.lio,self.relay,self.error,self.calls=lio,relay,error,[]
    def call(self,name):
        self.calls.append(name)
        if self.error==name:raise RuntimeError(name+' rejected')
    def gate(self):self.call('gate')
    def inventory(self):self.call('inventory');return self.lio,self.relay
    def require_idle(self):self.call('idle')
    def start_lio(self):self.call('start_lio')
    def wait_source(self):self.call('source')
    def start_relay(self):self.call('start_relay')
    def stop_relay(self):self.call('stop_relay')
    def stop_lio(self):self.call('stop_lio')

class Guards(unittest.TestCase):
    def test_reuse_keeps_origin(self):
        f=Fake(True,True);self.assertTrue(operate('start',f)['ok']);self.assertEqual(f.calls,['gate','inventory','source'])
    def test_start_order(self):
        f=Fake();operate('start',f);self.assertEqual(f.calls,['gate','inventory','idle','start_lio','source','start_relay'])
    def test_existing_lio_only_starts_relay(self):
        f=Fake(True);operate('start',f);self.assertNotIn('start_lio',f.calls);self.assertIn('start_relay',f.calls)
    def test_stop_order(self):
        f=Fake(True,True);operate('stop',f);self.assertEqual(f.calls,['gate','inventory','idle','stop_relay','stop_lio'])
    def test_gate_identity_and_busy_rejections_have_no_mutation(self):
        for action in ('start','stop'):
            for error in ('gate','inventory','idle'):
                f=Fake(action=='stop',action=='stop',error)
                with self.assertRaises(RuntimeError):operate(action,f)
                self.assertFalse(set(f.calls)&{'start_lio','start_relay','stop_lio','stop_relay'})
    def test_missing_source_never_starts_relay(self):
        f=Fake(error='source')
        with self.assertRaises(RuntimeError):operate('start',f)
        self.assertNotIn('start_relay',f.calls)
    def test_relay_stop_failure_keeps_lio(self):
        f=Fake(True,True,'stop_relay')
        with self.assertRaises(RuntimeError):operate('stop',f)
        self.assertNotIn('stop_lio',f.calls)
    def test_orphan_relay_blocks_new_origin(self):
        f=Fake(False,True)
        with self.assertRaises(RuntimeError):operate('start',f)
        self.assertNotIn('start_lio',f.calls)
    def test_real_fresh_flight_gate(self):
        api=ExistingStack.__new__(ExistingStack)
        for age,mode,accepted in [(0.1,'POSCTL',True),(8.,'POSCTL',False),(0.,'OFFBOARD',False),(float('nan'),'POSCTL',False)]:
            delta=SimpleNamespace(to_sec=lambda:age)
            class Stamp:
                def __sub__(self,other):return delta
            state=SimpleNamespace(header=SimpleNamespace(stamp=0),mode=mode)
            api.board=SimpleNamespace(gate=lambda:(state,{}),ros=lambda:(SimpleNamespace(Time=SimpleNamespace(now=Stamp)),None))
            if accepted:api.gate()
            else:
                with self.assertRaises(RuntimeError):api.gate()
    def test_real_source_resampled_after_identity_check(self):
        api=ExistingStack.__new__(ExistingStack);clock=[1.];calls=[]
        class Stamp:
            def __init__(self,value):self.value=value
            def __sub__(self,other):return SimpleNamespace(to_sec=lambda:self.value-other.value)
        def sample(*args,**kwargs):
            calls.append('sample')
            return SimpleNamespace(header=SimpleNamespace(frame_id='camera_init',stamp=Stamp(clock[0])),child_frame_id='body',pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0,y=0,z=0),orientation=SimpleNamespace(x=0,y=0,z=0,w=1))))
        def verify():calls.append('verify');clock[0]+=1.
        ros=SimpleNamespace(wait_for_message=sample,Time=SimpleNamespace(now=lambda:Stamp(clock[0])),ROSException=TimeoutError)
        api.board=SimpleNamespace(ros=lambda:(ros,None),alive=lambda kind:True,ODOM='/factory_fastlio/odometry',verify_source=verify)
        with patch.dict(sys.modules,{'nav_msgs':SimpleNamespace(),'nav_msgs.msg':SimpleNamespace(Odometry=object)}):api.wait_source()
        self.assertEqual(calls,['sample','verify','sample'])

    def test_real_idle_gate_execution_and_preview(self):
        api=ExistingStack.__new__(ExistingStack)
        with TemporaryDirectory() as d:
            api.ws=Path(d);api.execution=SimpleNamespace(current=lambda:{'pid':123})
            with self.assertRaises(RuntimeError):api.require_idle()
            api.execution=SimpleNamespace(current=lambda:None)
            api.board=SimpleNamespace(graph=lambda:{'/ego_planner/manual_preview/status':['/preview']})
            with self.assertRaises(RuntimeError):api.require_idle()
            api.board=SimpleNamespace(graph=lambda:{})
            api.require_idle()

if __name__=='__main__':unittest.main(verbosity=2)
