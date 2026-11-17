"""缓存等待期间有新回调时，快照不能产生负接收年龄。"""
from collections import deque
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

from telemetry_reader import TelemetryReader


class SnapshotTime(unittest.TestCase):
    def test_message_arriving_while_snapshot_waits_for_cache(self):
        waiting=threading.Event();release=threading.Event()

        class Gate:
            def __enter__(self):
                waiting.set()
                if not release.wait(2):raise RuntimeError('快照等待超时')
            def __exit__(self,*args):pass

        reader=object.__new__(TelemetryReader)
        reader.lock=Gate();reader.args=S(grid_only=False,stream_id='test')
        reader.boot='test';reader.planning={};reader.texts=deque();reader.errors={}
        reader.graph={'received':None,'pubs':{},'endpoints':{}}
        reader.node=S(get_clock=lambda:S(now=lambda:S(nanoseconds=time.time_ns())))
        reader.definitions={'lidar':('/livox/lidar',None,.4)}
        reader.cache={};reader.times={'lidar':deque()}
        result=[];failures=[]

        def snapshot():
            try:result.append(reader.snapshot(.2))
            except BaseException as error:failures.append(error)

        with patch('telemetry_reader.recording_status',return_value={'available':False}):
            worker=threading.Thread(target=snapshot)
            worker.start()
            try:
                self.assertTrue(waiting.wait(2))
                # 在快照取得缓存锁前到达的真实新样本。
                receipt=time.monotonic()
                reader.cache['lidar']={'receipt':receipt,'stamp':time.time(),'frame':'livox_frame','values':{'point_count':1}}
                reader.times['lidar'].append(receipt)
            finally:
                release.set();worker.join(2)
        self.assertFalse(worker.is_alive());self.assertFalse(failures)
        self.assertGreaterEqual(result[0]['topics']['lidar']['receipt_age_s'],0)
        self.assertGreaterEqual(result[0]['server_monotonic'],receipt)


if __name__=='__main__':unittest.main(verbosity=2)
