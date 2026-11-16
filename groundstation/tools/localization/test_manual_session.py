#!/usr/bin/env python3
"""手动入口的重复启动和会话归属守卫；完全离线。"""
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from test_stack import Fake
from manual_session import lifecycle_locks, start_session, stop_owned


class ManualGuards(unittest.TestCase):
    def test_only_lio_on_first_terminal(self):
        f=Fake();start_session(f, 'localization')
        self.assertEqual(f.calls, ['gate','inventory','idle','start_lio'])

    def test_only_relay_on_second_terminal(self):
        f=Fake(True);start_session(f, 'relay')
        self.assertEqual(f.calls, ['gate','inventory','idle','source','start_relay'])

    def test_repeated_lio_restarts_in_order_and_restores_relay(self):
        f=Fake(True,True);start_session(f,'localization')
        self.assertEqual(f.calls,['gate','inventory','idle','stop_relay','stop_lio','start_lio','source','start_relay'])

    def test_repeated_lio_without_relay_keeps_relay_off(self):
        f=Fake(True,False);start_session(f,'localization')
        self.assertEqual(f.calls,['gate','inventory','idle','stop_lio','start_lio'])

    def test_repeated_relay_only_restarts_relay(self):
        f=Fake(True,True);start_session(f,'relay')
        self.assertEqual(f.calls,['gate','inventory','idle','source','stop_relay','start_relay'])

    def test_failed_relay_stop_does_not_reset_origin(self):
        f=Fake(True,True,'stop_relay')
        with self.assertRaises(RuntimeError):start_session(f,'localization')
        self.assertNotIn('stop_lio',f.calls);self.assertNotIn('start_lio',f.calls)

    def test_no_source_preserves_existing_relay(self):
        f=Fake(True,True,'source')
        with self.assertRaises(RuntimeError):start_session(f,'relay')
        self.assertNotIn('stop_relay',f.calls);self.assertNotIn('start_relay',f.calls)

    def test_missing_lio_never_starts_relay(self):
        f=Fake(False,False)
        with self.assertRaises(RuntimeError):start_session(f,'relay')
        self.assertNotIn('start_relay',f.calls)

    def test_cancel_during_restart_does_not_start_new_process(self):
        f=Fake(True,True)
        from manual_session import StopRequested
        with self.assertRaises(StopRequested):
            start_session(f,'localization',cancelled=lambda:'stop_relay' in f.calls)
        self.assertNotIn('stop_lio',f.calls);self.assertNotIn('start_lio',f.calls)

    def test_record_new_lio_before_waiting_for_restored_relay(self):
        f=Fake(True,True)
        start_session(f,'localization',started=lambda:f.calls.append('remember'))
        self.assertLess(f.calls.index('remember'),f.calls.index('source'))

    def test_start_guards_never_mutate_on_rejection(self):
        for kind in ('localization','relay'):
            for error in ('gate','inventory','idle'):
                f=Fake(kind=='relay',False,error)
                with self.assertRaises(RuntimeError):start_session(f,kind)
                self.assertFalse(set(f.calls)&{'start_lio','start_relay'})

    def test_lio_exit_stops_relay_first(self):
        f=Fake(True,True)
        with patch('manual_session.session_token',return_value=(1,'a',Path('/run'))):
            stop_owned(f,'localization',(1,'a',Path('/run')))
        self.assertEqual(f.calls,['gate','inventory','idle','stop_relay','stop_lio'])

    def test_relay_exit_preserves_lio(self):
        f=Fake(True,True)
        with patch('manual_session.session_token',return_value=(2,'a',Path('/run'))):
            stop_owned(f,'relay',(2,'a',Path('/run')))
        self.assertNotIn('stop_lio',f.calls)
        self.assertIn('stop_relay',f.calls)

    def test_replaced_pid_or_start_time_preserves_new_session(self):
        for current in [(2,'a',Path('/run')),(1,'b',Path('/run'))]:
            f=Fake(True,True)
            with patch('manual_session.session_token',return_value=current):
                with self.assertRaises(RuntimeError):stop_owned(f,'localization',(1,'a',Path('/run')))
            self.assertFalse(set(f.calls)&{'stop_lio','stop_relay'})

    def test_stop_guards_and_failed_relay_stop_preserve_lio(self):
        for error in ('gate','inventory','idle','stop_relay'):
            f=Fake(True,True,error)
            with patch('manual_session.session_token',return_value=(1,'a',Path('/run'))):
                with self.assertRaises(RuntimeError):stop_owned(f,'localization',(1,'a',Path('/run')))
            self.assertNotIn('stop_lio',f.calls)

    def test_manual_and_gcs_lifecycle_lock_contention(self):
        with TemporaryDirectory() as d:
            root=Path(d)/'localization';root.mkdir();ws=Path(d)/'ego_planner';ws.mkdir()
            with lifecycle_locks(root,ws):
                with self.assertRaises(RuntimeError):
                    with lifecycle_locks(root,ws):pass
            with lifecycle_locks(root,ws):pass


if __name__=='__main__':unittest.main(verbosity=2)
