import importlib.util, tempfile, unittest, json, io, base64, sys
from pathlib import Path
from types import SimpleNamespace
spec=importlib.util.spec_from_file_location('remote_records',Path(__file__).parents[1]/'remote_records.py')
remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote)
class RemoteRecords(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.run=self.root/'flight';self.run.mkdir()
        self.bag=self.run/'inputs_0.bag';self.bag.write_bytes(b'#ROSBAG V2.0\n'+bytes(range(256))*10)
    def tearDown(self):self.tmp.cleanup()
    def token(self,p):return base64.urlsafe_b64encode(p.encode()).decode()
    def stream(self,path=None,size=None,mtime=None):
        st=self.bag.stat();out=io.BytesIO();old=sys.stdout;sys.stdout=SimpleNamespace(buffer=out)
        try:remote.stream(self.root,self.token(path or 'flight/inputs_0.bag'),st.st_size if size is None else size,st.st_mtime_ns if mtime is None else mtime)
        finally:sys.stdout=old
        return out.getvalue()
    def test_list_and_binary_stream(self):
        rows=remote.catalogue(self.root)['records'];self.assertEqual(len(rows),1);self.assertEqual(rows[0]['parts'],1)
        self.assertIsInstance(rows[0]['files'][0]['mtime_ns'],str)
        self.assertEqual(self.stream(),self.bag.read_bytes())
    def test_active_run_excluded_and_read_rejected(self):
        (self.run/'new.bag.active').touch();self.assertEqual(remote.catalogue(self.root)['records'],[])
        with self.assertRaises(ValueError):self.stream()
    def test_recording_state_excluded(self):
        (self.run/'recording-status.json').write_text(json.dumps({'phase':'RECORDING'}));self.assertEqual(remote.catalogue(self.root)['records'],[])
    def test_symlinks_and_traversal_rejected(self):
        (self.run/'linked.bag').symlink_to(self.bag);self.assertEqual(remote.catalogue(self.root)['records'][0]['parts'],1)
        for path in ['../outside.bag','/tmp/outside.bag','flight/linked.bag']:
            with self.assertRaises(ValueError):self.stream(path)
    def test_changed_file_rejected(self):
        with self.assertRaises(ValueError):self.stream(size=1)
        with self.assertRaises(ValueError):self.stream(mtime=1)
    def test_invalid_bag_rejected(self):
        self.bag.write_bytes(b'not a bag'*20)
        with self.assertRaises(ValueError):self.stream()
if __name__=='__main__':unittest.main()
