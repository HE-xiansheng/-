"""隔离验证安装启动地址、视频代理范围；不登录机载或拉取现场视频。"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SRC = Path(__file__).resolve().parents[4]
START = SRC / 'tools/packaging/start_bundle.sh'
WRAPPER = SRC / 'tools/desktop/video-bin/gst-launch-1.0'


class VideoRelease(unittest.TestCase):
    def test_saved_host_and_explicit_override_with_chinese_path(self):
        with tempfile.TemporaryDirectory(prefix='视频 release ') as temp:
            root = Path(temp)
            (root / 'dianyunGCS-master').mkdir()
            (root / 'build/linux').mkdir(parents=True)
            (root / '.pointcloud-link').mkdir()
            binary = root / 'build/linux/customgeometry'
            binary.write_text('#!/usr/bin/python3\nimport json,os,sys\nprint(json.dumps({"args":sys.argv[1:],"path":os.environ["PATH"]}))\n')
            binary.chmod(0o755)
            start = root / 'start-groundstation.sh'
            shutil.copy2(START, start)
            config = root / '.pointcloud-link/connection.json'
            config.write_text(json.dumps({'host': '192.168.1.13'}))
            def run(*args):
                return json.loads(subprocess.check_output([str(start), *args], text=True))
            result = run('--live', '9000', '--pose-port', '9001')
            self.assertEqual(result['args'], ['--cloud-port', '9000', '--pose-port', '9001', '--host', '192.168.1.13'])
            self.assertTrue(result['path'].startswith(str(root / 'dianyunGCS-master/tools/desktop/video-bin') + ':'))
            for args in (('--host', '192.168.1.122'), ('--host=192.168.1.122',)):
                self.assertEqual(run('--live', '9000', *args)['args'], ['--cloud-port', '9000', *args])
            self.assertEqual(run('--no-splash')['args'], ['--no-splash'])
            for raw in ('broken json', '{"host":"--no-splash"}', '{"host":"a b"}', '{"host":null}'):
                config.write_text(raw)
                self.assertEqual(run('--live', '9000')['args'], ['--cloud-port', '9000'])

    def test_only_current_gimbal_endpoint_bypasses_inherited_proxy(self):
        # 截取包装器最后的exec，观察真正传给系统解码器的环境与原始参数。
        code = '''function exec() {
 /usr/bin/python3 -c 'import os,json,sys; print(json.dumps({"args":sys.argv[1:],"lower":os.environ.get("no_proxy"),"upper":os.environ.get("NO_PROXY"),"proxy":os.environ.get("http_proxy")}))' "$@"
}
source "$1" "${@:2}"
'''
        env = dict(os.environ, no_proxy='localhost', NO_PROXY='127.0.0.1', http_proxy='http://127.0.0.1:7890')
        for host in ('192.168.1.13', '192.168.1.235', 'onboard.example'):
            args = ['-q', 'rtspsrc', 'location=rtsp://' + host + ':8554/gimbal', 'protocols=tcp', '!', 'fakesink']
            result = json.loads(subprocess.check_output(['bash', '-c', code, 'fixture', str(WRAPPER), *args], env=env, text=True))
            self.assertEqual(result, {'args': ['/usr/bin/gst-launch-1.0', *args], 'lower': 'localhost,' + host, 'upper': '127.0.0.1,' + host, 'proxy': env['http_proxy']})
        for location in ('rtsp://192.168.1.13:8554/other', 'rtsp://example.com:554/gimbal', 'https://example.com/video'):
            result = json.loads(subprocess.check_output(['bash', '-c', code, 'fixture', str(WRAPPER), 'location=' + location], env=env, text=True))
            self.assertEqual(result['lower'], 'localhost')
            self.assertEqual(result['upper'], '127.0.0.1')
            self.assertEqual(result['proxy'], env['http_proxy'])

    def test_wrapper_runs_system_decoder(self):
        subprocess.run([str(WRAPPER), '-q', 'videotestsrc', 'num-buffers=2', '!', 'fakesink'], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
