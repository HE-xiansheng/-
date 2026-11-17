#!/usr/bin/env python3
"""仅安装并启用本次明确批准的两个基础服务；不包含任务或飞行命令。"""
import hashlib,json,os,subprocess,tempfile
from pathlib import Path

stage=Path('/tmp/drone-base-autostart')
product=Path('/home/orangepi/drone_platform')
units=['gimbal-rtsp-relay.service','Safina_sensor.service']
manifest=json.loads((stage/'manifest.json').read_text())
def run(*args):
    subprocess.run(args,check=True)
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
assert os.geteuid()==0 and os.uname().nodename=='orangepi5ultra'
for name in units:
    source=product/'03_tools/systemd'/name
    assert digest(source)==manifest['units'][name]
    target=Path('/etc/systemd/system')/name
    assert not target.exists() and not target.is_symlink(), str(target)+' already exists'
assert digest(product/'03_tools/systemd/wait_for_sensor_devices.py')==manifest['wait_helper']
run('systemd-analyze','verify',*(str(product/'03_tools/systemd'/name) for name in units))
run('runuser','-u','orangepi','--','python3',str(stage/'probe_px4_disarmed.py'),
    '--dialect',str(stage/'common.py'))
created=[]
try:
    for name in units:
        target=Path('/etc/systemd/system')/name
        with tempfile.NamedTemporaryFile(dir=target.parent,prefix='.'+name+'.',delete=False) as stream:
            temporary=Path(stream.name)
            stream.write((product/'03_tools/systemd'/name).read_bytes())
            stream.flush();os.fchmod(stream.fileno(),0o644);os.fsync(stream.fileno())
        os.replace(temporary,target);created.append(name)
    run('systemctl','daemon-reload')
    run('systemctl','enable',*units)
    run('systemctl','start','gimbal-rtsp-relay.service')
    run('systemctl','start','--no-block','Safina_sensor.service')
    run('systemctl','is-enabled',*units)
    print(json.dumps({'installed':units,'enabled':True,'sensor_start_requested':True}),flush=True)
except BaseException:
    for name in reversed(created):
        subprocess.run(['systemctl','disable','--now',name],check=False)
        target=Path('/etc/systemd/system')/name
        if target.is_file() and digest(target)==manifest['units'][name]:target.unlink()
    subprocess.run(['systemctl','daemon-reload'],check=False)
    raise
