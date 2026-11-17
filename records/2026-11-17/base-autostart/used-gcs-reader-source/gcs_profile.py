"""本地配置及自包含SSH载荷；本模块不依赖ROS，Windows/WSL均可导入。"""
import base64
import json
import os
from pathlib import Path
import re
import shlex

HERE = Path(__file__).resolve().parent


def load_profile(path=None):
    profile = json.loads(Path(path or os.environ.get('GCS_ROS2_PROFILE') or HERE/'profile.json').read_text(encoding='utf-8'))
    if profile.get('schema') != 1 or profile.get('backend') != 'ros2':
        raise ValueError('ROS2配置版本或后端无效')
    if not isinstance(profile.get('domain_id'), int) or isinstance(profile['domain_id'], bool) or not 0 <= profile['domain_id'] <= 232:
        raise ValueError('ROS_DOMAIN_ID需为0至232')
    for key in ('user', 'host', 'rmw'):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', profile.get(key, '')) or profile[key].startswith('-'):
            raise ValueError('ROS2配置字段无效: '+key)
    if not isinstance(profile.get('overlays', []), list):
        raise ValueError('overlays必须是数组')
    paths = [profile.get('remote_env', ''), profile.get('app_root', ''), *profile.get('overlays', [])]
    if 'records_root' in profile: paths.append(profile['records_root'])
    for value in paths:
        if not isinstance(value,str) or not value.startswith('/') or '\x00' in value or '\n' in value:
            raise ValueError('ROS2环境文件必须是绝对路径')
    return profile


def command(program, profile=None):
    p = profile or load_profile()
    config = base64.b64encode(json.dumps(p, ensure_ascii=False).encode()).decode()
    parts = ['unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME', 'export PYTHONNOUSERSITE=1',
             'source '+shlex.quote(p['remote_env'])+' || exit 78']
    parts += ['source '+shlex.quote(path)+' || exit 78' for path in p.get('overlays', [])]
    parts += ['export ROS_DOMAIN_ID='+str(p['domain_id'])+' RMW_IMPLEMENTATION='+shlex.quote(p['rmw'])+' GCS_ROS2_CONFIG='+shlex.quote(config), program]
    return 'bash -c '+shlex.quote('; '.join(parts))


def payload(source):
    """通过stdin执行时仍能import共用运行时，不依赖机载安装本地兄弟文件。"""
    if isinstance(source, bytes): source=source.decode('utf-8')
    code='import sys,types,base64\n'
    for name,file in [('gcs_ros2_runtime','runtime.py'),('gcs_ros2_recording','recording.py')]:
        encoded=base64.b64encode((HERE/file).read_bytes()).decode()
        code+=('_gcs_module=types.ModuleType('+repr(name)+');sys.modules['+repr(name)+']=_gcs_module\n'
               '_gcs_module.__gcs_source__=base64.b64decode('+repr(encoded)+')\n'
               'exec(_gcs_module.__gcs_source__,_gcs_module.__dict__)\n')
    return (code+source).encode('utf-8')
