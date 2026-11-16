#!/usr/bin/env python3
"""验证实际归档解压、依赖重定位和完整主程序首帧；不连接或启动机载任务。"""
import hashlib,json,os,signal,subprocess,sys,tarfile,time
from pathlib import Path
archive=Path(sys.argv[1]).resolve();work=Path(sys.argv[2]).resolve();out=Path(sys.argv[3]).resolve()
out.mkdir(parents=True,exist_ok=True);work.mkdir(parents=True,exist_ok=False)
checks={};result={};p=None;completed=False
try:
    with tarfile.open(archive) as tar:tar.extractall(work)
    roots=list(work.iterdir());assert len(roots)==1;bundle=roots[0]
    manifest=json.loads((bundle/'payload-manifest.json').read_text())
    for rel,record in manifest['files'].items():
        f=bundle/rel
        if 'symlink' in record:assert f.is_symlink() and os.readlink(f)==record['symlink'],rel
        else:assert hashlib.sha256(f.read_bytes()).hexdigest()==record['sha256'] and oct(f.stat().st_mode&0o777)==record['mode'],rel
    checks['archive_manifest_and_modes']=True
    checks['only_current_runtime_tools']=len([f for f in (bundle/'tools').rglob('*') if f.is_file()])==12
    checks['no_historical_ros1_installers']=not any('automatic-navigation' in str(f) or 'task_manager.py'==f.name for f in bundle.rglob('*'))
    env=dict(os.environ);env['LD_LIBRARY_PATH']=str(bundle/'lib');env['LD_LIBRARY_PATH']=str(bundle/'lib')
    deps=[]
    for f in sorted(bundle.rglob('*')):
        if not f.is_file() or f.is_symlink():continue
        with f.open('rb') as stream:elf=stream.read(4)==b'\x7fELF'
        if not elf:continue
        q=subprocess.run(['ldd',str(f)],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        deps.append(str(f)+'\n'+q.stdout)
        if q.returncode or 'not found' in q.stdout:raise AssertionError('Missing runtime dependency: '+str(f)+'\n'+q.stdout)
        for line in q.stdout.splitlines():
            if 'libQt6' in line or 'libicu' in line:
                target=line.split('=>',1)[-1].strip().rsplit(' (',1)[0]
                assert str(Path(target).resolve()).startswith(str(bundle)+'/'),line
    (out/'ldd-all-elf.txt').write_text('\n'.join(deps));checks['all_elf_dependencies_and_qt_relocation']=True
    env.update(XDG_CONFIG_HOME=str(work/'private-config'),XDG_DATA_HOME=str(work/'private-data'),XDG_CACHE_HOME=str(work/'private-cache'),GCS_STATE_DIR=str(work/'private-state'),QT_DEBUG_PLUGINS='1')
    log=out/'application.log'
    with log.open('w') as f:
        p=subprocess.Popen([str(bundle/'start-groundstation.sh'),'--no-splash','--cloud-port','20100'],cwd='/tmp',env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            content=log.read_text(errors='replace')
            if p.poll() is not None:raise AssertionError('Main application exited '+str(p.returncode))
            if '地面站首帧就绪' in content:break
            time.sleep(.25)
        else:raise AssertionError('Main application did not render a first frame')
        checks['full_application_main_first_frame']=True
        maps=Path('/proc/'+str(p.pid)+'/maps').read_text();(out/'application-maps.txt').write_text(maps)
        loaded=[line.split(None,5)[5] for line in maps.splitlines() if 'libQt6' in line]
        checks['loaded_qt_libraries_from_relocated_bundle']=bool(loaded) and all(v.startswith(str(bundle)+'/') for v in loaded)
        checks['no_qml_load_errors']='failed to load component' not in content.lower() and 'is not installed' not in content.lower()
    completed=True
    result.update(bundle=str(bundle),archive=str(archive),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
finally:
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
    result.update(checks=checks,application_exit=None if p is None else p.returncode,passed=completed and bool(checks) and all(checks.values()))
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False));assert result['passed']
