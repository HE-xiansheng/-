#!/usr/bin/env python3
"""只读列举及传输已结束的机载飞行录包；不依赖ROS、不修改录制状态。"""
import argparse, base64, json, os, stat, sys
from pathlib import Path

def inside(root, path):
    try: path.resolve().relative_to(root.resolve()); return True
    except ValueError: return False

def closed(run):
    if any(run.glob('*.active')): return False
    state=run/'recording-status.json'
    if state.is_file():
        try:
            if json.loads(state.read_text()).get('phase') in ('RECORDING','STARTING_BAG','PREPARING','STOPPING'):return False
        except (ValueError,OSError): return False
    return True

def catalogue(root):
    if not root.is_dir(): raise ValueError('机载电脑尚无 flight-records 目录')
    rows=[]
    for run in sorted(root.iterdir(),key=lambda p:p.name,reverse=True):
        if not run.is_dir() or run.is_symlink() or not inside(root,run) or not closed(run):continue
        files=[]
        for bag in sorted(list(run.glob('*.bag'))+list(run.glob('inputs-*/*.db3'))):
            if bag.suffix=='.db3' and (bag.parent.is_symlink() or not (bag.parent/'metadata.yaml').is_file()):continue

            if bag.is_symlink() or not bag.is_file() or not inside(root,bag):continue
            st=bag.stat()
            if st.st_size<=13:continue
            files.append({'path':bag.relative_to(root).as_posix(),'name':bag.name,'size':st.st_size,'mtime_ns':str(st.st_mtime_ns)})
        if files: rows.append({'id':run.relative_to(root).as_posix(),'name':run.name,'files':files,'size':sum(f['size'] for f in files),'parts':len(files)})
        if len(rows)>=500:break
    return {'records':rows}

def stream(root, token, size, mtime):
    rel=base64.urlsafe_b64decode(token.encode()).decode('utf-8')
    path=root/rel
    parts=Path(rel).parts
    run=root/parts[0] if parts else root
    if Path(rel).is_absolute() or '..' in parts or not inside(root,path) or any((root.joinpath(*parts[:i])).is_symlink() for i in range(1,len(parts)+1)) or path.suffix not in ('.bag','.db3') or not run.is_dir() or (path.suffix=='.bag' and len(parts)!=2) or not closed(run):raise ValueError('录包路径无效或仍在录制')
    if path.suffix=='.db3' and (len(parts)!=3 or not (path.parent/'metadata.yaml').is_file()):raise ValueError('ROS2录包尚未收尾')

    fd=os.open(str(path),os.O_RDONLY|getattr(os,'O_NOFOLLOW',0))
    with os.fdopen(fd,'rb') as f:
        before=os.fstat(f.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size!=size or before.st_mtime_ns!=mtime:raise ValueError('录包已变化，请刷新列表')
        magic=f.read(16)
        if (path.suffix=='.bag' and not magic.startswith(b'#ROSBAG V2.0\n')) or (path.suffix=='.db3' and magic!=b'SQLite format 3\0'):raise ValueError('录包文件头无效')
        f.seek(0)
        while True:
            block=f.read(1024*256)
            if not block:break
            sys.stdout.buffer.write(block)
        sys.stdout.buffer.flush();after=os.fstat(f.fileno())
        if after.st_size!=before.st_size or after.st_mtime_ns!=before.st_mtime_ns or not closed(run):raise ValueError('读取期间录包发生变化，请重试')

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['list','read']);p.add_argument('--path');p.add_argument('--size',type=int);p.add_argument('--mtime',type=int)
    a=p.parse_args();root=Path.home()/'drone_platform/01_apps/ego_planner/flight-records'
    if a.action=='list':print(json.dumps(catalogue(root),ensure_ascii=False))
    else:stream(root,a.path,a.size,a.mtime)
if __name__=='__main__':
    try:main()
    except Exception as e:print(str(e),file=sys.stderr);sys.exit(1)
