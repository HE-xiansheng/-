#!/usr/bin/env python3
"""Qt SSH stdin入口；委托已部署的原生ROS2管理器，不复制状态机。"""
import importlib.util
from pathlib import Path
import sys
path=Path.home()/'drone_platform/01_apps/fast_lio/manage_stack.py'
if not path.is_file():
    # 仅供电脑离线策略检查；机载始终优先使用部署文件。
    path=Path(__file__).resolve().parents[3]/'platform/01_apps/fast_lio/manage_stack.py'
spec=importlib.util.spec_from_file_location('native_localization_stack',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
ExistingStack=module.ExistingStack
operate=module.operate
locks=module.locks
if __name__=='__main__':
    try:module.main()
    except Exception as exc:
        import json
        print(json.dumps({'ok':False,'message':str(exc)},ensure_ascii=False),flush=True)
        sys.exit(1)
