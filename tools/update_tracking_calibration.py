#!/usr/bin/env python3
"""从已通过的真实相机审计导出运行参数；不访问/启动ROS，不包含机体外参。"""
import argparse,json,math
from pathlib import Path
import numpy as np
import yaml

def update(report,config,serial):
    if report.get('passed') is not True or report.get('returncodes')!=[0,0] or not all(c.get('passed') for c in report.get('checks',[])):
        raise ValueError('相机/NPU审计或正常停止未通过')
    if not serial or not serial.isalnum():raise ValueError('请明确填写实际相机序列号')
    color=report['camera_info']['color'];depth=report['camera_info']['depth'];extrinsic=report['depth_to_color']
    p=config['/**']['ros__parameters']
    if (color['width'],color['height'])!=(p['camera_cols'],p['camera_rows']):raise ValueError('彩色分辨率与任务不一致')
    if (color['frame'],depth['frame'])!=('camera_color_optical_frame','camera_depth_optical_frame'):
        raise ValueError('光学坐标系未按实际部署命名；请先核对变换方向')
    R=np.asarray(extrinsic['R'],float);t=np.asarray(extrinsic['t'],float)
    if R.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(t).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-6) or abs(np.linalg.det(R)-1.)>1e-6 or np.linalg.norm(t)>.1:
        raise ValueError('无效的深度光学→彩色光学刚体变换')
    for prefix,info in [('',color),('depth.',depth)]:
        k=info['k']
        if len(k)!=9 or not all(math.isfinite(v) for v in k) or k[0]<=0 or k[4]<=0:raise ValueError('内参无效')
        for suffix,index in [('fx',0),('fy',4),('cx',2),('cy',5)]:p[('camera_' if not prefix else prefix)+suffix]=float(k[index])
    p['depth.extrinsic_rotation']=R.reshape(-1).tolist();p['depth.extrinsic_translation']=t.tolist()
    return '# Factory calibration for Gemini 335L '+serial+'; Pc = R_depth_to_color * Pd + t (m).\n'+yaml.safe_dump(config,sort_keys=False,allow_unicode=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--report',type=Path,required=True);parser.add_argument('--config',type=Path,required=True);parser.add_argument('--camera-serial',required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    a.output.write_text(update(json.loads(a.report.read_text()),yaml.safe_load(a.config.read_text()),a.camera_serial))
