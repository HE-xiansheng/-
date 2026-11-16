#!/usr/bin/env python3
"""Recreate the resource figure from the archived actual result.json."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent
d=json.loads((root/'result.json').read_text());rows=d['resources']
times=[(r['wall_time_ns']-d['begin_ns'])/60e9 for r in rows]
busy=[]
for a,b in zip(rows,rows[1:]):
 delta=[y-x for x,y in zip(a['cpu'][:8],b['cpu'][:8])]
 busy.append(100*(sum(delta)-delta[3]-delta[4])/sum(delta))
fig,axes=plt.subplots(3,1,figsize=(9,7),sharex=True)
axes[0].plot(times,[max(r['temperature_c'].values()) for r in rows],label='Highest thermal-zone reading')
axes[0].set_ylabel('Temperature (C)');axes[0].legend(loc='lower right')
axes[1].plot(times[1:],busy,label='Whole-system CPU busy')
axes[1].set_ylabel('CPU busy (%)');axes[1].legend(loc='lower right')
for name in sorted(rows[0]['cpufreq']):
 axes[2].plot(times,[int(r['cpufreq'][name]['scaling_cur_freq'])/1000 for r in rows],label=name)
axes[2].set_ylabel('Sampled CPU (MHz)');axes[2].set_xlabel('Minutes after measurement start');axes[2].legend(loc='lower right')
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('RK3588: 600-second concurrent ROS2 workload\nReal camera/NPU, replay LiDAR, synthetic H264, recorder; no flight',fontsize=12)
fig.tight_layout(rect=(0,0,1,.93));fig.savefig(root/'resources.png',dpi=150)
