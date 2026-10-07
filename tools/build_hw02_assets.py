#!/usr/bin/env python3
"""Draw physical rooms only; never import instructor planning algorithms."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'assignments/hw02/environment/src/hw02_sim'))
from hw02_sim.scenario import LAYOUTS,START,GOAL,ROBOT_RADIUS
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle

fig,axes=plt.subplots(1,3,figsize=(12,3.5),layout='constrained')
for ax,layout in zip(axes,LAYOUTS):
    ax.add_patch(Rectangle((0,0),8,6,fill=False,lw=3,color='black'))
    for xy in LAYOUTS[layout]:ax.add_patch(Circle(xy,.4,color='#b85838'))
    ax.add_patch(Circle(START,ROBOT_RADIUS,color='#3570b6'))
    ax.plot(*GOAL,'*',color='green',ms=12)
    ax.set(title=layout,xlim=(-.2,8.2),ylim=(-.2,6.2),xlabel='map X (m)',ylabel='map Y (m)')
    ax.set_aspect('equal');ax.grid(alpha=.15)
fig.savefig(ROOT/'assignments/hw02/assets/rooms.png',dpi=180)
plt.close(fig)
