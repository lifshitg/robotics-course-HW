"""Inspect raw data and a dead-reckoning baseline; no SLAM algorithm."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from hw06_support import load_dataset, load_development_truth, endpoints
from hw06_support.geometry import integrate_odometry


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',choices=['development','assessment'],default='development')
    parser.add_argument('--truth',action='store_true',help='development-only answer overlays')
    parser.add_argument('--show',action='store_true')
    parser.add_argument('--output',default='build/hw06/demo')
    args=parser.parse_args()
    if args.truth and args.dataset!='development': parser.error('Assessment answers are not provided')
    d=load_dataset(args.dataset); odom=integrate_odometry(d)
    print(f'{d.name}: scans={len(d)}, beams={len(d.angles)}, duration={d.timestamps[-1]:.1f}s')
    print(f'Odometry std [m,m,rad]: {d.metadata["odometry_std"]}; range std: {d.metadata["lidar"]["range_std"]}m')
    fig,axes=plt.subplots(1,2,figsize=(11,5))
    valid=d.status[0]!=0; _,points=endpoints([0,0,0],d.ranges[0,valid],d.angles[valid])
    axes[0].scatter(points[:,0],points[:,1],s=5); axes[0].set_title('Raw scan 0 (sensor frame)')
    axes[1].plot(odom[:,0],odom[:,1],label='odometry only')
    if args.truth:
        truth=load_development_truth(); poses=truth['trajectory']
        axes[1].plot(poses[:,0],poses[:,1],'g--',label='development truth')
        grid=d.metadata['grid']; x,y=grid['origin']; r=grid['resolution']
        axes[1].imshow(truth['occupancy'],origin='lower',extent=[x,x+r*grid['width'],y,y+r*grid['height']],cmap='gray_r',alpha=.3)
    for ax in axes: ax.set(xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    axes[1].legend(); fig.tight_layout(); output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    fig.savefig(output/f'{d.name}.png',dpi=140)
    if args.show: plt.show()
    plt.close(fig)
    print(f'PASS: loaded sensor data and saved {output/args.dataset}.png; no SLAM performed')

if __name__=='__main__': main()
