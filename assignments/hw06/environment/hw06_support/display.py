"""Shared display/replay: students supply estimates, maps and diagnostics."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from .geometry import Grid


class LiveDisplay:
    def __init__(self, metadata, truth=None):
        if truth is not None and metadata['name'] != 'development':
            raise ValueError('Answer overlays are development-only')
        self.grid=Grid.from_metadata(metadata); self.truth=truth
        self.fig,self.ax=plt.subplots(figsize=(9,7))
        self.image=self.ax.imshow(np.full(self.grid.shape,.5),origin='lower',extent=self.grid.extent,
                                  cmap='gray_r',vmin=0,vmax=1,interpolation='nearest')
        self.path,=self.ax.plot([],[],'tab:blue',label='selected particle history')
        self.particles=self.ax.scatter([],[],s=9,c='tab:orange',label='particles')
        self.pose,=self.ax.plot([],[],'ro',markersize=5,label='current pose')
        self.heading,=self.ax.plot([],[],'r-',linewidth=2)
        self.truth_line,=self.ax.plot([],[],'g--',label='development truth' if truth is not None else '_nolegend_')
        if truth is not None:
            # True map is a development-only static overlay, explicitly labeled as answers.
            self.ax.contour(truth['occupancy']>0,levels=[.5],origin='lower',extent=self.grid.extent,colors='green',linewidths=.6)
        self.ax.set(xlabel='X (m)',ylabel='Y (m)',aspect='equal'); self.ax.legend(loc='upper right')
        # Reserve space for the two-line time/ESS title added during updates.
        self.fig.tight_layout(rect=(0,0,1,.92))

    def update(self, index, timestamp, log_odds, trajectory, particles, ess, resampled, pause=.001):
        """Only pass a trajectory prefix and the map available at this scan."""
        if len(trajectory)!=index+1:
            raise ValueError('Trajectory must end at the displayed scan; no future poses')
        self.image.set_data(1/(1+np.exp(-np.asarray(log_odds))))
        self.path.set_data(trajectory[:,0],trajectory[:,1]); self.particles.set_offsets(particles[:,:2])
        x,y,theta=trajectory[-1]; self.pose.set_data([x],[y])
        self.heading.set_data([x,x+.35*np.cos(theta)],[y,y+.35*np.sin(theta)])
        if self.truth is not None:
            prefix=self.truth['trajectory'][:index+1]; self.truth_line.set_data(prefix[:,0],prefix[:,1])
        self.ax.set_title(f'Scan {index} | t={timestamp:.1f}s | ESS={ess:.1f} | resampled={bool(resampled)}\n'
                          'white=free, gray=unknown, black=occupied')
        self.fig.canvas.draw_idle()
        if pause>0: plt.pause(pause)

    def close(self):
        plt.close(self.fig)


def validate_replay(arrays, metadata):
    """Check shapes, finite data and causal prefixes before opening a window."""
    required={'timestamps','ess','resampled','snapshot_indices','snapshot_maps',
              'snapshot_trajectories','snapshot_particles'}
    if not required.issubset(arrays):
        raise ValueError('Replay archive lacks required arrays')
    times=arrays['timestamps']; indices=arrays['snapshot_indices']
    n=len(times); k=len(indices); grid=Grid.from_metadata(metadata)
    if (n<1 or times.shape!=(n,) or not np.isfinite(times).all()
            or not (np.diff(times)>0).all() or indices.shape!=(k,) or k<1
            or not np.issubdtype(indices.dtype,np.integer)
            or indices[0]!=0 or indices[-1]!=n-1 or not (np.diff(indices)>0).all()):
        raise ValueError('Invalid replay timestamps or snapshot indices')
    if (arrays['ess'].shape!=(n,) or not np.isfinite(arrays['ess']).all()
            or (arrays['ess']<1-1e-9).any() or arrays['resampled'].shape!=(n,)
            or not np.isin(arrays['resampled'],[0,1]).all()):
        raise ValueError('Invalid replay diagnostics')
    maps=arrays['snapshot_maps']; histories=arrays['snapshot_trajectories']; particles=arrays['snapshot_particles']
    if (maps.shape!=(k,*grid.shape) or histories.shape!=(k,n,3)
            or particles.ndim!=3 or particles.shape[0]!=k or particles.shape[1]<1
            or particles.shape[2]!=3 or not np.isfinite(maps).all()
            or not np.isfinite(particles).all()):
        raise ValueError('Invalid replay map/history/particle shapes or values')
    for history,index in zip(histories,indices):
        if not np.isfinite(history[:index+1]).all() or not np.isnan(history[index+1:]).all():
            raise ValueError('Replay histories must have finite prefixes and NaN future poses')


def replay(result_path, metadata, truth=None, delay=.03, save=None):
    """Replay saved causal snapshots, optionally write a GIF using Pillow."""
    from matplotlib.animation import FuncAnimation, PillowWriter
    with np.load(result_path,allow_pickle=False) as z:
        arrays={key:z[key] for key in z.files}
    validate_replay(arrays,metadata)
    if not np.isfinite(delay) or delay<=0:
        raise ValueError('Replay delay must be positive and finite')
    view=LiveDisplay(metadata,truth)
    def frame(k):
        index=int(arrays['snapshot_indices'][k])
        view.update(index,arrays['timestamps'][index],arrays['snapshot_maps'][k],
                    arrays['snapshot_trajectories'][k,:index+1],arrays['snapshot_particles'][k],
                    arrays['ess'][index],arrays['resampled'][index],pause=0)
        return []
    animation=FuncAnimation(view.fig,frame,frames=len(arrays['snapshot_indices']),
                            interval=delay*1000,repeat=False)
    if save:
        Path(save).parent.mkdir(parents=True,exist_ok=True)
        animation.save(save,writer=PillowWriter(fps=max(1,round(1/delay))))
        view.close()
    else: plt.show()


def save_final_figure(log_odds, trajectory, metadata, output, truth=None, odometry=None):
    view=LiveDisplay(metadata,truth)
    if odometry is not None: view.ax.plot(odometry[:,0],odometry[:,1],color='purple',label='odometry only')
    view.update(len(trajectory)-1,(len(trajectory)-1)/metadata['frequency_hz'],log_odds,trajectory,
                np.empty((0,3)),0,False,pause=0)
    view.ax.legend(loc='upper right'); view.fig.savefig(output,dpi=150); view.close()
