"""Metric/grid geometry only: no occupancy update, weight or motion sampling."""
from dataclasses import dataclass
import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap_angle(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi

@dataclass(frozen=True)
class Grid:
    origin: tuple = (-6., -5.)
    resolution: float = .1
    width: int = 120
    height: int = 100

    @classmethod
    def from_metadata(cls, metadata):
        return cls(**metadata['grid'])

    @property
    def shape(self):
        return self.height, self.width

    @property
    def extent(self):
        x, y = self.origin
        return x, x + self.width*self.resolution, y, y + self.height*self.resolution

    def indices(self, points):
        """Return row, column, in-bounds mask. Never clip invalid coordinates."""
        points = np.asarray(points)
        finite = np.isfinite(points).all(axis=-1)
        ij = np.floor((np.where(np.isfinite(points), points, -1e6) - self.origin)
                      / self.resolution).astype(np.int64)
        col, row = ij[..., 0], ij[..., 1]
        valid = finite & (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        return row, col, valid

    def centers(self, row, col):
        return np.stack((self.origin[0] + (np.asarray(col)+.5)*self.resolution,
                         self.origin[1] + (np.asarray(row)+.5)*self.resolution), axis=-1)


def endpoints(pose, ranges, angles, extrinsics=(0., 0., 0.)):
    """Return world sensor origin and endpoints. Angles increase counterclockwise."""
    x, y, theta = pose
    sx, sy, st = extrinsics
    origin = np.array([x + np.cos(theta)*sx-np.sin(theta)*sy,
                       y + np.sin(theta)*sx+np.cos(theta)*sy])
    headings = theta + st + np.asarray(angles)
    points = origin + np.asarray(ranges)[..., None] * np.stack((np.cos(headings), np.sin(headings)), axis=-1)
    return origin, points


def scan_cells(origin, ends, grid):
    """All positive-length traversed cells as (ray index, row, col).

    Split each segment at every grid-line crossing and sample interval midpoints.
    Zero-length corner touches are excluded. Includes the final interval's cell;
    callers decide whether a hit endpoint should receive occupied or free evidence.
    Returns only in-grid cells. No map values are read or changed.
    """
    ends = np.asarray(ends, dtype=float).reshape(-1, 2)
    if not len(ends):
        return tuple(np.empty(0, dtype=int) for _ in range(3))
    if not np.isfinite(origin).all() or not np.isfinite(ends).all():
        raise ValueError('Ray endpoints must be finite')
    delta = ends - origin
    crossings = []
    for axis, count in ((0, grid.width), (1, grid.height)):
        boundaries = grid.origin[axis] + np.arange(count+1)*grid.resolution
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (boundaries[None, :] - origin[axis]) / delta[:, axis, None]
        crossings.append(np.where((t > 0) & (t < 1), t, 1.))
    t = np.sort(np.concatenate([np.zeros((len(ends), 1)), *crossings,
                                np.ones((len(ends), 1))], axis=1), axis=1)
    widths = np.diff(t, axis=1)
    mid = (t[:, 1:] + t[:, :-1])*.5
    points = origin + mid[..., None]*delta[:, None, :]
    row, col, valid = grid.indices(points)
    valid &= (widths > 1e-12) & (np.linalg.norm(delta, axis=1)[:, None] > 1e-12)
    ray = np.broadcast_to(np.arange(len(ends))[:, None], mid.shape)
    return ray[valid], row[valid], col[valid]


def ray_cells(origin, end, grid):
    _, row, col = scan_cells(np.asarray(origin), [end], grid)
    return np.stack((row, col), axis=1)


def distance_field(occupied, resolution):
    """Distance in metres to occupied-cell centers; infinity for an empty mask."""
    occupied = np.asarray(occupied, dtype=bool)
    if not occupied.any():
        return np.full(occupied.shape, np.inf)
    return distance_transform_edt(~occupied, sampling=resolution)


def integrate_odometry(dataset):
    """Deterministic dead-reckoning baseline, with no sensor correction."""
    poses = np.zeros((len(dataset), 3))
    poses[0] = dataset.metadata['initial_pose']
    for t in range(1, len(dataset)):
        x, y, theta = poses[t-1]
        dx, dy, dt = dataset.odometry[t]
        poses[t] = [x + np.cos(theta)*dx - np.sin(theta)*dy,
                    y + np.sin(theta)*dx + np.cos(theta)*dy, wrap_angle(theta+dt)]
    return poses
