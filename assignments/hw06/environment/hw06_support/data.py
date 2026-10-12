"""Sensor loading intentionally separate from answer loading."""
from dataclasses import dataclass
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
MISSING, HIT, NO_HIT = 0, 1, 2

@dataclass(frozen=True)
class Dataset:
    name: str
    metadata: dict
    timestamps: np.ndarray
    odometry: np.ndarray
    angles: np.ndarray
    ranges: np.ndarray
    status: np.ndarray

    def __len__(self):
        return len(self.timestamps)


def load_dataset(name_or_path):
    path = Path(name_or_path)
    if not path.is_dir():
        path = ROOT / 'assignments/hw06/data' / name_or_path
    meta = json.loads((path / 'metadata.json').read_text())
    with np.load(path / 'sensors.npz', allow_pickle=False) as z:
        expected = {'timestamps', 'odometry', 'angles', 'ranges', 'status'}
        if set(z.files) != expected:
            raise ValueError('Sensor archive must contain only the five documented arrays')
        arrays = {key: z[key].copy() for key in expected}
    d = Dataset(meta['name'], meta, **arrays)
    n, b = len(d), len(d.angles)
    if (n < 2 or d.timestamps.shape != (n,) or d.odometry.shape != (n, 3)
            or d.ranges.shape != (n, b) or d.status.shape != (n, b)):
        raise ValueError('Invalid dataset dimensions')
    if not (np.isfinite(d.timestamps).all() and (np.diff(d.timestamps) > 0).all()
            and np.isfinite(d.odometry).all() and np.isfinite(d.angles).all()
            and (np.diff(d.angles) > 0).all() and np.all(d.odometry[0] == 0)):
        raise ValueError('Invalid timestamps, angles or increments')
    if not np.isin(d.status, [MISSING, HIT, NO_HIT]).all():
        raise ValueError('Unknown range status')
    valid = d.status != MISSING
    if not (np.isfinite(d.ranges[valid]).all()
            and (d.ranges[valid] >= meta['lidar']['range_min']).all()
            and (d.ranges[valid] <= meta['lidar']['range_max']).all()
            and np.isnan(d.ranges[~valid]).all()
            and np.all(d.ranges[d.status == NO_HIT] == meta['lidar']['range_max'])):
        raise ValueError('Range values disagree with their status')
    return d


def load_development_truth():
    """Explicit evaluation-only API; cannot select assessment answers."""
    path = ROOT / 'assignments/hw06/data/development/truth.npz'
    with np.load(path, allow_pickle=False) as z:
        return {key: z[key].copy() for key in ('trajectory', 'occupancy', 'timestamps')}
