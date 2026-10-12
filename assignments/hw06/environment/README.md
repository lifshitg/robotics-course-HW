# HW6 setup and data guide

Supported setup: **Ubuntu 24.04, Python 3.12, CPU**. No ROS, Gazebo, MuJoCo, GPU,
or external dataset download is needed. Sensor archives are included locally.
See [verification](VERIFICATION.md) for measured versions, runtimes and limitations.

## 1. Install from the repository root

Fork/clone the course repository and create your `hw06` branch using the
[course workflow](../../../README.md#forking-and-initial-setup). Deactivate any
existing virtual environment. Use a separate HW6 environment:

```bash
sudo apt update
sudo apt install python3-venv python3-pip python3-tk
python3 -m venv assignments/hw06/environment/.venv
source assignments/hw06/environment/.venv/bin/activate
unset PYTHONPATH
python -m pip install -r assignments/hw06/environment/requirements.txt
python -m pip install -e assignments/hw06/environment
python -m pip check
```

The editable install makes `hw06_support` importable in your own scripts. Dependencies
are NumPy 2.2.6, SciPy 1.15.3, Matplotlib 3.10.8 and Pytest 8.4.2. It supplies no
SLAM algorithm. Do not use `sudo pip` or source ROS for this assignment.

In **every new terminal**, from the root:

```bash
source assignments/hw06/environment/.venv/bin/activate
unset PYTHONPATH
python -c 'import sys, numpy, scipy, matplotlib, hw06_support; print(sys.executable); print(numpy.__version__, scipy.__version__, matplotlib.__version__)'
```

Expect the HW6 `.venv` interpreter and the pinned versions. NumPy archive loading
uses `allow_pickle=False`; no executable model or license is required.

## 2. Public readiness checks and data demo

```bash
python -m pytest -q assignments/hw06/tests
MPLBACKEND=Agg python assignments/hw06/my_solution/demo_data.py --dataset development --truth
MPLBACKEND=Agg python assignments/hw06/my_solution/demo_data.py --dataset assessment
```

Expect 500 scans, 181 beams and 249.5 seconds for each dataset, finite valid ranges,
a printed noise summary, and a PASS. Images go to ignored `build/hw06/demo/`.
The assessment demo shows only raw data and odometry. Neither demo performs SLAM.
Public tests check loading and geometry; passing them does not validate your filter.

For desktop viewing:

```bash
python assignments/hw06/my_solution/demo_data.py --dataset development --truth --show
python assignments/hw06/my_solution/demo_data.py --dataset assessment --show
```

Close the plot to finish. Headless computation requires no display. Matplotlib
animations need a working desktop backend; do not set `MPLBACKEND=Agg` for a live
window. The `python3-tk` system package provides the desktop Tk backend. GIF export
uses Pillow, installed with Matplotlib, and works headlessly.

## 3. Use the provided interfaces

```python
from hw06_support import load_dataset, Grid, endpoints, scan_cells, distance_field

sensors = load_dataset("assessment")  # no answer file is opened
spec = sensors.metadata
grid = Grid.from_metadata(spec)
# Access sensors.odometry[t], sensors.ranges[t], sensors.status[t], sensors.angles.
```

This snippet only illustrates data access. Students implement their own estimator.
See [instructions](../instructions.md) for complete noise and algorithm conventions.

| Interface | Inputs and output |
| --- | --- |
| `load_dataset(name_or_path)` | `development`, `assessment`, or a data-directory path; validated `Dataset` |
| `load_development_truth()` | Explicit development-only dictionary: trajectory, occupancy, timestamps |
| `Grid.indices(points)` | World `(…, 2)` points → rows, columns and valid mask; never clips |
| `Grid.centers(row, col)` | Cell indices → world cell centers |
| `endpoints(pose, ranges, angles, extrinsics)` | World sensor origin and world ray endpoints |
| `ray_cells(origin, end, grid)` | Ordered `(row, column)` cells crossed over positive length |
| `scan_cells(origin, ends, grid)` | Batched `(ray_index, row, column)` arrays; no map updates |
| `distance_field(occupied_mask, resolution)` | Metre distances to occupied-cell centers; infinity if empty |
| `integrate_odometry(dataset)` | Deterministic odometry-only trajectory; no corrections |
| `trajectory_metrics(estimate, truth)` | Position RMSE, final position error, heading RMSE |

`integrate_odometry` is in `hw06_support.geometry`; `trajectory_metrics` is in
`hw06_support.evaluation`. Unknown/occupied classification is your responsibility;
`distance_field` only accepts your Boolean mask. Ray cells include the cell of the
last positive-length segment; use hit endpoint indices to distinguish occupied
from free evidence. Endpoints outside the grid have a false valid mask.

### Visualization and replay contract

Use `hw06_support.display.LiveDisplay(metadata, truth=None)` and call:

```python
view.update(index, timestamp, log_odds, trajectory_prefix,
            particle_poses, ess, resampled)
```

Shapes: log odds `(100,120)`, trajectory prefix `(index+1,3)`, particles `(N,3)`.
The map and history must belong to the same selected particle at that timestep.
`update` uses a small GUI pause; call `view.close()` in cleanup. You can also write
your own display, as long as all required information is visible. To overlay
answers, pass `load_development_truth()` only for development.

The shared `replay(path, metadata, truth=None, delay=0.03, save=None)` accepts a
NumPy archive with these keys:

| Key | Shape / content |
| --- | --- |
| `timestamps`, `ess`, `resampled` | One value per processed scan |
| `snapshot_indices` | Increasing saved scan indices; include first and final |
| `snapshot_maps` | `(K,100,120)` selected-particle log-odds maps at those scans |
| `snapshot_trajectories` | `(K,T,3)` selected-particle histories; NaN after each snapshot index |
| `snapshot_particles` | `(K,N,3)` particle poses before resampling |

Save every tenth scan and the final scan. Store independent copies: saving array
references and mutating them later would reveal the final map at earlier times.
Use `save="build/hw06/replay.gif"` for headless GIF export. Live replay does not
execute the estimator; your processing program must support visualizing its own
intermediate results too.

Final submission archives should be lightweight: final `trajectory`, `timestamps`,
`log_odds`, and `observed` mask, plus per-step `ess`, `resampled`, and numerical
recovery flags. Record filenames and keys in your submission README. Save full
replay archives under ignored `build/hw06/`, and provide regeneration commands.

## Troubleshooting and restart

| Symptom | Check/action |
| --- | --- |
| `python` missing / imports fail | Activate this HW6 environment and use `python -m pip`. |
| `hw06_support` missing | Run the editable-install command from the root. |
| Wrong dataset path | Run root commands; name-based loading uses the repository data directory. |
| Missing NaNs cause errors | Mask missing status before geometry/scoring; do not replace it with zero distance. |
| Map mirrored or rotated | Check body/world frames, row/column ordering, angle signs and `origin='lower'`. |
| Strange border obstacles | Check valid masks; never clip endpoints into border cells. |
| Animation blank / non-interactive | Remove `MPLBACKEND=Agg` for desktop viewing; use GIF export if no display. |
| IDE display failure | Try a normal Ubuntu terminal; an IDE may inherit incompatible GUI settings. |
| No usable particle weights | Use log-space normalization and record recovery; inspect unknown-cell scoring. |
| Slow processing | Use batched ray geometry, beam subsampling for scoring, limited snapshots and no per-ray drawing. |
| Development truth missing | Check that `data/development/truth.npz` exists; assessment truth is intentionally absent. |

Ctrl+C stops scripts. Close figures in cleanup paths. Rerun with recorded seeds
for reproducibility; `deactivate` leaves the environment. Processing can run faster
or slower than dataset time. Other operating systems and fresh-OS installation
are unverified unless listed in the verification record.

## References and PDF

- [Grid-based RBPF SLAM paper](https://doi.org/10.1109/TRO.2006.889486)
- [NumPy archive loading](https://numpy.org/doc/stable/reference/generated/numpy.load.html)
- [SciPy Euclidean distance transform](https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.distance_transform_edt.html)
- [Matplotlib animation](https://matplotlib.org/stable/api/animation_api.html)

Instructors build the PDF; students do not need LaTeX:

```bash
sudo apt install pandoc texlive-xetex texlive-fonts-recommended fonts-dejavu-core
python3 tools/build_hw06_pdf.py
```

Output: ignored `build/hw06/hw06.pdf`, including this guide as an appendix.
