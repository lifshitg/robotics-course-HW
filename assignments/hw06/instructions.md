# HW6: Offline Particle-Filter SLAM

**Draft:** consult the [verification record](environment/VERIFICATION.md) before distribution.

## Goal

Estimate both a robot's **2D trajectory** and an **occupancy-grid map** from noisy
odometry and LiDAR. The robot has already collected the data: your program reads
it sequentially, without ROS, Gazebo, a running robot, or a GPU. Offline does not
mean using future scans to solve earlier steps. You will implement the main
components yourself and show the map developing over dataset time.

Use a simplified **grid-based Rao–Blackwellized particle filter (RBPF)**. A
particle represents a possible robot trajectory, with its own occupancy map
conditioned on that trajectory. This is related to FastSLAM's factorization, but
uses grid cells rather than landmark IDs and landmark EKFs. No scan matching,
pose graph, separate loop-closure detector, or research-level proposal is required.
See [Grisetti, Stachniss and Burgard's grid-based RBPF paper](https://doi.org/10.1109/TRO.2006.889486)
for context; you need not implement its advanced techniques.

## Two datasets

Each environment has **500 scans**, at **2 Hz**, spanning **249.5 seconds**. Both
contain turns and revisits, but their static obstacle arrangements and routes
differ. The robot translates and turns in place; scans are instantaneous and have
no within-scan motion distortion.

- **Development:** sensor data, true trajectory and true occupancy map. Use the
  answers to debug, overlay results, and measure error.
- **Assessment:** sensor data only. Its map and trajectory answers are withheld.
  Use it for your report and live demonstration; instructors evaluate it privately.

Your estimator must receive **only sensors and metadata**, on both datasets.
Never initialize maps from the development answer, use true poses for prediction,
choose particles using true error, or tune with assessment answers. Load answers
only in separate development visualization/evaluation code.

### Arrays, time and frames

`load_dataset(name)` returns a `Dataset` with:

| Array | Shape | Meaning |
| --- | --- | --- |
| `timestamps` | `(500,)` | Scan times in seconds, strictly increasing |
| `odometry` | `(500, 3)` | Noisy body-frame increments `(dx, dy, dtheta)` |
| `angles` | `(181,)` | LiDAR beam angles in radians, increasing counterclockwise |
| `ranges` | `(500, 181)` | Distances in metres; missing readings are NaN |
| `status` | `(500, 181)` | 0: missing, 1: hit, 2: maximum-range/no-hit |

At index $t>0$, odometry describes motion **from scan $t-1$ to scan $t$**, expressed
in the robot frame at $t-1$. Compose it using the previous heading, then associate
the resulting pose with scan $t$. Index 0 has zero motion and initializes the map.
No velocity integration or timestamp interpolation is needed.

The map frame is fixed by the exact initial pose $(0,0,0)$. Positive X is the
initial forward direction; positive Y is left; headings increase counterclockwise.
Wrap angular differences to $[-\pi,\pi)$. The LiDAR is centered on the robot,
with zero yaw offset, as specified by metadata. The geometry helper also accepts
nonzero extrinsics so frames remain explicit.

Arrays use `grid[row, column]`: columns increase with X, rows with Y. Display with
`origin='lower'`. Storage bounds are $[-6,6)\times[-5,5)$ m, with 0.10 m cells
(shape `(100, 120)`). These are **storage limits, not known occupied walls**.
All cells begin unknown. Outside-grid endpoints must not be clipped into border
cells or used as negative NumPy indices.

### Noise: do not guess these parameters

The private generator adds independent, zero-mean Gaussian errors to each true
body-frame increment. The standard deviations are **0.01 m in dx, 0.005 m in dy,
and 0.005 rad in dtheta**, independent between timesteps and components. There
are no hidden persistent biases. Metadata contains these values. Even independent
small errors accumulate when composing a trajectory.

For the simplified motion proposal, sample a possible increment as
$\tilde u_t=u_t+\epsilon_t$, where
$\epsilon_t\sim\mathcal N(0,\operatorname{diag}(\sigma_x^2,\sigma_y^2,\sigma_\theta^2))$.
This is the assignment's proposal approximation, not a claim that measured
odometry is exact. Apply it for $t>0$; do not perturb the known starting pose.

LiDAR has 181 beams over $[-\pi/2,\pi/2]$, with range limits **0.10–8 m**. For a
true hit below maximum range, add Gaussian noise with **0.02 m standard deviation**,
then clip to the range limits. Each beam independently has **1% probability of
being missing**. Status is authoritative:

- **Missing (0):** skip it in scoring and mapping; NaN is not a distance.
- **Hit (1):** clear traversed cells before the endpoint and add occupied evidence
  at an in-grid endpoint. A clipped hit can equal maximum range and remains a hit.
- **No-hit (2):** the range is exactly 8 m. Clear the traversed in-grid ray, but
  do not create an occupied endpoint. Use it for mapping, not hit-endpoint scoring.

Physical range noise (0.02 m) is different from the larger likelihood-field
width below: map discretization and uncertain poses broaden endpoint agreement.

## Required sequential workflow

For each dataset and random seed:

1. Initialize independent unknown maps, uniform weights, and particles at the
   known initial pose. Store each particle's trajectory ancestry.
2. At the next scan, sample and compose motion for each particle.
3. Score hit endpoints against **that particle's previous map**.
4. Combine prior weights and scan scores; normalize stably.
5. Update each particle's map with the current scan.
6. Select an estimate and record its trajectory prefix, map, particles and diagnostics.
7. Compute effective sample size and systematically resample if needed.
8. Copy selected maps and histories independently; reset resampled weights uniformly.
9. Continue until the last scan. Save a final map and trajectory from the **same
   final particle lineage**; evaluate afterward where answers exist.

At scan 0, no map evidence exists: keep uniform weights and insert the first scan.
For subsequent scans, do not insert a scan before scoring it. That would let each
pose validate itself against information it just created.

### Motion prediction

For a sampled body increment and previous pose $(x,y,\theta)$, compose:

$$x'=x+\cos\theta\,\tilde{dx}-\sin\theta\,\tilde{dy},\qquad
 y'=y+\sin\theta\,\tilde{dx}+\cos\theta\,\tilde{dy},\qquad
 \theta'=\operatorname{wrap}(\theta+\tilde{d\theta}).$$

Students implement sampling and composition; the supplied deterministic odometry
integration is only the comparison baseline.

### Scan weights

Use a likelihood field: compute distances to occupied-cell centers in each
previous map, then score projected **hit endpoints**. Initially recommend every
sixth beam (indices 0, 6, ..., 180); filter this selection by status, not vice versa.

Classify occupied cells for scoring using $L>0.5$. For in-grid, previously observed
endpoint cells, use the nearest occupied-cell distance $d$ and the dimensionless
score

$$q(d)=0.95\exp\left(-\frac{d^2}{2\sigma_{\mathrm{score}}^2}\right)+0.05,
\qquad \sigma_{\mathrm{score}}=0.15\ \mathrm m.$$

Accumulate $\log w_i^{\mathrm{old}}+\sum_b\log q(d_{ib})$. This is a simplified
endpoint compatibility model, not the exact generative LiDAR density. The floor
prevents one disagreeing beam from making a particle impossible.

Unknown endpoint cells and outside-grid endpoints receive the floor score 0.05;
they must not be rewarded as perfect matches. Maintain a separate observed-cell
mask: log odds equal to zero can also result from conflicting observations. If a
particle has no occupied cells yet, skip its scan score. A scan with no selected
hits leaves weights unchanged. Do not use true geometry to calculate distances.

Subtract the maximum log weight before exponentiating and normalize. Check that
weights are finite, nonnegative and sum to one. If every candidate is invalid,
reset to uniform weights and record an explicit recovery diagnostic. Reject
non-finite poses; never silently accept NaNs.

### Occupancy mapping

Represent occupancy with log odds:

$$L(c)=\log\frac{p(c)}{1-p(c)},\qquad p(c)=\frac{1}{1+\exp(-L(c))}.$$

Start with $L=0$ (probability 0.5, unknown). Add **-0.4** for observed free cells,
**+0.85** for hit endpoint cells, and clamp to **[-5, 5]**. The prior log odds are
zero, so no prior correction term is needed. Mark updated cells as observed.
Use all valid beams for mapping. Apply at most one free/occupied update per cell
per scan; if hit and free evidence conflict in that scan, occupied takes precedence.
Do not clear hit endpoint cells as free before adding their occupied evidence.

The supplied ray geometry identifies cells intersected over positive-length ray
segments. Zero-length corner touches are excluded. It does not change map values.
Cells outside the map remain unrepresented; do not invent an occupied storage border.

### Resampling and estimates

$$N_{\mathrm{eff}}=\frac{1}{\sum_i w_i^2}.$$

Use systematic resampling when $N_{\mathrm{eff}}<N/2$. Implement the cumulative
weights and one random offset yourself. Resampling duplicates a particle's entire
map and history; copies must not share mutable arrays. Keep parent identity or
copied history so the final trajectory belongs to the final map.

For display, select the highest-weight particle before resampling. Its history
may differ from the previous selected history, which is normal. Do not concatenate
unrelated best poses into the final trajectory or average incompatible maps.

## Support code and library rules

The shared `hw06_support` package provides:

- `load_dataset`, with validation, and separate `load_development_truth`.
- `Grid`, endpoint projection, `ray_cells` and batched `scan_cells` geometry.
- `distance_field`, returning metre distances (infinity for an empty mask).
- `integrate_odometry`, a deterministic comparison baseline.
- `trajectory_metrics`, accepting answers explicitly after estimation.
- `LiveDisplay`, final-figure helpers and saved-snapshot replay.

See the [setup guide](environment/README.md) for API examples and root commands.
Allowed: NumPy, SciPy, Matplotlib, standard Python, ordinary logging and tests.
Not allowed: complete SLAM/filter pipelines, GMapping, Cartographer, imported
particle-filter algorithms, or copied complete solutions. Students implement
motion sampling, weights, occupancy updates, normalization and resampling.

## Experiments and visualization

Run **both datasets**, **10 and 30 particles**, and algorithm seeds **0, 1, 2**:
**twelve runs**. Reuse the fixed measurements; do not regenerate sensor noise.
Use the same other settings across datasets and particle counts. Save seed,
configuration, runtime and package versions. Additional experiments must be labeled.

Show an animated visualization during processing and allow replay of saved
results. At each displayed scan show:

- Evolving map: unknown gray, free white, occupied black.
- Current estimated pose and heading, trajectory prefix, and particle positions.
- Scan index or timestamp, effective sample size and resampling indicator.

Display only the map and trajectory available at that step. Development truth
map overlays are allowed and must be labeled; show true trajectory only up to the
current index. Assessment displays must contain no answer overlays. Replay must
use saved intermediate maps and histories, not progressively reveal the final map.
Snapshots every ten scans plus the first/final scan are sufficient for replay;
live estimates and diagnostics must still be computed for every scan. Playback
need not match the 2 Hz acquisition rate.

### Evaluation and report

On development, plot ground truth, odometry-only and SLAM trajectories together,
plus position error over time. Compute position RMSE, final position error, and
wrapped heading RMSE in radians **without alignment**; the initial frame is known.
Report each run and aggregate mean/sample standard deviation across seeds.

For both environments, provide final maps/trajectories, effective sample size and
resampling plots, runtime, settings, and a comparison of 10 versus 30 particles.
For assessment, discuss visible consistency, revisits and uncertainty; **do not
claim ground-truth trajectory accuracy**. Instructors measure it privately.

## Submission and live defense

Submit under `assignments/hw06/my_solution/`:

- Source, configurations, root-directory commands for processing, plotting and replay.
- Lightweight saved final trajectories, per-step diagnostics and final maps for all
  twelve runs. Include timestamps so instructor evaluation can match scans.
- Required figures, development metrics table, runtime and particle-count comparison.
- A short report (approximately 3–5 pages) explaining decisions, changes and their
  reasons, sensor models, scoring, mapping, resampling, failures and limitations.
- An animation or short recording for **each environment**, and setup-check results.

Store large intermediate maps, recordings and temporary outputs under ignored
`build/hw06/`. Export lightweight results/figures to the submission; include recording
links and commands that regenerate snapshots. Do not commit virtual environments.

For the live defense, run your assessment solution and show the map and trajectory
progressing over dataset time. Prepare saved replay as a fallback for slow execution.
All group members attend and explain the implementation. No online robot or simulator
is required. Follow the [course PR submission process](../../README.md#submitting-an-assignment),
with a title containing every student's name, for example:

```text
HW06 — Firstname Lastname, Firstname Lastname
```

There is no invented trajectory-error threshold for student grading. Assess
correctness, reproducibility, evidence and explanation; particle SLAM need not
improve every individual seed. Report failures honestly. Before submitting, check
frames, missing/no-hit handling, scoring before mapping, log-odds signs, finite
weights, independent resampled maps, trajectory ancestry, and absence of ground
truth in the estimator.
