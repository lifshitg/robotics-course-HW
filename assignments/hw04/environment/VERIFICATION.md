# HW4 verification record

**Implemented draft: corrected reference experiments and sustained-motion checks
pass on the tested host. Instructor release review remains pending.**

## Tested setup

Recorded 2026-10-07 on Ubuntu 24.04 x86-64, Python 3.12.3, Intel Core i3-10110U
(2 cores/4 threads). A new isolated Python virtual environment was installed on
this host; this is not a fresh operating-system installation. No ROS, Gazebo,
GPU build or private helper package is used by HW4.

| Package | Tested version |
| --- | --- |
| Gymnasium | 1.2.3 |
| MuJoCo | 3.15.0 |
| PyTorch | 2.10.0+cpu (CUDA build: None) |
| NumPy | 2.2.6 |
| Matplotlib | 3.10.8 |
| Pytest | 8.4.2 |

`pip check` reports no broken requirements in the isolated environment. Clear
inherited ROS `PYTHONPATH` settings before testing if your shell was previously
used for ROS; the student setup explicitly uses a separate Python environment.

## Completed checks

- Two public checks pass: default shapes/bounds and 1,000-step truncation;
  reproducible reset/action sampling and trajectories.
- Eleven private checks pass: known return calculations; identical initial policies
  and sampling streams with/without value initialization; transformed action
  density agreement and bounded actions; independent policy/value gradients;
  deterministic evaluation without changes to parameters or training RNG; real
  1,000-step rollouts and finite, nonzero updates for both methods; two short
  complete training runs produce identical evaluation results and final parameters;
  batch integration checks verify episode boundaries, unchanged policy during
  collection, one actor update per batch and the configured critic-update count.
- The random-action demo passes with finite returns. Seed 0 gives 10.434 and
  the second seeded episode gives 11.376 on this combination. These are examples,
  not reward acceptance thresholds.
- Public checks/demo copied to a separate directory without TA files also pass.

## Original experiment (superseded)

All six runs completed at 100,000 training steps each, with the default 1,000-step
horizon and 0.99 discount. Total measured matrix time was **296.40 seconds
(4 minutes 56 seconds)**, including initial/final evaluation. Each policy receives
20 evaluation episodes in addition to its training budget: 720,000 simulator steps
for the full matrix. PyTorch used one CPU thread. Package installation and PDF
building are outside that measurement. This is a measured laptop example, not a
maximum runtime promise.

| Method | Seed | Initial evaluation mean | Final evaluation mean | Final episode sample SD | Run seconds |
| --- | --- | --- | --- | --- | --- |
| REINFORCE | 0 | 11.96 | 20.87 | 6.16 | 36.57 |
| REINFORCE | 1 | -13.12 | 22.24 | 5.01 | 59.34 |
| REINFORCE | 2 | 4.63 | 20.11 | 4.33 | 52.39 |
| With baseline | 0 | 11.96 | 21.49 | 5.57 | 47.91 |
| With baseline | 1 | -13.12 | 22.67 | 4.12 | 52.17 |
| With baseline | 2 | 4.63 | 20.78 | 3.45 | 47.22 |

Each evaluation uses stochastic actions and reset seeds 10000–10009. Initial
results are exactly identical for each paired training seed, not just equal after
rounding. Final aggregation over three seed-level means gives **21.07 ± 1.08**
for REINFORCE and **21.64 ± 0.95** with the baseline (sample standard deviation,
not a confidence interval). Training curves are noisy and the improvement from
the baseline is small. This does not establish a general variance-reduction or
performance advantage. Independent motion checks subsequently showed that the original baseline folds
into a nearly stationary posture. Its last-half forward speed is -0.0076 m/s,
despite a positive episode score. The 100,000-step/0.99 protocol is superseded;
passing numerical tests did not establish a good swimming policy.

Raw CSVs, evaluation JSON, checkpoints and unsmoothed figures are in ignored
`build/hw04/reference-original-100k/`. These local artifacts include instructor output; they
are not student starter files.

## Corrected experiment and motion checks

The instructor authorized a revised student protocol after the original policy's
poor movement was observed. The required comparison now uses **gamma 1,
500,000 training steps per seed, ten complete episodes per actor update, and
20 Monte Carlo value fits per batch**. Both methods remain REINFORCE from scratch.
No PPO, TD, GAE, normalization, entropy bonus or external RL algorithm was added.
The reference also uses centered initial policy means and appropriate optimizer
settings; its paired initial parameters are exactly identical.

The final evaluation uses **fresh reset seeds 30000–30009**. Original resets
10000–10009 were used in development and are no longer presented as held-out
final evidence. Trained checkpoints were not changed during reevaluation.

| Method | Seed | Initial mean | Final mean | Final episode sample SD | Mean X displacement | Last-half mean forward speed |
| --- | --- | --- | --- | --- | --- | --- |
| REINFORCE | 0 | 0.47 | 99.23 | 17.13 | 3.97 m | 0.101 m/s |
| REINFORCE | 1 | -0.07 | 145.98 | 11.03 | 5.84 m | 0.142 m/s |
| REINFORCE | 2 | 1.84 | 82.86 | 7.56 | 3.32 m | 0.074 m/s |
| With baseline | 0 | 0.47 | 111.20 | 7.70 | 4.45 m | 0.112 m/s |
| With baseline | 1 | -0.07 | 177.31 | 10.82 | 7.10 m | 0.189 m/s |
| With baseline | 2 | 1.84 | 169.75 | 9.49 | 6.79 m | 0.178 m/s |

Across the three seed-level means: REINFORCE **109.36 ± 32.76**, baseline
**152.75 ± 36.19** (sample SD, not confidence intervals). These results show
meaningful continued forward movement, not just an initial thrust. The baseline
improves this particular three-seed comparison; no universal or statistically
conclusive superiority is claimed and student grading has no score threshold.

An independent checkpoint audit repeats ten seeded rollouts per policy, verifies
saved evaluation returns, measures world position/late velocity and checks paired
initial parameters, seeds and budgets. All six pass. Stronger methods on older
Swimmer versions have reported scores above 300; see
[the Swimmer study](https://arxiv.org/pdf/2208.07587). Those are not directly
comparable targets for this limited-budget REINFORCE exercise.

### CPU workload

The six-run matrix took **995.83 seconds**; the fresh final evaluation added
**42.34 seconds**, for **1,038.17 seconds (17 minutes 18 seconds)**. That measured
workload includes an extra evaluation pass retained from development; a normal
reproduction uses only the final reset set. The required protocol is 3,000,000
training steps plus 120,000 initial/final evaluation steps. Independent motion
verification, installation and PDF building are outside the reported matrix time.
The same i3-10110U laptop and one PyTorch thread were used; other laptop runtimes
are not guaranteed. The larger budget remains below the approximate 30-minute
CPU target on this host.

Corrected outputs are in ignored `build/hw04/reference/`; original 100,000-step
results and diagnostic trials are preserved in separate ignored directories.
A verified corrected private source archive is kept under
`build/hw04/private-backups/`; an off-machine private backup remains an instructor
responsibility.

## Instruction and display review

- The optional desktop random-action viewer completed a full episode successfully.
  A libdecor warning reported fallback to an undecorated window; it did not prevent
  execution. The corrected default baseline checkpoint also completed a fixed-
  camera desktop demonstration with 4.17 m displacement and 0.092 m/s late speed
  on its development demonstration reset. Headless training uses no display.
- The revised handout matches the corrected reference: gamma-1 finite horizon,
  averaged summed episode losses, separate per-episode returns, pre-fit baseline,
  repeated critic fits, seeds, budgets and isolated final evaluation.
- The updated 11-page PDF builds with its setup appendix. Equations, workflow,
  experimental requirements and installation commands were visually inspected;
  inline command spaces and PR-title formatting are preserved. The HW4-specific
  code filter leaves the HW1/HW2 PDF formatting unchanged.
- After the submission update, all 13 tests passed again (13.71 seconds), and
  the public demo and two public checks passed from a temporary directory without
  TA files. `pip check` found no broken requirements, and public local documentation
  links resolved. The PDF's figure definitions, seed-level statistical summaries,
  both-method live-demo requirements and reward guidance were checked against the
  current instructions; submission-page layout was visually inspected.
  The public desktop demo also completed all 1,000 steps again with a PASS after
  granting desktop access outside the sandbox; the same decoration fallback
  warning was harmless. Rendering inside the sandbox could not connect to Wayland.
- All public local documentation links resolve. TA sources, the Python virtual
  environment, checkpoints, PDF and private backup are ignored; no TA file is tracked.

## Limitations and instructor release review

A fresh Ubuntu installation and independent student walkthrough have not been
performed. Windows/macOS, ARM systems, VMs and remote rendering are untested.
The installed CPU wheel command is tested on Ubuntu x86-64. No guaranteed reward,
statistical superiority or cross-platform bitwise reproducibility is claimed.
Confirm deadlines and grading weights before distribution and retain a permanent
private backup outside this checkout.
