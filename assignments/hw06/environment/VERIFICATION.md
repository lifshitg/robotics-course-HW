# HW6 verification record

**Implemented draft: current-machine data, reference, display and PDF checks pass.
Fresh-OS installation and an independent student walkthrough remain untested.**

## Tested platform and dependencies

Recorded 2026-10-12 on Ubuntu 24.04 x86-64, Python 3.12.3, Intel Core i3-10110U
(two cores/four hardware threads), CPU-only execution. A new HW6 virtual environment
was created and the documented requirements/editable package installed. This is
not a fresh operating-system installation. `pip check` reports no broken requirements.

| Dependency | Version |
| --- | --- |
| NumPy | 2.2.6 |
| SciPy | 1.15.3 |
| Matplotlib | 3.10.8 |
| Pytest | 8.4.2 |
| Tk desktop backend | 8.6 |

## Dataset and privacy checks

- Both fixed datasets contain 500 scans, 181 beams, 2 Hz timestamps, turns and revisits.
  Each public sensor archive is approximately 650 KiB; the complete public data is
  about 1.3 MiB. No runtime sensor/world download is needed.
- Noise parameters and status semantics match the private generator. Development
  has 925 missing beams and 4,521 no-hit beams; assessment has 903 and 5,223.
- Development answers are public and separately loaded. Assessment contains only
  `metadata.json` and `sensors.npz`, whose keys are timestamps, odometry, angles,
  ranges and status. It contains no answer map, true trajectory or world definition.
- Sensor SHA-256 checksums, schema, frame conventions and initial pose pass checks.
  Assessment answers/generation code and complete reference remain under ignored
  `ta_solution/`; no TA file is tracked.

## Automated and reference verification

**18 automated checks pass:** nine public infrastructure/display checks and nine
private algorithm checks. They cover data contracts, grid/ray geometry, distance
units, evaluation without alignment, causal replay validation, stable weights,
missing/no-hit handling, mapping saturation, resampling, independent maps,
reproducibility, score-before-map order and coherent final ancestry.

All **twelve complete reference configurations** ran: two environments × 10/30
particles × seeds 0/1/2. An independent audit of saved outputs verifies finite
poses/maps, timestamp agreement, ESS bounds, causal histories, final-map/trajectory
coherence and recorded metrics. No numerical-weight recoveries occurred.

Development position RMSE (metres), without alignment:

| Estimate | Seed 0 | Seed 1 | Seed 2 | Mean ± sample SD |
| --- | ---: | ---: | ---: | ---: |
| Odometry only | 0.452 | 0.452 | 0.452 | Fixed measurements |
| 10 particles | 0.067 | 0.093 | 0.108 | 0.089 ± 0.021 |
| 30 particles | 0.079 | 0.101 | 0.070 | 0.083 ± 0.016 |

Both environments pass the private release criterion: 30-particle median RMSE
improves over odometry-only, and the reconstructed maps are useful. Assessment
answers and detailed accuracy results are kept in private verification notes.
More particles do not improve every seed. These results are context, not student
accuracy thresholds or a claim of robust performance on arbitrary real data.

Individual estimation runs took **18–81 seconds**. The twelve-run matrix took
**596.12 seconds (9 minutes 56 seconds)** including evaluation and figures, excluding
installation, tests, animation exports and PDF building. Some display/check work
ran alongside parts of the matrix. All runs are below the approximate five-minute
per-run target on this host; other laptop runtimes are not guaranteed.

## Public setup, visualization and documentation

- Both data demos and public checks passed in a copied public-only directory with
  its own data/support paths and no TA files. Public local documentation links resolve.
- A desktop Tk run processed 80 assessment scans and displayed evolving maps,
  particle positions, heading, trajectory prefix, timestamp, ESS and resampling.
- Development and assessment GIF exports each contain 51 causal snapshots.
  First/middle/final frames were reviewed for map orientation and progression;
  development truth overlays are labeled, and assessment has no answer overlay.
  Layout reserves space for the complete time/ESS title.
- The 10-page PDF builds with the setup appendix. Motion, likelihood, log-odds and
  ESS equations, workflow, commands, tables and page layout were inspected.
  The shared PDF builder extension preserves HW1/HW2/HW4 behavior.

## Limitations

Fresh Ubuntu installation and a walkthrough by someone unfamiliar with the task
have not been performed. Windows/macOS, ARM, VMs and other GUI backends are untested.
The synthetic static environments have known initialization and modest independent
noise. No scan matching, explicit loop-closure optimization, dynamic obstacles or
real-world sensor robustness is included. Keep HW6 draft until remaining release
review is completed. No grading weights or private backup were added.
