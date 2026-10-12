# Your HW6 implementation

Run from the repository root after the [setup guide](../environment/README.md):

```bash
source assignments/hw06/environment/.venv/bin/activate
unset PYTHONPATH
MPLBACKEND=Agg python assignments/hw06/my_solution/demo_data.py --dataset development --truth
MPLBACKEND=Agg python assignments/hw06/my_solution/demo_data.py --dataset assessment
python -m pytest -q assignments/hw06/tests
```

Add `--show` and remove `MPLBACKEND=Agg` for desktop plots. The demo inspects raw
scans and odometry; it supplies no SLAM solution. Assessment answers are absent.

Create your own Python estimator, plotting and replay entry points here. Use the
shared loader/geometry/display helpers, but implement particle motion, scan
weights, mapping, normalization and resampling yourself. Read
[instructions](../instructions.md), including the exact noise model and workflow.
Keep answer access outside your estimator, even on development data.

Document your root-directory commands here. Run both datasets at 10 and 30 particles
with seeds 0/1/2. Submit source, configurations, lightweight final maps/trajectories,
per-step diagnostics, figures, a 3–5-page report, and animation/recording links for
both environments. Report development accuracy; assessment accuracy is private.
Keep replay archives and recordings under ignored `build/hw06/` and include commands
to regenerate them. Use a PR title with every student's name.

For the live defense, show your assessment estimator processing scans and building
its map over time. Have saved replay ready if computation is slow. Display current
pose/heading, trajectory prefix, particles, map, timestamp, ESS and resampling.
No ROS, online simulation or GPU is needed. Every group member attends.
